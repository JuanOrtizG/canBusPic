#include "18F4580.h"
#fuses NOWDT,NOPROTECT,NOLVP,NOWDT
#use delay(crystal=16MHz)
#use rs232(UART1,baud=9600)

//VELOCIDAD DEL BUS CAN
#define Set_500K_Baud TRUE
#include "can-18xxx8.c"

/*BUG*/
//#define CAN_DO_DEBUG FALSE
/*DEFINICIONES*/
//#define PID 0x42
#define REQUEST 0x7DF

/*LIBRERIAS AUXILIARES OBD*/
//#define "obd2.c"

/*VARIABLES GLOBALES*/
int i;
int32 value=0;
int cnt = -1; 
int PID = 0x0d; 

int16 rpm=0;
int16 vs = 0;
int16 bat=0; 
int16 fuel=0;
int16 registro = 0;
signed int16 oiltemp =0;
/*################## OBTENEMOS EL PID DEL SERVIDOR #######################################*/
//VARIABLES GLOBALES PARA OBTENER EL PID DEL SERVIDOR
int valor; 
int16 ayuda=0; 
int rcvPID = 0x0d; 

void procesar (int vector[]){
  int cnt_x = 0; 
  
 while(cnt_x<3){
  valor = vector[cnt_x];//almaceno el primer valor, luego el siguiente
  cnt_x++;
    
   if(valor>='0' && valor<='9') {
      ayuda = ayuda*0x10; 
      ayuda = ayuda + valor -'0'; 
   }
   else if (valor>='A' && valor<='F'){
      ayuda=ayuda*0x10;
      ayuda = ayuda + 10 + valor -'A';
   }
   else if (valor>='a' && valor<='f'){
      ayuda=ayuda*0x10;
      ayuda = ayuda + 10 + valor -'a';
   }
   else if (valor =='t'){
      rcvPID = ayuda; 
   }
 }//end while cnt_x
} //end f-procesar

/*################## TRATAMIENTO DE INTERRUPCION #######################################*/
//VARIABLES GLOBALES DEL TRATAMIENTO
int recivido = 0;
const int len=5;
int buff[len]="0ctx"; 
int control=0;

#int_rda
void serial_isr() {                    // Interrupción recepción serie USART
   recivido = getc();                     //borra la bandera de interrupción al extraer el dato
   buff[control++]=recivido;                  
   if (control==4){control=0; buff[len-1]=0x00;}//aquí: buff[cnt]=0x00 caracter NULL                 //limite de datos recibidos
   if (recivido=='i'){control=0;}// 0x7A es la letra z(zeta) minuscula, reinicia el conteo
}

/*MAIN*/
void main(){
   /*REGISTROS DEL TRANSMISOR*/
   int out_data[8];
   int32 tx_id=PID;
   int1 tx_rtr=0;//con rtr = 1 no funciona la recepción como se esperaría ¿que es rtr?
   int1 tx_ext=0;
   int tx_len=8;
   int tx_pri=3;
   
   //REGISTROS DEL RECEPTOR
   struct rx_stat rxstat;
   int32 rx_id;
   int in_data[8];
   int rx_len;
  
  //RESETEO DE LOS REGISTROS DE DATOS DEL BUS
   int i;
   for (i=0;i<8;i++) {
      out_data[i]=0;
      in_data[i]=0;
   }
  
   //INICIALIZAR MÓDULO CAN
   can_init(0);                     //inicializa el módulo con ID standar 11 bits
   enable_interrupts(INT_RDA);      //Activa la interrupción serial
   enable_interrupts(GLOBAL);       //Activa las interrupciones en el PIC
   
     
   //CONFIGURACIÓN DE LOS FILTROS
   
   can_set_mode(CAN_OP_CONFIG); //must be in confi g mode //before params can be set
   can_set_id(RX0MASK,0x7FC,FALSE);
   can_set_id(RX0FILTER0,0x7E8,FALSE);
   can_set_id(RX0FILTER1,0x7E8,FALSE);
   
   can_set_id(RX1MASK,0x7FC,FALSE);
   can_set_id(RX1FILTER2,0x7E8,FALSE);
   can_set_id(RX1FILTER3,0x7E8,FALSE);
   can_set_id(RX1FILTER4,0x7E8,FALSE);
   can_set_id(RX1FILTER5,0x7E8,FALSE);
   can_set_mode(CAN_OP_NORMAL);
   
   //printf("\r\nRunning...");
   
    //CARGAMOS LA PETICIÓN DE DATOS y el ID CORRESPONDIENTE
   out_data[0]=0x02;
   out_data[1]=0x01;
   out_data[2]=PID;  
   tx_id = REQUEST;
   
 while(TRUE){
 
   
   procesar (&buff); //actualizo el rcvPID
   out_data[2]=rcvPID; //guardo en memoria el rcvPID
   /*
   cnt++;
   switch(cnt){
      case 0 : out_data[2] = 0x0d;  break; //
      case 1 : out_data[2] = 0x0c;  break; //
      case 2 : out_data[2] = 0x5e;  break; //fuel rate
      case 3 : out_data[2] = 0x42;  
               cnt=-1; 
               break;
               
      default: break; 
   }
   */
   
   
   //ESCRIBIMOS LOS DATOS EN EL BUS
   if ( can_tbe()){        
     i=can_putd(tx_id, out_data, tx_len,tx_pri,tx_ext,tx_rtr); //put data on transmit buffer
     if (i != 0xFF) { 
     /*
         printf("\r\n Enviando datos... ");
          printf("\r\n    DATA = ");
           for (i=0;i<tx_len;i++) {
              printf("%X ",out_data[i]);
            }
      */
     } 
     else {
      //printf("\r\nFAIL on PUTD\r\n");
     }
   }
   
   //ESPERAMOS Y RECIBIMOS LOS DATOS
   
   if ( can_kbhit() ){
        if( can_getd(rx_id, &in_data[0], rx_len, rxstat) ){
        
         //printf("\r\nGOT: BUFF=%U ID=%LX LEN=%U OVF=%U ", rxstat.buffer, rx_id, rx_len, rxstat.err_ovfl);// %U, %LU 
        // printf("FILT=%U RTR=%U EXT=%U INV=%U", rxstat.filthit, rxstat.rtr, rxstat.ext, rxstat.inv);
         //printf("\r\n    DATA = ");
           for (i=0;i<rx_len;i++) {
             // printf("%X ",in_data[i]);
            }
        }
        
        //CONVERCIÓN A VALORES NORMALIZADOS
    /*PARA VERIFICAR LOS PID*/
     #define A in_data[3]
     #define B in_data[4]
     #define C in_data[5]
     #define D in_data[6]
     #define uint32_t unsigned int32
     #define int16_t int16
     
      switch (in_data[2]) { //in_data[2] es el pid
              
       case 0x21: //DISTANCE_TRAVELED_WITH_MIL_ON:
                  registro= (A * 256.0 + B);
                  break;
   
       case 0x2F: //FUEL_TANK_LEVEL_INPUT:
                  registro= (A / 2.55);
                  break;
      
       case 0x5C: //ENGINE_OIL_TEMPERATURE:
                  oiltemp= (A - 40.0);
                  break;
      
       case 0x0D: //VEHICLE_SPEED:
                  vs= (A);
                  registro = vs; 
                  break;
   
       case 0x0C: //ENGINE_RPM:
                  rpm= ((A * 256.0 + B) / 4.0);
                  registro=rpm;
                  break;
         
       case 0x5e : //ENGINE_FUEL_RATE:
                  fuel= ((A * 256.0 + B) / 20.0);
                  registro = fuel;
                  break;
          
       case 0x42: //CONTROL_MODULE_VOLTAGE:
                  bat= ((A * 256.0 + B) / 1000.0);
                  registro=bat;
                  break;
         
       default:   break;
     }

   
   //printf("%Lu_%Lu_%Lu_%U\r\n",rpm,vs,bat,fuel);
   //tipo de respuesta, si es dashboard o solo un dato. 
   if (buff[3]=='x'){
      printf ("{\"rpm\":%Ld,\"vs\":%Ld,\"bat\": %Ld,\"fuel\": %Ld,\"valor\": %Ld}\r\n",rpm,vs,bat,fuel,registro); 
   }
   else if (buff[3]=='y'){
      printf("{\"valor\": %Ld}\r\n", registro);
   } 
   else if (buff[3]=='z'){
      printf( "{\"A\":\"%x\", \"B\":\"%x\", \"C\":\"%x\", \"D\":\"%x\"}\r\n",A,B,C,D );
   }
   else if (buff[3]=='w'){
      printf("{\"valor\": %Ld}\r\n", oiltemp);
   }
   
        
        
   }//END can_kbhit()
     // else{ printf("\n\r no hay interrupcion"); }
   
   
   //printf("\r\n");
   
   
  
  //RETARDAMOS mili SEGUNDOs
   delay_ms(2000);
 }//while  
   
}
