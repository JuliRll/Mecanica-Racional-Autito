/*
Nombre: Test de motores y control del heartbeat
Comentario: Antes de probar controlar los pines de los motores
*/

#include "mbed.h"

#define INTERVAL    500 /*Periodo para cambiar el estado del heartbeat*/
#define TRUE    1
#define FALSE   0
#define CHANGE  4   /*Esto es para el heartbeat*/


/*Zona de definicion de prototipos*/
void togleLed();

/*Zona de definicion de variables globales y pines*/
DigitalOut LED(PC_13);
Timer myTimer;
uint8_t estado=0;
int tiempoMs=0;

///////////////////////
/*pines para el motor 2 DERECHO*/
PwmOut velM2(PA_8); /*Der fisico*/
DigitalOut N3(PB_7);
DigitalOut N4(PB_6);


/*Pines para el motor 1 IZQUIERDO*/
PwmOut velM1(PB_5); /*Izq fisico*/
DigitalOut N1(PB_15);
DigitalOut N2(PB_14);
//////////////////////


int main(){
    
    int bandera = FALSE;
    LED = 0;
    
    myTimer.start();


    /*Logica para el motor 1 izq*/
    N1 = 1;
    N2 = 0;
    
    /*Loica para el motor 2 der*/
    N3 = 1;
    N4 = 0;   

    /*Definicion de la tension para los motores*/
    velM1.period_us(1000);
    velM2.period_us(1000);

    velM1.pulsewidth_us(1000);
    velM2.pulsewidth_us(1000);
 

    while(1){

        if((myTimer.read_ms()-tiempoMs)>INTERVAL){
           tiempoMs=myTimer.read_ms();
           estado++;
           if(estado==CHANGE){
               estado=FALSE;
               
               if (bandera==FALSE)
               {
                   bandera=TRUE;
               }else{
                   bandera=FALSE;
               }
               
           }
           if(bandera==FALSE){
               togleLed();
           }
        }
    	
    }
}

void togleLed(){
    LED=!LED;
}
