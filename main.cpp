//Prueba de pull 1 12.8

#include "mbed.h"
#include <stdbool.h>



#define INTERVAL             100

#define ULTRASONIDOINTERVAL  200

#define PERIOD               200

#define HEARTBEATINTERVAL    3000

#define VALORLINEA           25000
//Estados modo
#define IDLE                 0     
#define GOTO_OBJECT          1     
#define FOLLOW_LINE          2     
#define MAZE                 3 
//Estados modo 2 y 3
#define AHEAD                1
#define TURN_RIGHT           2    
#define TURN_LEFT            3
#define FOLLOW_OBS           4  
#define TURN_BACK            5
#define ANALIZE              6
#define WIN                  7


/**
 * @brief Enumeracion de posibles estados del boton para la maquina de estados 
 * 
 */
typedef enum{
    DOWN, //0
    UP,   //1
    RISING, //2
    FALLING, //3      
 } _eFlanco;

_eFlanco flanco;

/**
 * @brief Estructura de varaibles del boton
 * Almacena el estado del boton, cuando fue presionado y el tiempo que se mantuvo presionado
 */
typedef struct{
    uint8_t state;
    int timeDown;
    int timeDif;
}_ePulsador;

_ePulsador boton;


/**
 * @brief Enumeración de eventos del botón
 * Se usa para saber si el botón está presionado o nó
 */
typedef enum{
    EV_PRESSED,
    EV_NOT_PRESSED,
    EV_NONE
}_eButtonEvent;


/**
 * @brief Enumeración de la MEF para decodificar el protocolo
 * 
 */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

/**
 * @brief Enumeración de la lista de comandos
 * 
 */
 typedef enum{
        ACK=0x0D,
        ALIVE=0xF0,
        FIRMWARE = 0xF1,
        GET_IR = 0xA0,
        SET_POWER = 0xA1,
        SET_SERVO = 0xA2,
        GET_DISTANCE = 0xA3,
        GET_SPEED = 0xA4
    }_eID;

/**
 * @brief Estructura de datos para el puerto serie
 * 
 */
typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t cheksumtx;       //!< Cheksumm Tx
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[256];   //!< Buffer circular de recepción
    uint8_t bufferTx[256];   //!< Buffer circular de transmisión
    uint8_t payload[32];     //!< Buffer para el Payload de datos recibidos
}_sDato ;

volatile _sDato datosComProtocol;

/**
 * @brief Mapa de bits para implementar banderas
 */
typedef union{
    struct{
        uint8_t checkButtons :1;
        uint8_t servoMove: 1;
        uint8_t in1: 1;
        uint8_t in2: 1;
        uint8_t in3: 1;
        uint8_t in4: 1;
        uint8_t status:1;
        uint8_t servoAngle:1;
        uint8_t launchControl:1;
        uint8_t rotateFlag: 1;
        uint8_t moved: 1;
        uint8_t turn: 1;
    }individualFlags;
    uint8_t allFlags;
}_bGeneralFlags;

volatile _bGeneralFlags myFlags;

uint8_t heartBeatEvent;

/**
 * @brief Unión para descomponer/componer datos mayores a 1 byte
 * 
 */
typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
    int8_t i8[4];
}_udat;

_udat myWord;

/*************************************************************************************************/
/* Prototipo de Funciones */

/**
 * @brief Función que se llama en la interrupción de recepción de datos
 * Cuando se llama la fucnión se leen todos los datos que llagaron.
 */
void onDataRx(void);

/**
 * @brief Decodifica las tramas que se reciben 
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(void);

/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(void);

/**
 * @brief Envía los datos a la PC
 * La función consulta si el puerto serie está libra para escribir, si es así envía 1 byte y retorna
 */
void sendData(void);

/**
 * @brief Función que se llama con la interrupción del TICKER
 * maneja el tiempo de debounce de los botones, cambia un falg cada vez que se deben revisar los botones
 */
void OnTimeOut(void);

/**
 * @brief Funcion que calcula el ancho de pulso para posicionar el servo en el angulo correspondiente
 * 
 */
void servoMove(void);

/**
 * @brief Funcion que envia la onda
 * 
 */
void startEcho(void);

/**
 * @brief Funcion que recibe la onda
 * 
 */
void endEcho(void);

/**
 * @brief Funcion que corta el pulso enviado
 * 
 */
void myTrigger(void);

/**
 * @brief Funcion que almacena los datos de los sensores cada 500ms
 * 
 */
void getSpeed(void);

/**
 * @brief Funcion que aumenta un contador cada vez que el sensor de horquilla izquierdo detecta un pulso
 * 
 */
void speedMoment1(void);

/**
 * @brief Funcion que aumenta un contador cada vez que el sensor de horquilla derecho detecta un pulso
 * 
 */
void speedMoment2(void);

 /**
  * @brief Funcion que actualiza el estado del boton 
  * 
  */
void updateMef(void);

/**
 * @brief Eventos de prueba de motores y servo
 * 
 */
void presentacion(void);

/**
 * @brief Funcion que lee el valor de los sensores de linea cada 100ms
 * 
 */
void getIR(void);

/**
 * @brief Funcion que gira el auto hacia la derecha sobre su eje
 * 
 * @param pulsos cantidad de pulsos que deben girar las ruedas
 */
void pulse(uint8_t pulsos);

/*****************************************************************************************************/
/* Configuración del Microcontrolador */

//Led de heartbeat
DigitalOut HEARBEAT(PC_13);

//Pin Servo
PwmOut SERVO(PB_10);

//Pines sensores de horquilla
InterruptIn HR(PB_8);
InterruptIn HL(PB_9);

//Pines puente H para motores
PwmOut MOTOR1(PA_8), MOTOR2(PB_5);
DigitalOut IN1(PB_14),IN2(PB_15),IN3(PB_7),IN4(PB_6);

//Pines sensor infrarrojo
DigitalOut TRIGGER(PB_13);
InterruptIn ECHO(PB_12);

//Pines sensores de lineas
AnalogIn IRR(PA_0);
AnalogIn IRL(PA_1);

//Puerto Serie
Serial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

//Pulsador
DigitalIn BUTTON(PB_4);

//Timers para funciones
Timer miTimer, echoTimer, servoTimer; 

//Tickers para funciones
Ticker timerGeneral, speedTicker, IRticker;
Timeout pulsoUltrasonido;

//Variables de protocolo
int8_t anguloServo, contAngulo;
uint16_t valueIRR, valueIRL; //Lecturas de sensores d linea
uint32_t distance_us, speedM1, speedM2;
int32_t powerM1, powerM2; //Potencia de los motores
 
//Variables auxiliares
uint8_t contServo2 = 0, debounceTrigger, modeState, direction, mazeState, randomDirec;
uint16_t auxAngulo; //Ancho de pulso del servo
uint32_t comp = 0, hearbeatTime = 0, tiempo = 100, sum = 0,sequenceTime = 0, servoTime = 0, contGiro = 0, contServo = 0, auxCont = 0; //Comparadores de tiempo para intervalos
uint32_t distanciaIzq, distanciaDer, distanciaAde, distanciaPared; //Distancia y velocidades leidas
volatile uint32_t contSpeed1, contSpeed2; //Contadores para almacenar las velocidades

/*****************************************************************************************************/
/************  Función Principal ***********************/

int main()
{
    //Iniciar modos
    modeState = IDLE;
    mazeState = AHEAD;
    direction = AHEAD;

    //Iniciar el boton en UP
    boton.state = UP;

    heartBeatEvent = 0;
    debounceTrigger = ULTRASONIDOINTERVAL;

    //Inicializacion de banderas
    myFlags.individualFlags.checkButtons = false;
    myFlags.individualFlags.servoMove = false;
    myFlags.individualFlags.status = false;
    myFlags.individualFlags.servoAngle = true;
    myFlags.individualFlags.launchControl = false;
    myFlags.individualFlags.rotateFlag = true;
    myFlags.individualFlags.moved = true;

    //Inicio de timers
    miTimer.start();
    echoTimer.start();
    servoTimer.start();
    srand(miTimer.read_us());

    //Interrupciones
    speedTicker.attach_us(&getSpeed, 500000);
    timerGeneral.attach_us(&OnTimeOut, 50000);
    IRticker.attach_us(&getIR, 10000);
    
    //Conexion de puerto serie
    pcCom.attach(&onDataRx,Serial::RxIrq);

    //Lectura de las horquillas
    HL.rise(&speedMoment1);
    HR.rise(&speedMoment2);

    //Inicializacion funciones del ultrasonido
    ECHO.rise(&startEcho);
    ECHO.fall(&endEcho);
    //Inicializacion periodo del servo
    SERVO.period_ms(20);
    //Inicializacion periodo de motores
    MOTOR1.period_us(1000);
    MOTOR2.period_us(1000);
    
    //Probar componentes
    presentacion();

    while(true)
    {
        //Hearbeat
        if(modeState == IDLE){
            if ((miTimer.read_ms()-hearbeatTime) >= INTERVAL){
                hearbeatTime=miTimer.read_ms();
                HEARBEAT = !HEARBEAT;
            }
            //Setear motores
            MOTOR1.pulsewidth_us(powerM1);
            MOTOR2.pulsewidth_us(powerM2);
            IN1 = myFlags.individualFlags.in1;
            IN2 = myFlags.individualFlags.in2;
            IN3 = myFlags.individualFlags.in3;
            IN4 = myFlags.individualFlags.in4;
        }else{
            if ((miTimer.read_ms()-hearbeatTime)<= HEARTBEATINTERVAL){
                if((miTimer.read_ms() - sequenceTime) >= tiempo){
                    if(heartBeatEvent < ((modeState*2)+sum)){
                        HEARBEAT=!HEARBEAT;
                        heartBeatEvent++;
                        sequenceTime = miTimer.read_ms();
                        if(heartBeatEvent == 2){
                            tiempo = 100;
                        }
                    }else{
                        HEARBEAT=1;
                    }
                }
            }else{
                hearbeatTime=miTimer.read_ms();
                heartBeatEvent = 0;
                if(myFlags.individualFlags.status == 1)
                    tiempo = 500;
            }
        }

        //boton cambia de modo
        if(myFlags.individualFlags.checkButtons){
            myFlags.individualFlags.checkButtons = false;
            updateMef();
        }

        //Leer distancia
        //Envia un pulso
        debounceTrigger--;
        if(!debounceTrigger){
            debounceTrigger = ULTRASONIDOINTERVAL;
            TRIGGER.write(1);
            pulsoUltrasonido.attach_us(&myTrigger,10);
        } 



        //Tareas
        if(myFlags.individualFlags.status){
            switch (modeState){
                case IDLE:
                    break;
                case GOTO_OBJECT://*******************************************************************************            MODO1
                    //Oscilar el servo
                    if((servoTimer.read_ms() - servoTime) >= 100){ 
                        servoTime = servoTimer.read_ms();
                        if(myFlags.individualFlags.servoAngle){
                            if(anguloServo >= 45){
                                anguloServo -= 10;
                                myFlags.individualFlags.servoAngle = false;
                            }else{
                                anguloServo += 10;

                            }
                        }else{
                            if(anguloServo <= (-45)){
                                anguloServo += 10;
                                myFlags.individualFlags.servoAngle = true;
                            }else{
                                anguloServo -= 10;

                            }
                        }
                        servoMove();
                        if(anguloServo >= 42){
                            contServo++;
                        }
                    }

                    //Encontrar objeto
                    if(distance_us <= 1500){
                        servoTimer.stop();
                        servoTime = servoTimer.read_ms();

                        //Acercarse o alejarse
                        if(anguloServo >= -5 && anguloServo <= 5){
                            if(distance_us > 450){ //Acercarse
                                if(!myFlags.individualFlags.launchControl){
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                }else{
                                    powerM2 = 800;
                                    powerM1 = 800;
                                }
                                //Configuarar motores
                                myFlags.individualFlags.in1 = false;
                                myFlags.individualFlags.in2 = true;
                                myFlags.individualFlags.in3 = false;
                                myFlags.individualFlags.in4 = true;
                            }

                            if(distance_us < 250){ //Alejarse
                                if(!myFlags.individualFlags.launchControl){
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                }else{
                                    powerM2 = 800;
                                    powerM1 = 800;
                                }
                                //Configuarar motores
                                myFlags.individualFlags.in1 = true;
                                myFlags.individualFlags.in2 = false;
                                myFlags.individualFlags.in3 = true;
                                myFlags.individualFlags.in4 = false;
                            }
                            
                            if(distance_us < 450 && distance_us > 250){ //Parar
                                powerM1 = 0;
                                powerM2 = 0;
                            }

                        }else{ //Posicionarse de frente al objeto

                            if(anguloServo > 0){ //Girar hacia la izquierda
                                if(contGiro >= 2){
                                    //Posicionar servo
                                    anguloServo -= 5;
                                    contGiro = 0;
                                    servoMove();
                                }
                                //Configuarar motores
                                powerM2 = 1000;
                                powerM1 = 1000;
                                myFlags.individualFlags.in1 = true;
                                myFlags.individualFlags.in2 = false;
                                myFlags.individualFlags.in3 = false;
                                myFlags.individualFlags.in4 = true;
                            
                            }

                            if(anguloServo < 0){ //Girar hacia la derecha
                                if(contGiro >= 2){
                                    //Posicionar servo
                                    anguloServo += 5;
                                    contGiro = 0;
                                    servoMove();
                                }
                                //Configuarar motores
                                powerM1 = 1000;
                                powerM2 = 1000;
                                myFlags.individualFlags.in1 = false;
                                myFlags.individualFlags.in2 = true;
                                myFlags.individualFlags.in3 = true;
                                myFlags.individualFlags.in4 = false;
                                
                            }

                            if(anguloServo == 0){ //parar
                                powerM1 = 0;
                                powerM2 = 0;
                            }
                        }
                                               
                    }else{ //Buscar objeto
                        servoTimer.start();
                        powerM1 = 0;
                        powerM2 = 0;
                        //Girar sobre su eje cada dos ciclos si no encontro objetos
                        if(contServo >= 2){
                            powerM1 = 1000;
                            powerM2 = 1000;
                            myFlags.individualFlags.in1 = false;
                            myFlags.individualFlags.in2 = true;
                            myFlags.individualFlags.in3 = true;
                            myFlags.individualFlags.in4 = false;
                            if(contGiro > 10){
                                powerM1 = 0;
                                powerM2 = 0;
                                contGiro = 0;
                                contServo = 0;
                            }
                        }
                    }
                    
                    //Setear motores
                    MOTOR1.pulsewidth_us(powerM1);
                    IN1 = myFlags.individualFlags.in1;
                    IN2 = myFlags.individualFlags.in2;
                    MOTOR2.pulsewidth_us(powerM2);
                    IN3 = myFlags.individualFlags.in3;
                    IN4 = myFlags.individualFlags.in4;
                    
                    break;
                case FOLLOW_LINE://*******************************************************************************            MODO2

                    switch (direction){
                        case AHEAD: //Seguir la linea
                            if(distance_us > 700){
                                if(!myFlags.individualFlags.launchControl){
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    myFlags.individualFlags.rotateFlag = !myFlags.individualFlags.rotateFlag;
                                    //Configurar motores
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;
                                    anguloServo = 0;
                                    servoMove();
                                }else{
                                    powerM2 = 800;
                                    powerM1 = 800;
                                }

                                //Acomodarse hacia la derecha
                                if(valueIRR < VALORLINEA){ 
                                    powerM2 = 1000;
                                    powerM1 = 0;

                                }

                                //Acomodarse hacia la izquierda
                                if(valueIRL < VALORLINEA){
                                    powerM2 = 0;
                                    powerM1 = 1000;
                                    
                                }
                                
                            }else{ //Si encuentra un obstaculo gira hacia la derecha
                                contGiro = 0;    
                                powerM2 = 0;
                                powerM1 = 0;                                          
                                direction = TURN_RIGHT;
                            }
                            break;
                        case TURN_RIGHT: //Girar hacia la derecha
                            //Setear motores y servo 
                            powerM2 = 1000;
                            powerM1 = 1000;
                            if(myFlags.individualFlags.rotateFlag){
                                    myFlags.individualFlags.in3 = true;
                                    myFlags.individualFlags.in4 = false;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true; 
                                    if(contGiro > 20){
                                        direction = AHEAD;
                                        contGiro = 0;
                                        myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    }                       
                            }else{
                                if(contGiro < 20){
                                    myFlags.individualFlags.in3 = true;
                                    myFlags.individualFlags.in4 = false;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                }else{                           
                                    anguloServo = 90;
                                    servoMove();
                                    direction = FOLLOW_OBS;
                                }
                            }

                            break;
                        case FOLLOW_OBS: //Rodear el obstaculo
                            
                            if(distance_us > 1200){ //Si deja de detectar el obstaculo girar hacia la izquierda
                                if(contGiro < 20){
                                    //Setear motores
                                    powerM2 = 800;
                                    powerM1 = 800;       
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;                                
                                }else{
                                    pulse(15);
                                }
                                

                            }else{
                                //Mantener el obstaculo a 20cm aprox
                                if((distance_us < 900) && (distance_us > 800)){
                                    //Configurar motores
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                }
                                if(distance_us > 900){
                                    //Configurar motores
                                    powerM2 = 1000;
                                    powerM1 = 500;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                }
                                if(distance_us < 800){
                                    //Configurar motores
                                    powerM2 = 500;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                }

                                //Si vuelve a detectar una linea girar a la derecha para volver al camino
                                if(valueIRR > VALORLINEA){
                                    contGiro = 0;
                                    myFlags.individualFlags.rotateFlag = true;
                                    direction = TURN_RIGHT;
                                }
                            }
                            break;
                        default:
                            break;
                    }

                    //Setear motores
                    MOTOR1.pulsewidth_us(powerM1);
                    IN1 = myFlags.individualFlags.in1;
                    IN2 = myFlags.individualFlags.in2;
                    MOTOR2.pulsewidth_us(powerM2);
                    IN3 = myFlags.individualFlags.in3;
                    IN4 = myFlags.individualFlags.in4;
                    break;

                case MAZE://*******************************************************************************            MODO3

                    switch(mazeState){
                        case AHEAD: //Hacia adelante
                            if(myFlags.individualFlags.moved){ //Si no esta en una bifurcacion
                                if(distance_us < 1800 || (valueIRL > VALORLINEA && valueIRR > VALORLINEA)){ //Si detecta un objeto o se termina la linea
                                    //Ir al modo de decicion
                                    mazeState = ANALIZE;
                                    powerM1 = 0;
                                    powerM2 = 0;
                                    comp = miTimer.read_ms();
                                    contGiro = 0;
                                    myFlags.individualFlags.rotateFlag = false;
                                }else{
                                    //Correccion de la rueda derecha
                                    if(valueIRR < VALORLINEA){
                                        powerM2 = 1000;
                                        powerM1 = 0;
                                    }
                                    //COrreccion de la rueda izquierda
                                    if(valueIRL < VALORLINEA){
                                        powerM2 = 0;
                                        powerM1 = 1000;
                                    }  
                                    //Seguir adelante
                                    if(valueIRR < VALORLINEA && valueIRL < VALORLINEA){
                                        if(!myFlags.individualFlags.launchControl){
                                            powerM2 = 1000;
                                            powerM1 = 1000;
                                            myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                            anguloServo = 0;
                                            servoMove();
                                        }else{
                                            powerM2 = 800;
                                            powerM1 = 800;
                                        }                         
                                    }
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;  
                                }
                            }else{
                                // //Si vuelve a detectar una linea girar a la derecha para volver al camino
                                if(contGiro < 80){
                                    if(valueIRR < VALORLINEA){
                                        powerM2 = 1000;
                                        powerM1 = 0;
                                    }
                                    //COrreccion de la rueda izquierda
                                    if(valueIRL < VALORLINEA){
                                        powerM2 = 0;
                                        powerM1 = 1000;
                                    }  
                                    //Seguir adelante
                                    if(valueIRR < VALORLINEA && valueIRL < VALORLINEA){
                                        if(!myFlags.individualFlags.launchControl){
                                            powerM2 = 1000;
                                            powerM1 = 1000;
                                            myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                            anguloServo = 0;
                                            servoMove();
                                        }else{
                                            powerM2 = 800;
                                            powerM1 = 800;
                                        }                         
                                    }
                                    myFlags.individualFlags.in1 = false;
                                    myFlags.individualFlags.in2 = true;
                                    myFlags.individualFlags.in3 = false;
                                    myFlags.individualFlags.in4 = true;
                                }else{
                                   myFlags.individualFlags.moved = true;
                                   contGiro = 0; 
                                }
                            }

                        break;
                        case TURN_RIGHT: //Girar a la derecha y parar cuando se paso la linea
                            powerM2 = 1000;
                            powerM1 = 1000;
                            myFlags.individualFlags.in3 = true;
                            myFlags.individualFlags.in4 = false;
                            myFlags.individualFlags.in1 = false;
                            myFlags.individualFlags.in2 = true; 
                            if(contGiro > 15){
                                if(!myFlags.individualFlags.launchControl){
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    
                                }else{
                                    powerM2 = 700;
                                    powerM1 = 700;
                                }
                                myFlags.individualFlags.in3 = false;
                                myFlags.individualFlags.in4 = true;
                                myFlags.individualFlags.in1 = false;
                                myFlags.individualFlags.in2 = true;   
                                if(valueIRL < VALORLINEA && valueIRR > VALORLINEA){
                                    powerM1 = 0;
                                    powerM2 = 0;
                                    mazeState = 10;
                                    comp = miTimer.read_ms();
                                    contGiro = 0;
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    myFlags.individualFlags.turn = true;
                                }
                            }
                        break;
                        case TURN_LEFT: //Girar a la izquierda y parar cuando se paso la linea
                            powerM2 = 1000;
                            powerM1 = 1000;
                            myFlags.individualFlags.in3 = false;
                            myFlags.individualFlags.in4 = true;
                            myFlags.individualFlags.in1 = true;
                            myFlags.individualFlags.in2 = false; 
                            if(contGiro > 15){
                                if(!myFlags.individualFlags.launchControl){
                                    powerM2 = 1000;
                                    powerM1 = 1000;
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    
                                }else{
                                    powerM2 = 700;
                                    powerM1 = 700;
                                }
                                myFlags.individualFlags.in3 = false;
                                myFlags.individualFlags.in4 = true;
                                myFlags.individualFlags.in1 = false;
                                myFlags.individualFlags.in2 = true;   
                                if(valueIRR < VALORLINEA && valueIRL > VALORLINEA){
                                    powerM1 = 0;
                                    powerM2 = 0;
                                    mazeState = 10;
                                    comp = miTimer.read_ms();
                                    contGiro = 0;
                                    myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                    myFlags.individualFlags.turn = false;
                                }
                            }                       
                        break;
                        case TURN_BACK: //Girar 180° y volver
                            powerM2 = 1000;
                            powerM1 = 1000;
                            myFlags.individualFlags.in3 = false;
                            myFlags.individualFlags.in4 = true;
                            myFlags.individualFlags.in1 = true;
                            myFlags.individualFlags.in2 = false; 
                            if(contGiro > 55){
                                mazeState = AHEAD;
                                contGiro = 0;
                                myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                                powerM1 = 0;
                                powerM2 = 0;
                            }  
                        break;
                        case ANALIZE: //Tomar distancias y decidir si girar o seguir
                            powerM1 = 0;
                            powerM2 = 0;
                            //Tomar distancias adelante y en los laterales
                            if((miTimer.read_ms() - comp) >= 500 && myFlags.individualFlags.rotateFlag == false){
                                switch (contServo2)
                                {
                                case 0:
                                    distanciaAde = distance_us;
                                    anguloServo = 90;
                                    servoMove();
                                    contServo2++;                                   
                                    break;
                                case 1:
                                    distanciaIzq = distance_us;
                                    anguloServo = -90;
                                    servoMove();
                                    contServo2++;
                                    break;
                                case 2:
                                    distanciaDer = distance_us;
                                    anguloServo = 0;
                                    servoMove();
                                    contServo2++;
                                    myFlags.individualFlags.rotateFlag = true;
                                    break;
                                default:
                                    break;
                                }
                                comp = miTimer.read_ms();
                            }

                            //Encontro la salida
                            if(myFlags.individualFlags.rotateFlag && (distanciaAde >= 2700 && distanciaAde <= 3100) && (distanciaDer > 1800 && distanciaIzq > 1800)){
                                mazeState = WIN;
                                contGiro = 0;
                                comp = miTimer.read_ms();
                                myFlags.individualFlags.rotateFlag = false;
                            }

                            //Si no puede ir hacia adelante girar por el camino posible
                            if(myFlags.individualFlags.rotateFlag && distanciaAde < 1800){
                                if(distanciaDer >= 1800 && distanciaIzq <= 1800)
                                    mazeState = TURN_RIGHT;
                                if(distanciaDer <= 1800 && distanciaIzq >= 1800)
                                    mazeState = TURN_LEFT;
                                if(distanciaDer <= 1800 && distanciaIzq <= 1800)
                                    mazeState = TURN_BACK;
                                if(distanciaDer >= 1800 && distanciaIzq >= 1800){
                                    if(distanciaIzq > distanciaDer)
                                        mazeState = TURN_LEFT;
                                    else
                                        mazeState = TURN_RIGHT;
                                }
                                myFlags.individualFlags.rotateFlag = false;
                                contGiro = 0;
                                contServo2 = 0;
                                myFlags.individualFlags.launchControl = !myFlags.individualFlags.launchControl;
                            }


                            //Si se termino la linea pero puede seguir para adelante decide si doblar o seguir
                            if(myFlags.individualFlags.rotateFlag && distanciaAde > 1800){
                                randomDirec = (rand() % 2) + 1;
                                if(distanciaDer > distanciaIzq){//derecha
                                    if(randomDirec == 1){
                                        mazeState = 10;
                                        myFlags.individualFlags.turn = true;
                                    }else{
                                        mazeState = AHEAD;
                                        myFlags.individualFlags.moved = false; 
                                        distanciaPared = distanciaIzq;
                                        pulse(10);
                                    }                                    
                                    
                                }else{//izquierda
                                    if(randomDirec == 1){
                                        mazeState = 10;
                                        myFlags.individualFlags.turn = false;
                                        
                                    }else{
                                        mazeState = AHEAD; 
                                        myFlags.individualFlags.moved = false; 
                                        distanciaPared = distanciaDer;
                                        pulse(10);
                                    } 
                                }

                                myFlags.individualFlags.rotateFlag = false;
                                contGiro = 0;
                                contServo2 = 0;
                            } 


                        break;

                        case WIN:
                            if((miTimer.read_ms() - comp) <= 3000){
                                powerM1 = 1000;
                                powerM2 = 1000;
                                myFlags.individualFlags.in3 = false;
                                myFlags.individualFlags.in4 = true;
                                myFlags.individualFlags.in1 = true;
                                myFlags.individualFlags.in2 = false; 
                            }else{
                                powerM1 = 0;
                                powerM2 = 0;
                                modeState = IDLE;
                                contGiro = 0;
                                sum = 0;
                                myFlags.individualFlags.status = !myFlags.individualFlags.status;
                            } 

                        break;
                        default:
                                if(myFlags.individualFlags.turn){
                                    powerM1 = 1000;
                                    powerM2 = 0;
                                    if(valueIRL < VALORLINEA && valueIRR < VALORLINEA){
                                        mazeState = AHEAD;
                                        myFlags.individualFlags.moved = false;
                                        contGiro = 0;
                                    }
                                }else{
                                    powerM1 = 0;
                                    powerM2 = 1000;
                                    if(valueIRL < VALORLINEA && valueIRR < VALORLINEA){
                                        mazeState = AHEAD;
                                        myFlags.individualFlags.moved = false;
                                        contGiro = 0;
                                    }
                                }
                                
                        break;
                    }

                    //Setear motores
                    MOTOR1.pulsewidth_us(powerM1);
                    IN1 = myFlags.individualFlags.in1;
                    IN2 = myFlags.individualFlags.in2;
                    MOTOR2.pulsewidth_us(powerM2);
                    IN3 = myFlags.individualFlags.in3;
                    IN4 = myFlags.individualFlags.in4;


                    break;
                default:
                    break;
            }
        }
        //Chequeo de buffer de entrada
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        //Chequeo buffer de salida
        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();
    }
    return 0;
}

void myTrigger(void){ //Cortar el pulso
    TRIGGER.write(0);
}

void startEcho(void){ //Enviar onda
    echoTimer.reset();
}

void endEcho(void){ //Guardar distancia
    distance_us = echoTimer.read_us();
}

void getSpeed(void){ //Actualizar velocidad
    speedM1 = contSpeed1;
    speedM2 = contSpeed2;
    contSpeed2 = 0;
    contSpeed1 = 0;
}

void getIR(void){ //Almacenar mediciones de los sensores de linea
    valueIRL = IRL.read_u16();
    valueIRR = IRR.read_u16();
}

void speedMoment1(void){ //Sensor de horquilla izquierda
    contSpeed1++;
    contGiro++;
}

void speedMoment2(void){ //Sensor de horquilla derecha
    contSpeed2++;
}
/*****************************************************************************************************/
/************  MEF para decodificar el protocolo serie ***********************/
void decodeProtocol(void)
{
    static int8_t nBytes=0, indice=0;
    while (datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosComProtocol.cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosComProtocol.indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosComProtocol.indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                nBytes=datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosComProtocol.cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                    datosComProtocol.payload[0]=nBytes;
                    indice=1;
                }
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosComProtocol.payload[indice++]=datosComProtocol.bufferRx[datosComProtocol.indexReadRx];
                    datosComProtocol.cheksumRx ^= datosComProtocol.bufferRx[datosComProtocol.indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosComProtocol.cheksumRx == datosComProtocol.bufferRx[datosComProtocol.indexReadRx]){
                        decodeData();
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }

}


/*****************************************************************************************************/
/************  Función para procesar el comando recibido ***********************/
void decodeData(void)
{
    uint8_t auxBuffTx[50], indiceAux=0, cheksum;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosComProtocol.payload[1]) {
        case ALIVE: //Confirma Conexion
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
        case SET_POWER: //Setea potecia de motores
            if(modeState == IDLE){
                myWord.i8[0] = datosComProtocol.payload[2];
                myWord.i8[1] = datosComProtocol.payload[3];
                myWord.i8[2] = datosComProtocol.payload[4];
                myWord.i8[3] = datosComProtocol.payload[5];
                powerM1 = myWord.i32;
                myWord.i8[0] = datosComProtocol.payload[6];
                myWord.i8[1] = datosComProtocol.payload[7];
                myWord.i8[2] = datosComProtocol.payload[8];
                myWord.i8[3] = datosComProtocol.payload[9];
                powerM2 = myWord.i32;
                //Motor izquierdo           
                if(powerM1 <= 0){
                    //Reversa
                    powerM1 *= -1;
                    myFlags.individualFlags.in1 = true;
                    myFlags.individualFlags.in2 = false;
                }else{
                    //Drive
                    myFlags.individualFlags.in1 = false;
                    myFlags.individualFlags.in2 = true; 
                }    
                //Motor derecho
                if(powerM2 <= 0){
                    //Reversa
                    powerM2 *= -1;
                    myFlags.individualFlags.in3 = true;
                    myFlags.individualFlags.in4 = false;
                }else{
                    //Drive
                    myFlags.individualFlags.in3 = false;
                    myFlags.individualFlags.in4 = true;
                }
            }
            auxBuffTx[indiceAux++] = SET_POWER;
            auxBuffTx[indiceAux++] = ACK;
            auxBuffTx[NBYTES] = 0x03;
            break;
        case SET_SERVO: //Mueve el servo
            auxBuffTx[indiceAux++] = SET_SERVO;
            if(modeState == IDLE){
                anguloServo = datosComProtocol.payload[2];
                if(myFlags.individualFlags.servoMove){
                    auxBuffTx[indiceAux++] = ACK;
                    myFlags.individualFlags.servoMove = false;
                    servoMove();
                }else{
                    auxBuffTx[indiceAux++] = 0x0A;
                    myFlags.individualFlags.servoMove = true;
                }
            }else{
                auxBuffTx[indiceAux++] = ACK;   
            }
            auxBuffTx[NBYTES] = 0x03;
            break;
        case GET_IR: //Envia los datos de los sensores de linea
            auxBuffTx[indiceAux++] = GET_IR;
            myWord.ui16[0] = valueIRR;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            myWord.ui16[0] = valueIRL;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[NBYTES] = 0x06;
            break;
        case GET_DISTANCE: //Envia los datos obtenidos por el infrarrojo
            auxBuffTx[indiceAux++] = GET_DISTANCE;
            myWord.ui32 = distance_us;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];  
            auxBuffTx[NBYTES] =0x06;          
            break;
        case GET_SPEED: //Envia la velocidad de las ruedas
            //Leer velocidad
            auxBuffTx[indiceAux++] = GET_SPEED;
            myWord.ui32 = speedM1;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];            
            myWord.ui32 = speedM2;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];            
            auxBuffTx[NBYTES] = 0x0A;
            break;
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++){
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}


/*****************************************************************************************************/
/************  Función para enviar los bytes hacia la pc ***********************/
void sendData(void)
{
    if(pcCom.writable())
        pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);

}

/**********************************************************************************/
/* Servicio de Interrupciones*/

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}

void OnTimeOut(void) //Checkear botones
{
    if(!myFlags.individualFlags.checkButtons)
        myFlags.individualFlags.checkButtons=true;
}

void servoMove(void){ //Mover servo
    //Calcular el pulso segun el angulo
    if(anguloServo > 0)
        auxAngulo = (anguloServo*11) + 1500;
    else
        auxAngulo = 1500 - (anguloServo*(-11));

    SERVO.pulsewidth_us(auxAngulo);

    if(modeState == IDLE){
        datosComProtocol.payload[1] = SET_SERVO;
        decodeData();
    }
}

void updateMef(void){

    switch(boton.state){
        case UP:            
            if(!(BUTTON.read() & 1)){
                boton.state = FALLING;
            }
            break;
            
        case DOWN:
        
            if(BUTTON.read() & 1){
                boton.state = RISING;   
            }
            break;
        
        case RISING:
            if(BUTTON.read() & 1){
                boton.state = UP;
                boton.timeDif = miTimer.read_ms() - boton.timeDown;

                //Cambio de modo 
                if((boton.timeDif >= 1000) && (boton.timeDif <= 2000) && (modeState != IDLE)){

                    myFlags.individualFlags.status = !myFlags.individualFlags.status;
                    tiempo = 500;

                    if(myFlags.individualFlags.status == true){
                        sum=2;
                    }else
                        sum=0;
                    powerM1 = 0;
                    powerM2 = 0;
                    MOTOR1.pulsewidth_us(powerM1);
                    MOTOR2.pulsewidth_us(powerM2);
                    anguloServo = 0;
                    servoMove();

                }else{
                    if(!myFlags.individualFlags.status){
                        switch (modeState){
                            case IDLE:
                                modeState = GOTO_OBJECT;
                                break;
                            case GOTO_OBJECT:
                                modeState = FOLLOW_LINE;
                                break;
                            case FOLLOW_LINE:
                                modeState = MAZE;
                                mazeState = AHEAD;
                                break;
                            case MAZE:
                                modeState = IDLE;
                                powerM2 = 0;
                                powerM1 = 0;
                                anguloServo = 0;
                                comp = 0;
                                servoMove();
                            default:
                                break;
                        }
                        contGiro = 0;
                        contServo2 = 0;
                        //Inicializacion de banderas
                        myFlags.individualFlags.checkButtons = false;
                        myFlags.individualFlags.servoMove = false;
                        myFlags.individualFlags.status = false;
                        myFlags.individualFlags.servoAngle = true;
                        myFlags.individualFlags.launchControl = false;
                        myFlags.individualFlags.rotateFlag = true;
                        myFlags.individualFlags.moved = true;
                    }
                }

                if(boton.timeDif >= 3000){ //Volver al menu principal
                    modeState = IDLE;
                    powerM2 = 0;
                    powerM1 = 0;
                    anguloServo = 0;
                    servoMove();
                }
                
            }else{
                boton.state = DOWN;
 
            }
            break;
        case FALLING:
            if(!(BUTTON.read() & 1)){
                boton.state = DOWN;
                boton.timeDown = miTimer.read_ms();
    
            }else{
                boton.state = UP;
            }
            break;
        default:
            boton.state = UP;
            break;

    }

}

void presentacion(void){   
    //Probar motores hacia adelante 
    MOTOR1.pulsewidth_us(1000);
    MOTOR2.pulsewidth_us(1000);
    IN2 = true;
    IN4 = true;
    wait_ms(500);
    //Probar motores hacia atras
    MOTOR1.pulsewidth_us(1000);
    MOTOR2.pulsewidth_us(1000);
    IN1 = true;
    IN2 = false;
    IN3 = true;
    IN4 = false;
    wait_ms(500);
    //Parar los motores
    MOTOR1.pulsewidth_us(0);
    MOTOR2.pulsewidth_us(0);
    IN1 = 0;
    IN2 = 0;
    IN3 = 0;
    IN4 = 0;
    //Mover el servo hacia extremos y centro
    anguloServo = 0;
    SERVO.pulsewidth_us(2500);
    wait_ms(500);
    SERVO.pulsewidth_us(500);
    wait_ms(500);
    SERVO.pulsewidth_us(1500);
}

void pulse(uint8_t pulsos){
    if(modeState == MAZE){
        if(contGiro < pulsos){
            powerM2 = 1000;
            powerM1 = 1000;
            myFlags.individualFlags.in3 = false;
            myFlags.individualFlags.in4 = true;
            myFlags.individualFlags.in1 = false;
            myFlags.individualFlags.in2 = true;
        }
    }else{
        if(contGiro < pulsos){
        powerM2 = 1000;
        powerM1 = 200;
        myFlags.individualFlags.in3 = false;
        myFlags.individualFlags.in4 = true;
        myFlags.individualFlags.in1 = true;
        myFlags.individualFlags.in2 = false;
    }
    }
    
}