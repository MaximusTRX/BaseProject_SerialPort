#include "mbed.h"
#include <stdbool.h>

#define NUMBUTT             4

#define MAXLED              4

#define NUMBEAT             4

#define HEARBEATINTERVAL    100

#define HIGH                1
#define LOW                 0

/**
 * @brief Enumeración que contiene los estados de la máquina de estados(MEF) que se implementa para 
 * el "debounce" 
 * El botón está configurado como PullUp, establece un valor lógico 1 cuando no se presiona
 * y cambia a valor lógico 0 cuando es presionado.
 */
typedef enum{
    BUTTON_DOWN,    //0
    BUTTON_UP,      //1
    BUTTON_FALLING, //2
    BUTTON_RISING   //3
}_eButtonState;

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
 * @brief Esturctura de Teclas
 * 
 */
typedef struct{
    _eButtonState estado;
    _eButtonEvent event;
    int32_t timeDown;
    int32_t timeDiff;
    uint32_t lastTime;
    uint8_t inerval;
}_sTeclas;

// _sTeclas ourButton[NUMBUTT];
_sTeclas ourButton;
//                0001 ,  0010,  0100,  1000
// uint16_t mask[]={0x0001,0x0002,0x0004,0x0008};

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
        FIRMWARE=0xF1,
        SET_LEDS=0xF2,
        IR_SENSOR=0xA0,
        MOTOR_ACTION=0xA1,
        SERVO_ACTION=0xA2,
        ULTRA_SONIC=0xA3,
        HORQUILLA=0xA4,
        OTHERS
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
 * @brief VOLATILE 
 * 
 * La palabra clave volatile se utiliza para evitar que el compilador aplique optimizaciones en variables
 * que pueden cambiar de formas en que el compilador no puede anticipar.
 * Los objetos declarados como volátiles se omiten de la optimización porque sus valores pueden ser cambiados por código 
 * fuera del alcance del código actual en cualquier momento. El sistema siempre lee el valor actual de un objeto volátil 
 * desde la ubicación de la memoria en lugar de mantener su valor en un registro temporal en el punto en el que se solicita, 
 * incluso si una instrucción previa solicitó un valor del mismo objeto. 
 * Se utilizan en variables globales modificadas por una rutina de servicio de interrupción(ISR). 
 * Si no declara la variable como volátil, el compilador optimizará el código.
 * Si no usamos un calificador volátil, pueden surgir los siguientes problemas
 *    1) Es posible que el código no funcione como se esperaba cuando la optimización está activada.
 *    2) Es posible que el código no funcione como se esperaba cuando las interrupciones están habilitadas y utilizadas.
 */
/**
 * @brief Mapa de bits para implementar banderas
 * Como uso una sola los 7 bits restantes los dejo reservados para futuras banderas
 */
typedef union{
    struct{
        uint8_t checkButtons :1;
        uint8_t Reserved :7;
    }individualFlags;
    uint8_t allFlags;
}_bGeneralFlags;

volatile _bGeneralFlags myFlags;

uint8_t hearBeatEvent;

/**
 * @brief Unión para descomponer/componer datos mayores a 1 byte
 * 
 */
typedef union {
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
}_udat;

_udat myWord;

/**
 * @brief Dato del tipo Enum para asignar los estados de la MEF
 * 
 */
_eButtonState myButton;

/*************************************************************************************************/
/* Prototipo de Funciones */

/**
 * @brief Inicializa la MEF
 * Le dá un estado inicial a myButton
 */
void startMef();

/**
 * @brief Máquina de Estados Finitos(MEF)
 * Actualiza el estado del botón cada vez que se invoca.
 * 
 * @param buttonState Este parámetro le pasa a la MEF el estado del botón.
 */
// void actuallizaMef(uint8_t indice );
void actuallizaMef(void);

/**
 * @brief Función para cambiar el estado del LED cada vez que sea llamada.
 * 
 */
// void togleLed(uint8_t indice);
/**
 * @brief Enciende o apaga los leds de la máscara
 * 
 * @param mask Se envian las posiciones de los leds que se quieren encender 
 */
//void manejadorLed(uint8_t mask);

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

void encodeData(uint8_t id);

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
 * @brief Función Hearbeat
 * Ejecuta las tareas del hearbeat 
 */
// void hearbeatTask(void);

void manejadorServo(void);

void toggleTrigger(void);//cambia el estado del trigger a LOW
void echoProcces(void);//inicia echoTimer en rise. Toma el tiempo actual en fall.

void manejadorMotor(uint16_t pulso);

void contHorquillaDer(void);
void contHorquillaIzq(void);

// void ledToggle(void);

void ledMode(void);

void setModes(void);
/*****************************************************************************************************/
/* Configuración del Microcontrolador */

// BusIn buttonArray(PB_6,PB_7,PB_8,PB_9);

// BusOut leds(PB_12,PB_13,PB_14,PB_15);

DigitalOut HEARBEAT(PC_13); //!< Defino la salida del led

Serial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

PwmOut Servo(PB_3);
PwmOut MotorDer(PA_8);
PwmOut MotorIzq(PB_4);

AnalogIn IRSensorDer(PA_1); //Sensor Infrarrojo Derecho
AnalogIn IRSensorIzq(PA_0); //Sensor Infrarrojo Izquierdo

DigitalIn ButtonMode(PA_4); //Boton para cambio de modo

DigitalOut TRIGGER(PB_13);

DigitalOut IN1(PB_5);   //Iqz
DigitalOut IN2(PB_6);   //Izq
DigitalOut IN3(PB_14);  //Der
DigitalOut IN4(PB_15);  //Der

Timer miTimer; //!< Timer general
Timer echoTimer;

Ticker timerGeneral;

Timeout Trigger;

InterruptIn uSonico(PB_12); //Interrupcion flaco Echo
/*
    InterruptIn espera el cambio de flanco del PIN especificado.
    Sin importar en que parte del codigo se encuentre la ejecucion
    al momento del cambio de estado, se atendera llamara a la "&function"
    
        InterruptIn interrupcion(PIN)
    Definicion del interrupt

        interrupcion.rise(&function)
    Al cambiar al estado de HIGH el PIN llama a la "function"

        interrupcion.fall(&function)
    Al cambiar al estado de LOW el PIN llama a la "function"

*/

InterruptIn HorquillaDer(PB_8);
InterruptIn HorquillaIzq(PB_9);
/*****************************************************************************************************/
/************  Función Principal ***********************/
typedef struct{
    bool isUsed=0;      //Habilita el llamado a la funcion manejarServo-
    int8_t angulo;      //Angulo que llega desde PC
    int time;           //Ultimo tiempo de accion
    uint8_t interval;   //Intervalo de accion
}_sServo;
_sServo servo;

typedef struct{
    volatile bool estate=0;             //Estado del echo
    volatile uint32_t echoTimeFall=0;   //Tiempo desde que se inicio el contador
    bool isUsed=0;                      //Habilita la medicion
    int timer;                          //Ultimo tiempo de accion
    int interval=100;                   //Setea el intervalo de muestreo
}_sSonico;
_sSonico ultraS;

typedef enum{
    mDer = 0x01,
    mIzq = 0x10,
    mAmbos = 0x11,
    mAdelante = 0x10,
    mAtras = 0x01,
}_eMotor;

typedef struct {
    bool isUsed=0;      //Habilita el llamado a la funcion manejorMotor
    uint8_t select;     //Motor 0=Derecha - FF=Izquierda
    uint8_t sentido;    //Direccion de giro 0=Adelante - FF=Atras
    uint16_t interval;  //Intervalo de movieminto
    int timer;          //Setea el momento de inicio el intervalo
}_sMotor;
_sMotor motor;

typedef struct{
    bool isUsed=0;
    volatile uint32_t speedM1=0;    //Motor Izquierdo
    volatile uint32_t speedM2=0;    //Motor Derecho
    uint16_t interval;              //Intervalo de movieminto
    int timer;                      //Setea el momento de inicio el intervalo
}_sHorquilla;
_sHorquilla horquilla;

typedef struct {
    bool isUsed=0;
    uint16_t valueIRIzq;
    uint16_t valueIRDer;
    int timer;
    uint16_t interval;
}_sIRSensor;
_sIRSensor sensorIR;

typedef enum{
    IDLE=0,
    MODE1,
    MODE2,
    MODE3,
}_eMode;

typedef struct{
    bool isUsed=0;              //0 no hay modo en ejecucion ---- 1 hay modo en ejecucion
    uint8_t actualMode;    //Modo seleccionado actualmente
    int lastTime;
    uint8_t ledEvent;
}_sModos;
_sModos mode;


uint8_t firmwareVer = 0x13;

int main()
{
    // int hearbeatTime=0;

    // uint32_t auxHorquillaDer=0, auxHorquillaIzq=0;
    miTimer.start();
    echoTimer.start();

    myFlags.individualFlags.checkButtons=false;
    
    pcCom.attach(&onDataRx,Serial::RxIrq);

    timerGeneral.attach_us(&OnTimeOut, 50000);

    uSonico.rise(&echoProcces);
    uSonico.fall(&echoProcces);

    HorquillaDer.rise(&contHorquillaDer);
    HorquillaIzq.rise(&contHorquillaIzq);
    horquilla.timer = miTimer.read_ms();

    MotorDer.period_us(1000);
    MotorIzq.period_us(1000);

    IRSensorDer.read_u16();
    sensorIR.interval=100;
    sensorIR.timer=miTimer.read_ms();

    startMef();
    ourButton.lastTime = miTimer.read_ms();

    // hearBeatEvent=0;
    mode.ledEvent=0;
    
    ourButton.inerval=40;
    
    mode.lastTime = miTimer.read_ms();
    mode.actualMode = IDLE;
    mode.isUsed = 0;

    while(true)
    {
        if ((miTimer.read_ms()-ourButton.lastTime)>=ourButton.inerval)
        {
            ourButton.timeDiff=0;
            ourButton.lastTime = miTimer.read_ms();
            if (ButtonMode.read()){
                ourButton.event = EV_NOT_PRESSED;
            }else{
                ourButton.event = EV_PRESSED;
            }
            actuallizaMef();
        }

        if((miTimer.read_ms()-mode.lastTime)>=100){
            mode.lastTime = miTimer.read_ms();
            ledMode();
        }

        if ((miTimer.read_ms()-sensorIR.timer)>=sensorIR.interval)
        {
            sensorIR.timer = miTimer.read_ms();
            sensorIR.valueIRIzq = 0;
            sensorIR.valueIRDer = 0;
            // if (sensorIR.isUsed){
                sensorIR.valueIRIzq = IRSensorIzq.read_u16();
                sensorIR.valueIRDer = IRSensorDer.read_u16();
                encodeData(IR_SENSOR);
            // }
        }

        if (motor.isUsed)
        {
            if ((miTimer.read_ms()-motor.timer)>=motor.interval)
            {
                // motor.timer=miTimer.read_ms();
                motor.isUsed=0;
                IN1 = LOW;
                IN2 = LOW;
                IN3 = LOW;
                IN4 = LOW;
            }
        }

        if(servo.isUsed)
        {
            if ((miTimer.read_ms()-servo.time)>=servo.interval)
            {
                manejadorServo();
            }
        }

        if ((miTimer.read_ms()-ultraS.timer)>=ultraS.interval)
        {
            ultraS.timer = miTimer.read_ms();
            TRIGGER = HIGH;
            ultraS.estate = HIGH;
            //Trigger.attach_us(&function, time) -> funcion que luego de llamarla espera "time"
            //                                      y realiza una interrupcion para llamar a "&function"
            //                                      ->"function" es una funcion cualquiera escrita por el usuario
            Trigger.attach_us(&toggleTrigger, 10);
        }

        if ((miTimer.read_ms()-horquilla.timer)>=horquilla.interval)
        {
            horquilla.timer = miTimer.read_ms();
            if (horquilla.isUsed){
                encodeData(HORQUILLA);
                horquilla.speedM1 = 0;
                horquilla.speedM2 = 0;
            }
        }        

        if (mode.isUsed)
        {
            switch (mode.actualMode)
            {
            case MODE1: //Modo de seguimiento del obstaculo
                
                if (servo.isUsed){
                    servo.isUsed=0;
                    servo.angulo = 0;
                    manejadorServo();
                }

                if (ultraS.echoTimeFall > 377)  //Si el obstaculo se aleja de 6.5
                {
                    motor.select = 0x11;
                    motor.sentido = 0x10;
                    manejadorMotor(750);
                }else if (ultraS.echoTimeFall < 319)    //Si el obstaculo se acarca a 5.5
                {
                    motor.select = 0x11;
                    motor.sentido = 0x01;
                    manejadorMotor(750);
                }else   //No hacer nada
                {
                    IN1 = LOW;
                    IN2 = LOW;
                    IN3 = LOW;
                    IN4 = LOW;
                }
                
                break;
            
            case MODE2:

                if (sensorIR.valueIRDer < 10000){
                    //Girar a la izquierda
                    motor.select = 0x10;
                    motor.sentido = 0x10;
                    manejadorMotor(500);
                }else if (sensorIR.valueIRIzq < 10000)
                {
                    //Girar a la derecha
                    motor.select = 0x01;
                    motor.sentido = 0x10;
                    manejadorMotor(500);
                }else{
                    //Mover recto
                    motor.select = 0x11;
                    motor.sentido = 0x10;
                    manejadorMotor(750);
                }
                
                break;

            case MODE3:
                
                break;
            }
        }
        
        if(datosComProtocol.indexReadRx!=datosComProtocol.indexWriteRx) 
            decodeProtocol();

        if(datosComProtocol.indexReadTx!=datosComProtocol.indexWriteTx) 
            sendData();
    }
    return 0;
}

void manejadorServo(void){
    Servo.period_ms(20);
    Servo.pulsewidth_us((1500+((servo.angulo*1000)/128)));
    // servo.isUsed = 0;
    
    // Servo.pulsewidth_us(1000);
    // wait_ms(2000);
    // Servo.pulsewidth_us(2000);
    // wait_ms(2000);
    // Servo.pulsewidth_us(1500);
}

void toggleTrigger(void){
    TRIGGER = LOW;
}

void echoProcces(void){
    if (ultraS.estate)
    {
        echoTimer.reset();
        ultraS.estate = LOW;
    }else{
        ultraS.echoTimeFall = echoTimer.read_us();
        ultraS.estate = HIGH;
        // if (ultraS.isUsed)
            encodeData(ULTRA_SONIC);
    }
}

void manejadorMotor(uint16_t pulso){
    switch (motor.select)
    {
    case mDer:
        switch (motor.sentido)
        {
        case mAtras:
            IN3 = HIGH;
            IN4 = LOW;
            MotorDer.pulsewidth_us(pulso);
            break;
        case mAdelante:
            IN3 = LOW;
            IN4 = HIGH;
            MotorDer.pulsewidth_us(pulso);
            break;
        default:
            break;
        }
        break;
    case mIzq:
        switch (motor.sentido)
        {
        case mAtras:
            IN1 = LOW;
            IN2 = HIGH;
            MotorIzq.pulsewidth_us(pulso);
            break;
        case mAdelante:
            IN1 = HIGH;
            IN2 = LOW;
            MotorIzq.pulsewidth_us(pulso);
            break;
        default:
            break;
        }
        break;
    case mAmbos:
        switch (motor.sentido)
        {
        case mAtras:
            IN1 = LOW;
            IN2 = HIGH;
            MotorIzq.pulsewidth_us(pulso);

            IN3 = HIGH;
            IN4 = LOW;
            MotorDer.pulsewidth_us(pulso);
            break;
        case mAdelante:
            IN1 = HIGH;
            IN2 = LOW;
            MotorIzq.pulsewidth_us(pulso);
            
            IN3 = LOW;
            IN4 = HIGH;
            MotorDer.pulsewidth_us(pulso);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void contHorquillaDer(void){
    horquilla.speedM2++;
}

void contHorquillaIzq(void){
    horquilla.speedM1++;
}

void ledMode(void){
    // HEARBEAT =! HEARBEAT;
    // encodeData(0x11);
    if (mode.isUsed)
    {
        switch (mode.actualMode)
        {
        case IDLE:
            HEARBEAT=!HEARBEAT;
            break;
        
        case MODE1:
            if (mode.ledEvent <= 5){
                HEARBEAT = LOW;
                mode.ledEvent++;
                if (mode.ledEvent >=5)
                    HEARBEAT = HIGH;
            }else if (mode.ledEvent < (2+5)){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=35) ? (0) : (mode.ledEvent+1);
            }
            break;
        
        case MODE2:
            if (mode.ledEvent <= 5){
                HEARBEAT = LOW;
                mode.ledEvent++;
                if (mode.ledEvent >=5)
                    HEARBEAT = HIGH;
            }else if (mode.ledEvent < (4+5)){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=35) ? (0) : (mode.ledEvent+1);
            }
            break;
        
        case MODE3:
            if (mode.ledEvent <= 5){
                HEARBEAT = LOW;
                mode.ledEvent++;
                if (mode.ledEvent >=5)
                    HEARBEAT = HIGH;
            }else if (mode.ledEvent < (6+5)){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=35) ? (0) : (mode.ledEvent+1);
            }
            break;
        }
    }else{
        switch (mode.actualMode)
        {
        case IDLE:
            HEARBEAT=!HEARBEAT;
            break;
        
        case MODE1:
            if (mode.ledEvent < 2){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=30) ? (0) : (mode.ledEvent+1);
            }
            break;
        
        case MODE2:
            if (mode.ledEvent < 4){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=30) ? (0) : (mode.ledEvent+1);
            }
            break;
        
        case MODE3:
            if (mode.ledEvent < 6){
                HEARBEAT=!HEARBEAT;
                mode.ledEvent++;
            }else{
                HEARBEAT=1;
                mode.ledEvent = (mode.ledEvent>=30) ? (0) : (mode.ledEvent+1);
            }
            break;
        }
    }
    
    
}

void setModes(void){
    if (ourButton.timeDiff >= 1000)
    {
        if (ourButton.timeDiff >= 2000)
        {
            //Salgo de la ejecucion
            mode.isUsed = 0;

            IN1 = LOW;
            IN2 = LOW;
            IN3 = LOW;
            IN4 = LOW;

            encodeData(0x011); 
        }else{
            //Entro en al ejecicion
            mode.isUsed = 1;
            servo.isUsed = 1;
            encodeData(0x011); 
        }
    }else{
        if (!(mode.isUsed))
        {
            mode.actualMode++;
            if (mode.actualMode >= 4){
                mode.actualMode = IDLE;
            }
            encodeData(0x011); 
        }
    }
}

void startMef(void){
   ourButton.estado=BUTTON_UP;
}

void actuallizaMef(void){
    switch (ourButton.estado)
    {
    case BUTTON_DOWN:
        if(ourButton.event)
           ourButton.estado=BUTTON_RISING;  
    break;
    case BUTTON_UP:
        if(!(ourButton.event))
            ourButton.estado=BUTTON_FALLING;  
    break;
    case BUTTON_FALLING:
        if(!(ourButton.event))
        {
            ourButton.timeDown=miTimer.read_ms();
            ourButton.estado=BUTTON_DOWN;
            //Flanco de bajada
        }
        else
            ourButton.estado=BUTTON_UP;    
    break;
    case BUTTON_RISING:
        if(ourButton.event){
            ourButton.estado=BUTTON_UP;
            //Flanco de Subida
            ourButton.timeDiff=miTimer.read_ms()-ourButton.timeDown;
            setModes();
        }
        else
            ourButton.estado=BUTTON_DOWN;   
    break;   
    default:
        startMef();
        break;
    }
}


// void actuallizaMef(uint8_t indice){
//     switch (ourButton[indice].estado)
//     {
//     case BUTTON_DOWN:
//         if(ourButton[indice].event)
//            ourButton[indice].estado=BUTTON_RISING;  
//     break;
//     case BUTTON_UP:
//         if(!(ourButton[indice].event))
//             ourButton[indice].estado=BUTTON_FALLING;  
//     break;
//     case BUTTON_FALLING:
//         if(!(ourButton[indice].event))
//         {
//             ourButton[indice].timeDown=miTimer.read_ms();
//             ourButton[indice].estado=BUTTON_DOWN;
//             //Flanco de bajada
//         }
//         else
//             ourButton[indice].estado=BUTTON_UP;    
//     break;
//     case BUTTON_RISING:
//         if(ourButton[indice].event){
//             ourButton[indice].estado=BUTTON_UP;
//             //Flanco de Subida
//             ourButton[indice].timeDiff=miTimer.read_ms()-ourButton[indice].timeDown;
//             togleLed(indice);
//         }
//         else
//             ourButton[indice].estado=BUTTON_DOWN;   
//     break;   
//     default:
//     startMef(indice);
//         break;
//     }
// }

/*****************************************************************************************************/
/************  Funciónes para manejar los LEDS ***********************/

// void togleLed(uint8_t indice){
//     uint16_t ledsAux=leds, auxmask=0;
//     auxmask |= 1<<indice;
//     if(auxmask & leds)
//         ledsAux &= ~(1 << (indice)) ; 
//     else
//          ledsAux |= 1 << (indice) ;
//     leds = ledsAux ;
// }

// void manejadorLed(uint8_t mask){
//     uint16_t auxled=0, setLeds=0;
//     auxled|=1<<3;
//     if(auxled & mask)
//         setLeds |= 1 <<3;
//     else
//         setLeds &= ~(1<<3);
//     auxled=0;
//     auxled|=1<<2;
//     if(auxled & mask)
//         setLeds |= 1 <<2;
//     else
//         setLeds &= ~(1<<2);
//     auxled=0;
//     auxled|=1<<1;
//     if(auxled & mask)
//         setLeds |= 1 <<1;
//     else
//         setLeds &= ~(1<<1);
//     auxled=0;
//     auxled|=1<<0;
//     if(auxled & mask)
//         setLeds |= 1 <<0;
//     else
//         setLeds &= ~(1<<0);

//     leds=setLeds;
// }
 
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
                   {
                       estadoProtocolo=HEADER_2;
                   }
                else{
                    datosComProtocol.indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='E')
                {
                    estadoProtocolo=HEADER_3;
                }
                else{
                    datosComProtocol.indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosComProtocol.bufferRx[datosComProtocol.indexReadRx++]=='R')
                {
                    estadoProtocolo=NBYTES;
                }
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
        case FIRMWARE:
            auxBuffTx[indiceAux++]=FIRMWARE;
            auxBuffTx[indiceAux++]=firmwareVer;
            auxBuffTx[NBYTES]=0x03;
            break;
        
        case ALIVE:
            auxBuffTx[indiceAux++]=ALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
        
        case IR_SENSOR:
            auxBuffTx[indiceAux++]=IR_SENSOR;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;

            // if (sensorIR.isUsed)
            //     sensorIR.isUsed = 0;
            // else
            //     sensorIR.isUsed = 1;
            
            // sensorIR.timer=miTimer.read_ms();
            break;

        case MOTOR_ACTION:
            auxBuffTx[indiceAux++]=MOTOR_ACTION;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            
            motor.isUsed = 1;
            motor.select = datosComProtocol.payload[2];
            motor.sentido = datosComProtocol.payload[3];

            myWord.ui8[0] = datosComProtocol.payload[4];
            myWord.ui8[1] = datosComProtocol.payload[5];

            motor.interval = myWord.ui32;
            //motor.interval = 2000;
            motor.timer=miTimer.read_ms();
            
            manejadorMotor(1000);
            break;

        case SERVO_ACTION:
            auxBuffTx[indiceAux++]=SERVO_ACTION;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            
            servo.angulo = datosComProtocol.payload[2];

            // servo.isUsed = 1;
            // servo.time = miTimer.read_ms();
            // servo.interval = 10;
            manejadorServo();
            break;
        
        case ULTRA_SONIC:
            auxBuffTx[indiceAux++]=ULTRA_SONIC;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;

            // ultraS.interval = 100;
            // if (ultraS.isUsed)
            //     ultraS.isUsed = 0;
            // else
            //     ultraS.isUsed = 1;
            
            break;

        case HORQUILLA:
            auxBuffTx[indiceAux++]=HORQUILLA;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;

            if (horquilla.isUsed)
                horquilla.isUsed = 0;
            else
                horquilla.isUsed = 1;
            
            horquilla.interval = 500;
            break;

        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++)
   {
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}

void encodeData(uint8_t id){
    uint8_t auxBuffTx[50], indiceAux=0, cheksum;
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (id) {
        case 0x11:
            auxBuffTx[indiceAux++]=0x11;
            auxBuffTx[NBYTES]=0x04;
            auxBuffTx[indiceAux++]=mode.actualMode;
            auxBuffTx[indiceAux++]=mode.isUsed;
            break;

        case IR_SENSOR:
            auxBuffTx[indiceAux++]=IR_SENSOR;
            auxBuffTx[NBYTES]=0x06;

            myWord.ui32 = sensorIR.valueIRIzq;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];

            myWord.ui32 = sensorIR.valueIRDer;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            break;
        
        case SERVO_ACTION:
            auxBuffTx[indiceAux++]=SERVO_ACTION;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;
            break;
        case ULTRA_SONIC:
            auxBuffTx[indiceAux++]=ULTRA_SONIC;
            auxBuffTx[NBYTES]=0x06;

            myWord.ui32 = ultraS.echoTimeFall;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];
            break;
        case HORQUILLA:
            auxBuffTx[indiceAux++]=HORQUILLA;
            auxBuffTx[NBYTES]=0x0A;
            //tengo que mandar horquila.speedM1 y M2

            myWord.ui32 = horquilla.speedM1;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];

            myWord.ui32 = horquilla.speedM2;
            auxBuffTx[indiceAux++] = myWord.ui8[0];
            auxBuffTx[indiceAux++] = myWord.ui8[1];
            auxBuffTx[indiceAux++] = myWord.ui8[2];
            auxBuffTx[indiceAux++] = myWord.ui8[3];
            break;

        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
   for(uint8_t a=0 ;a < indiceAux ;a++)
   {
       cheksum ^= auxBuffTx[a];
       datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=auxBuffTx[a];
   }
    datosComProtocol.bufferTx[datosComProtocol.indexWriteTx++]=cheksum;
}

void sendData(void)
{
    if(pcCom.writable())
        {
            pcCom.putc(datosComProtocol.bufferTx[datosComProtocol.indexReadTx++]);
        }
}

// void hearbeatTask(void)
// {
//     if(hearBeatEvent < 2){
//         HEARBEAT=!HEARBEAT;
//         hearBeatEvent++;
//     }else{
//         HEARBEAT=1;
//         hearBeatEvent = (hearBeatEvent>=30) ? (0) : (hearBeatEvent+1);    
//     }
// }

// void ledToggle(void)
// {
//     if (mode.ledEvent < 2){
//         HEARBEAT=!HEARBEAT;
//         mode.ledEvent++;
//     }else{
//         HEARBEAT=1;
//         mode.ledEvent = (mode.ledEvent>=30) ? (0) : (mode.ledEvent+1);
//     }
// }

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComProtocol.bufferRx[datosComProtocol.indexWriteRx++]=pcCom.getc();
    }
}

void OnTimeOut(void)
{
    if(!myFlags.individualFlags.checkButtons)
        myFlags.individualFlags.checkButtons=true;
}
