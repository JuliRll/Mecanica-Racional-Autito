#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include <QPainterPath>
#include <math.h>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    myTimer = new QTimer(this);
    timer2 = new QTimer(this);
    mySerial = new QSerialPort(this);
    mySettings = new SettingsDialog();
    estado = new QLabel;
    marca = new QLabel;
    myPaintBox = new QPaintBox(0,0,ui->Grafica);
    path = new QPainterPath();
    path2 = new QPainterPath();
    path3 = new QPainterPath();
    estado->setText("Desconectado...");
    ui->statusbar->addWidget(marca);
    ui->statusbar->addWidget(estado);
    ui->actionDesconectar->setEnabled(false);
    myFlags.individualFlags.servoAngle = true;

    estadoProtocolo=START; //Recibe

    estadoComandos=ALIVE; //Envia

    ///Conexión de eventos
    connect(mySerial,&QSerialPort::readyRead,this, &MainWindow::dataRecived ); //Si llega recibir
    connect(myTimer, &QTimer::timeout,this, &MainWindow::myTimerOnTime); //intervalo de tiempo
    connect(timer2, &QTimer::timeout,this, &MainWindow::PaintPulsos); //intervalo de tiempo
    connect(ui->actionEscaneo_de_Puertos, &QAction::triggered, mySettings, &SettingsDialog::show); //Esaneo de puerto
    connect(ui->actionConectar,&QAction::triggered,this, &MainWindow::openSerialPort); //Abrir puerto
    connect(ui->actionDesconectar, &QAction::triggered, this, &MainWindow::closeSerialPort); //Cerrar puerto
    connect(ui->actionSalir,&QAction::triggered,this,&MainWindow::close ); //Cerrar programa

    myTimer->start(10);
    timer2->start(1000);

}

MainWindow::~MainWindow()
{
    delete ui;
}

//Tareas a realizar cuando se establece conexion
void MainWindow::openSerialPort()
{
    SettingsDialog::Settings p = mySettings->settings();
    //Configuracion de comunicacion
    mySerial->setPortName(p.name);
    mySerial->setBaudRate(p.baudRate);
    mySerial->setDataBits(p.dataBits);
    mySerial->setParity(p.parity);
    mySerial->setStopBits(p.stopBits);
    mySerial->setFlowControl(p.flowControl);
    mySerial->open(QSerialPort::ReadWrite);
    if(mySerial->isOpen()){
        ui->actionConectar->setEnabled(false);
        ui->actionDesconectar->setEnabled(true);
        estado->setText(tr("Conectado a  %1 : %2, %3, %4, %5, %6  %7")
                                         .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                                         .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl).arg(p.fabricante));
    }
    else{
        QMessageBox::warning(this,"Menu Conectar","No se pudo abrir el puerto Serie!!!!");
    }
}

//Tareas a realizar cuando se desconecta
void MainWindow::closeSerialPort()
{
    if(mySerial->isOpen()){
        mySerial->close();
        ui->actionDesconectar->setEnabled(false);
        ui->actionConectar->setEnabled(true);
        estado->setText("Desconectado................");
    }
    else{
         estado->setText("Desconectado................");
    }

}

void MainWindow::myTimerOnTime()
{
    //Si timeout verificar si hay datos para recibir
    if(rxData.timeOut!=0){
        rxData.timeOut--;
    }else{
        estadoProtocolo=START;
    }
}

//Verificar protocolo
void MainWindow::dataRecived()
{
    unsigned char *incomingBuffer;
    int count;
    //numero de bytes
    count = mySerial->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    mySerial->read((char *)incomingBuffer,count);

    rxData.timeOut=5;
    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
            case START:
                if (incomingBuffer[i]=='U'){
                    estadoProtocolo=HEADER_1;
                    rxData.cheksum=0;
                }
                break;
            case HEADER_1:
                if (incomingBuffer[i]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    i--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (incomingBuffer[i]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    i--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                rxData.nBytes=incomingBuffer[i];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (incomingBuffer[i]==':'){
                   estadoProtocolo=PAYLOAD;
                    rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                    rxData.payLoad[0]=rxData.nBytes;
                    rxData.index=1;
                }
                else{
                    i--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (rxData.nBytes>1){
                    rxData.payLoad[rxData.index++]=incomingBuffer[i];
                    rxData.cheksum^=incomingBuffer[i];
                }
                rxData.nBytes--;
                if(rxData.nBytes==0){
                    estadoProtocolo=START;
                    if(rxData.cheksum==incomingBuffer[i]){
                        decodeData();
                    }
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    delete [] incomingBuffer;
}

//Decodificar datos
void MainWindow::decodeData()
{
    switch (rxData.payLoad[1]) {
        case GET_SPEED: //Velocidades de las ruedas en pulsos por segundo
            myWord.ui8[0] = rxData.payLoad[2];
            myWord.ui8[1] = rxData.payLoad[3];
            myWord.ui8[2] = rxData.payLoad[4];
            myWord.ui8[3] = rxData.payLoad[5];
            speedM1 = myWord.ui32;

            myWord.ui8[0] = rxData.payLoad[6];
            myWord.ui8[1] = rxData.payLoad[7];
            myWord.ui8[2] = rxData.payLoad[8];
            myWord.ui8[3] = rxData.payLoad[9];
            speedM2 = myWord.ui32;

            ui->speedScreen1->display(QString().number(speedM1,10));
            ui->speedScreen2->display(QString().number(speedM2,10));

            break;
        case SET_POWER:
            break;
        default:
            break;
    }
    estadoComandos = GET_SPEED;
    sendData();
}

//Enviar datos, elaborar protocolo
void MainWindow::sendData()
{
    //carga el header y token
    txData.index=0;
    txData.payLoad[txData.index++]='U';
    txData.payLoad[txData.index++]='N';
    txData.payLoad[txData.index++]='E';
    txData.payLoad[txData.index++]='R';
    txData.payLoad[txData.index++]=0;
    txData.payLoad[txData.index++]=':';
    //carga el ID y nBytes
    switch (estadoComandos) {
    case ALIVE:
        txData.payLoad[txData.index++]=ALIVE;
        txData.payLoad[NBYTES]=0x02;
        break;
    case GET_IR:
        txData.payLoad[txData.index++]=GET_IR;
        txData.payLoad[NBYTES]=0x02;
        break;
    case GET_DISTANCE:
        txData.payLoad[txData.index++]=GET_DISTANCE;
        txData.payLoad[NBYTES]=0x02;
        break;
    case GET_SPEED:
        txData.payLoad[txData.index++]=GET_SPEED;
        txData.payLoad[NBYTES]=0x02;
        break;
    case SET_POWER: //Potencia de las ruedas
        txData.payLoad[txData.index++]=SET_POWER;
        myWord.i32 = powerM1;
        txData.payLoad[txData.index++] = myWord.i8[0];
        txData.payLoad[txData.index++] = myWord.i8[1];
        txData.payLoad[txData.index++] = myWord.i8[2];
        txData.payLoad[txData.index++] = myWord.i8[3];
        myWord.i32 = powerM2;
        txData.payLoad[txData.index++] = myWord.i8[0];
        txData.payLoad[txData.index++] = myWord.i8[1];
        txData.payLoad[txData.index++] = myWord.i8[2];
        txData.payLoad[txData.index++] = myWord.i8[3];
        txData.payLoad[NBYTES]=0x0A;
        break;
    case SET_SERVO: //Angulo del servo
        txData.payLoad[txData.index++]=SET_SERVO;
        txData.payLoad[txData.index++]=anguloServo;
        txData.payLoad[NBYTES]=0x03;
    default:
        break;
    }

   txData.cheksum=0;

   //recuenta los bytes y carga el checksum
   for(int a=0 ;a<txData.index;a++)
       txData.cheksum^=txData.payLoad[a];
    txData.payLoad[txData.index]=txData.cheksum;
    if(mySerial->isWritable()){
        mySerial->write((char *)txData.payLoad,txData.payLoad[NBYTES]+6);
    }
}

//Probar motores
void MainWindow::on_powerButton_clicked()
{
    estadoComandos = SET_POWER;
    sendData();
}

//Setear potencia de motor 1
void MainWindow::on_dialPower1_valueChanged(int value)
{
    powerM1 = value;
    powerM2 = value;
    ui->lcdPower1->display(QString().number(value));
}

//Setear potencia de motor 1
void MainWindow::on_dialPower2_valueChanged(int value)
{
    powerM2 = value;
    ui->lcdPower2->display(QString().number(value));
}

//Pedir todos los datos en simultaneo
void MainWindow::on_botonAll_clicked()
{
    PaintGrafica();
}

void MainWindow::PaintGrafica(){
    QPen pen;
    int16_t posx = 0,x ,posy = 0, width, height;
    QPainter paint(myPaintBox->getCanvas());
    bool k=true;
    if(k==true){
        k = false;
        width = myPaintBox->width();
        height = myPaintBox->height();
        pen.setColor(Qt::white);
        paint.setPen(pen);
        posx = 0;
        posy = height-height;
        //paint.setRenderHint(QPainter::Antialiasing);
        paint.drawLine(0,height,posx,0);
        paint.drawLine(0,height,width,height-1);
    }
    myPaintBox->update();
}

void MainWindow::PaintPulsos(){
    QPen pen1,pen2;
    int16_t posx = 0,x ,posy = 0, width, height;
    QPainter paint(myPaintBox->getCanvas());
    width = myPaintBox->width();
    height = myPaintBox->height();
    pen1.setColor(Qt::green);
    pen2.setColor(Qt::red);
    paint.setPen(pen1);
    gradX = gradX + 10;
    paint.drawLine(gradX-10,height-M1Anterior,gradX,height-speedM1);
    M1Anterior = speedM1;
    paint.setPen(pen2);
    paint.drawLine(gradX-10,height-M2Anterior,gradX,height-speedM2);
    M2Anterior = speedM2;
    if(gradX >= width){
        paint.drawRect(0,0,width,height);
        gradX=0;
        width = myPaintBox->width();
        height = myPaintBox->height();
        pen1.setColor(Qt::white);
        paint.setPen(pen1);
        posx = 0;
        posy = height-height;
        paint.drawLine(0,height,posx,0);
        paint.drawLine(0,height,width,height-1);
    }
    myPaintBox->update();
}
