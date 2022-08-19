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
    servoTimer = new QTimer(this);
    mySerial = new QSerialPort(this);
    mySettings = new SettingsDialog();
    estado = new QLabel;
    marca = new QLabel;
    myPaintBox = new QPaintBox(0,0,ui->Radarwidget);
    speedPaintBox = new QPaintBox(0,0,ui->Speedwidget);
    path = new QPainterPath();
    path2 = new QPainterPath();
    path3 = new QPainterPath();
    estado->setText("Desconectado...");
    marca->setText("Re & Rllepca ©\t");
    ui->statusbar->addWidget(marca);
    ui->statusbar->addWidget(estado);
    ui->actionDesconectar->setEnabled(false);
    myFlags.individualFlags.servoAngle = true;

    estadoProtocolo=START; //Recibe

    estadoComandos=ALIVE; //Envia

    ///Conexión de eventos
    connect(mySerial,&QSerialPort::readyRead,this, &MainWindow::dataRecived ); //Si llega recibir
    connect(myTimer, &QTimer::timeout,this, &MainWindow::myTimerOnTime); //intervalo de tiempo
    connect(ui->actionEscaneo_de_Puertos, &QAction::triggered, mySettings, &SettingsDialog::show); //Esaneo de puerto
    connect(ui->actionConectar,&QAction::triggered,this, &MainWindow::openSerialPort); //Abrir puerto
    connect(ui->actionDesconectar, &QAction::triggered, this, &MainWindow::closeSerialPort); //Cerrar puerto
    connect(ui->actionSalir,&QAction::triggered,this,&MainWindow::close ); //Cerrar programa
    connect(servoTimer, &QTimer::timeout,this, &MainWindow::getAll); //intervalo de tiempo

    myTimer->start(10);
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
        drawBackground();
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
        case ALIVE:
            ui->aliveEdit->setAlignment(Qt::AlignCenter);
            ui->aliveEdit->setText("I´M ALIVE");

            break;
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
            paso = 1;
            drawDash();
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

//Recibir todos los datos en simultaneo
void MainWindow::getAll(){
    if(myFlags.individualFlags.servoAngle){
        if(auxAngulo >= 45){
            auxAngulo -= 10;
            myFlags.individualFlags.servoAngle = false;
        }else{
            auxAngulo += 10;

        }
    }else{
        if(auxAngulo <= (-45)){
            auxAngulo += 10;
            myFlags.individualFlags.servoAngle = true;
        }else{
            auxAngulo -= 10;

        }


    }

    switch(paso){
        case 1:
            estadoComandos = GET_DISTANCE;
            sendData();
            paso = 0;
            break;
        case 2:
            estadoComandos = GET_IR;
            sendData();
            paso = 0;
            break;
        case 3:
            estadoComandos = GET_SPEED;
            sendData();
            paso = 0;
            break;
        default:
            break;
    }

}

//Probar motores
void MainWindow::on_powerButton_clicked()
{
    estadoComandos = SET_POWER;
    sendData();
}

//Leer velocidad
void MainWindow::on_pushButton_clicked()
{
    estadoComandos = GET_SPEED;
    sendData();

}

//Pedir distancia
void MainWindow::on_pushButton_2_clicked()
{
    estadoComandos = GET_DISTANCE;
    sendData();
}

//Pedir valores de sensores de linea
void MainWindow::on_pushButton_3_clicked()
{
    estadoComandos = GET_IR;
    sendData();
}

//Dibujar velocimetro
void MainWindow::drawDash(){
    QPainter painter(speedPaintBox->getCanvas());
    painter.setRenderHints(QPainter::HighQualityAntialiasing);
    int posX,posY,auxSpeed;
    int lado = speedPaintBox->width();
    float angle, auxAngle;
    uint8_t num = 0;
    posX = 25 - (lado/2);
    float agujaAngle = 6.75; //Angulo de diferencia entre cada posicion
    QString str = "";
    QPen pen;
    QFont myFont;
    QPainterPath speedPath;
    speedPath.addEllipse(5,5,lado-10,lado-10);
    speedPath.closeSubpath();

    auxSpeed = (speedM1+speedM2)/2;
    if(auxSpeed > 40)
        auxSpeed = 40;

    QConicalGradient grad(150,150,-(agujaAngle*auxSpeed-45)+180);
    grad.setColorAt(1,Qt::black);
    grad.setColorAt(0,Qt::yellow);

    painter.fillPath(speedPath,grad);
    painter.drawPath(speedPath);


    angle = (33.75*3.14/180);
    auxAngle = (-4*angle);

    painter.translate(lado/2-15,lado/2+5);

    pen.setWidth(5);
    myFont.setBold(1);
    myFont.setPixelSize(25);
    painter.setFont(myFont);

    if(auxSpeed<30)
        pen.setColor(Qt::green);
    else
        pen.setColor(Qt::red);

    painter.setPen(pen);

    //Escribir velocidad
    posX = 0;
    posY = lado/4;
    str.setNum(auxSpeed);
    painter.drawText(posX,posY,str);

    pen.setWidth(3);
    pen.setColor(Qt::white);
    painter.setPen(pen);

    //Numeros de velocidad
    for(uint8_t i=0;i<=8;i++){
        posX = ((lado/2)-30)*cos(auxAngle);
        posY = (-1*((lado/2)-30)*sin(auxAngle));
        str.setNum(num);
        painter.drawText(posX,posY,str);
        auxAngle -= angle;

        num += 5;
    }


    speedPaintBox->update();

}

//Dibujar parte fija del velocimetro
void MainWindow::drawBackground(){
    QPainter speedPainter(speedPaintBox->getCanvas());
    speedPainter.setRenderHints(QPainter::HighQualityAntialiasing);
    int sposX, sposY;
    float angle, auxAngle;
    uint8_t num = 0;
    QString str = "";
    QPen pen;
    QFont myFont;
    QPainterPath speedPath;
    speedPath.clear();
    int slado = speedPaintBox->width();

    myFont.setFixedPitch(true);

    //Dibujar dash
    pen.setWidth(5);
    pen.setColor(Qt::blue);
    speedPainter.setPen(pen);
    speedPainter.drawEllipse(5,5,slado-10,slado-10);
    myFont.setBold(1);
    myFont.setPixelSize(25);
    speedPainter.setFont(myFont);
    pen.setColor(Qt::green);
    speedPainter.setPen(pen);

    speedPainter.translate(slado/2-15,slado/2+5);

    angle = (33.75*3.14/180);
    auxAngle = (-4*angle);

    pen.setColor(Qt::white);
    speedPainter.setPen(pen);

    for(uint8_t i=0;i<=8;i++){
        sposX = ((slado/2)-30)*cos(auxAngle);
        sposY = (-1*((slado/2)-30)*sin(auxAngle));
        str.setNum(num);
        speedPainter.drawText(sposX,sposY,str);
        auxAngle -= angle;

        num += 5;
    }

    //Dibujar
    speedPaintBox->update();
}

//Setear potencia de motor 1
void MainWindow::on_dialPower1_valueChanged(int value)
{
    powerM1 = value;
    ui->lcdPower1->display(QString().number(value));
}

//Setear potencia de motor 1
void MainWindow::on_dialPower2_valueChanged(int value)
{
    powerM2 = value;
    ui->lcdPower2->display(QString().number(value));
}

//Pedir alive
void MainWindow::on_botonAlive_clicked()
{
    estadoComandos = ALIVE;
    sendData();

}

//Pedir todos los datos en simultaneo
void MainWindow::on_botonAll_clicked()
{
    paso = 1;
    auxAngulo = 0;
    servoTimer->start(100);

}

