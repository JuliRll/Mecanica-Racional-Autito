#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QTimer>
#include <QTime>
#include <QLabel>
#include "settingsdialog.h"
#include "qpaintbox.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void openSerialPort();

    void closeSerialPort();

    void myTimerOnTime();

    void dataRecived();

    void decodeData();

    void sendData();

    void on_servoButton_clicked();

    void on_powerButton_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void getAll();

    void drawRadar();

    void drawDash();

    void drawBackground();

    void on_dialServo_valueChanged(int value);

    void on_dialPower1_valueChanged(int value);

    void on_dialPower2_valueChanged(int value);

    void on_botonAlive_clicked();

    void on_botonAll_clicked();

private:
    Ui::MainWindow *ui;
    QSerialPort *mySerial;
    QTimer *myTimer, *servoTimer, *distanceTimer;
    QPaintBox *myPaintBox, *speedPaintBox;
    SettingsDialog *mySettings;
    QLabel *estado, *marca;
    QPainterPath *path,*path2,*path3;



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

    typedef enum{
        ACK = 0x0D,
        ALIVE=0xF0,
        FIRMWARE = 0xF1,
        GET_IR = 0xA0,
        SET_POWER = 0xA1,
        SET_SERVO = 0xA2,
        GET_DISTANCE = 0xA3,
        GET_SPEED = 0xA4,
        OTHERS
    }_eID;

    _eID estadoComandos;

    typedef enum{
        LOBBY,
        GAME,
        FINISH
    }_eESTADOS;

    _eESTADOS gameState;

    typedef struct{
        uint8_t timeOut;
        uint8_t cheksum;
        uint8_t payLoad[50];
        uint8_t nBytes;
        uint8_t index;
    }_sDatos ;

    _sDatos rxData, txData;

    typedef union {
        float f32;
        int i32;
        int8_t i8[4];
        unsigned int ui32;        
        unsigned short ui16[2];
        short i16[2];
        uint8_t ui8[4];

        char chr[4];
        unsigned char uchr[4];
    }_udat;

    _udat myWord;

    typedef struct{
        uint8_t estado;
        uint8_t event;
        int32_t timeDown;
        int32_t timeDiff;
    }_sTeclas;

    _sTeclas ourButton[4];

    typedef union{
        struct{
            uint8_t servoAngle :1;
            uint8_t reserved:7;
        }individualFlags;
        uint8_t allFlags;
    }_bGeneralFlags;

    _bGeneralFlags myFlags;

    int32_t powerM1 = 0, powerM2 = 0;
    uint32_t distancia_us = 0, speedM1 = 0, speedM2 = 0;
    uint16_t valueIR1 = 0, valueIR2 = 0;
    int8_t anguloServo, count, auxAngulo, paso = 0;
    int posxT, posyT;
};
#endif // MAINWINDOW_H
