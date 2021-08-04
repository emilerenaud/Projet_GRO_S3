#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QCloseEvent>
#include <QDebug>
#include <QtWidgets>
#include <QJsonObject>
#include <QJsonDocument>
#include <QSerialPortInfo>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts>

#include <QPainter>

// Propres librairies
#include "csvwriter.h"
#include "serialprotocol.h"

#define PI 3.14159265

// Classe definissant l'application
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    const qint32 BAUD_RATE = 115200;

    explicit MainWindow(int updateRate, QWidget *parent = nullptr);
    explicit MainWindow(QWidget *parent = nullptr);
    virtual ~MainWindow() override;
    void closeEvent(QCloseEvent *event) override;

    void sendMessage(QString msg);
    void setUpdateRate(int rateMs);

    void onPeriodicUpdate();
    void onMessageReceived(QString);

private slots:
    void receiveFromSerial(QString);
    void sendPulseSetting();
    void sendPulseStart();
    void manageRecording(int);
    void changeJsonKeyValue();
    void startSerialCom(QString);
    void sendPID();
    void getPID();
    void resetGraph();

    void on_pushButton_hauteur_clicked();

    void on_horizontalScrollBar_sliderMoved(int position);

private:
    void connectTimers(int updateRate);
    void connectButtons();
    void connectSerialPortRead();
    void connectSpinBoxes();
    void startRecording();
    void stopRecording();
    void connectTextInputs();
    void connectComboBox();
    void portCensus();

    bool record = false;
    CsvWriter* writer_;
    QTimer updateTimer_;
    QString msgReceived_{""};
    QString msgBuffer_{""};
    SerialProtocol* serialCom_=nullptr;

    QString JsonKey_;
    QLineSeries series_;
    QLineSeries seriesGoal_;
    QChart chart_;
    void paintEvent(QPaintEvent *event);

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double angle = 50;
    double distance = 0.75;

protected:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
