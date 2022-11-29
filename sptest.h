#ifndef SPTEST_H
#define SPTEST_H

#include <QMainWindow>

#include "ui_sptest.h"
#include "serialport.h"

QT_BEGIN_NAMESPACE
namespace Ui { class sptest; }
QT_END_NAMESPACE

class sptest : public QMainWindow
{
    Q_OBJECT

public:
    sptest(QWidget *parent = nullptr);

private slots:
    void on_btnOpen_clicked();

    void on_btnSend_clicked();

    void on_btnReceive_clicked();

    void on_btnClose_clicked();

private:
    Ui::sptest ui;

    QMap<QString, sp::BaudRate> baudRateOptions;
    QMap<QString, sp::NumDataBits> dataBitsOptions;
    QMap<QString, sp::Parity> parityOptions;
    QMap<QString, sp::NumStopBits> stopBitsOptions;
    QMap<QString, sp::HardwareFlowControl> hwControlOptions;
    QMap<QString, sp::SoftwareFlowControl> swControlOptions;
    void FillMaps();
    void FillComboBoxes();

    sp::SerialPort sp;
};
#endif // SPTEST_H
