#include "sptest.h"
#include "./ui_sptest.h"

#include <QMessageBox>

sptest::sptest(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    FillMaps();
    FillComboBoxes();
}

void sptest::FillMaps()
{
    // baud rate
    baudRateOptions.insert("0",sp::BaudRate::B_0);
    baudRateOptions.insert("50",sp::BaudRate::B_50);
    baudRateOptions.insert("75",sp::BaudRate::B_75);
    baudRateOptions.insert("110",sp::BaudRate::B_110);
    baudRateOptions.insert("134",sp::BaudRate::B_134);
    baudRateOptions.insert("150",sp::BaudRate::B_150);
    baudRateOptions.insert("200",sp::BaudRate::B_200);
    baudRateOptions.insert("300",sp::BaudRate::B_300);
    baudRateOptions.insert("600",sp::BaudRate::B_600);
    baudRateOptions.insert("1200",sp::BaudRate::B_1200);
    baudRateOptions.insert("1800",sp::BaudRate::B_1800);
    baudRateOptions.insert("2400",sp::BaudRate::B_2400);
    baudRateOptions.insert("4800",sp::BaudRate::B_4800);
    baudRateOptions.insert("9600",sp::BaudRate::B_9600);
    baudRateOptions.insert("19200",sp::BaudRate::B_19200);
    baudRateOptions.insert("38400",sp::BaudRate::B_38400);
    baudRateOptions.insert("57600",sp::BaudRate::B_57600);
    baudRateOptions.insert("115200",sp::BaudRate::B_115200);
    baudRateOptions.insert("230400",sp::BaudRate::B_230400);
    baudRateOptions.insert("460800",sp::BaudRate::B_460800);

    // data bits
    dataBitsOptions.insert("5", sp::NumDataBits::FIVE);
    dataBitsOptions.insert("6", sp::NumDataBits::SIX);
    dataBitsOptions.insert("7", sp::NumDataBits::SEVEN);
    dataBitsOptions.insert("8", sp::NumDataBits::EIGHT);

    // parity
    parityOptions.insert("NONE", sp::Parity::NONE);
    parityOptions.insert("EVEN", sp::Parity::EVEN);
    parityOptions.insert("ODD", sp::Parity::ODD);

    // stop bits
    stopBitsOptions.insert("1", sp::NumStopBits::ONE);
    stopBitsOptions.insert("2", sp::NumStopBits::TWO);

    // hw control
    hwControlOptions.insert("OFF", sp::HardwareFlowControl::OFF);
    hwControlOptions.insert("ON", sp::HardwareFlowControl::ON);

    // sw control
    swControlOptions.insert("OFF", sp::SoftwareFlowControl::OFF);
    swControlOptions.insert("ON", sp::SoftwareFlowControl::ON);
}

void sptest::FillComboBoxes()
{
    // baud rate
    ui.baudRate->addItems(QStringList(baudRateOptions.keys()));
    ui.baudRate->setCurrentIndex(2);

    // data bits
    ui.numDataBits->addItems(QStringList(dataBitsOptions.keys()));
    ui.numDataBits->setCurrentIndex(3);

    // parity
    ui.parity->addItems(QStringList(parityOptions.keys()));
    ui.parity->setCurrentIndex(1);

    // stop bits
    ui.numStopBits->addItems(QStringList(stopBitsOptions.keys()));
    ui.numStopBits->setCurrentIndex(0);

    // hw control
    ui.hardwareFlowControl->addItems(QStringList(hwControlOptions.keys()));
    ui.hardwareFlowControl->setCurrentIndex(0);

    // sw control
    ui.softwareFlowControl->addItems(QStringList(swControlOptions.keys()));
    ui.softwareFlowControl->setCurrentIndex(0);
}


void sptest::on_btnOpen_clicked()
{
    sp::PortConfig pc;
    pc.SetBaudRate(baudRateOptions.value(ui.baudRate->currentText()));
    pc.SetNumDataBits(dataBitsOptions.value(ui.numDataBits->currentText()));
    pc.SetParity(parityOptions.value(ui.parity->currentText()));
    pc.SetNumStopBits(stopBitsOptions.value(ui.numStopBits->currentText()));
    pc.SetTimeout(ui.timeout->text().toInt());
    pc.SetHwFlowControl(hwControlOptions.value(ui.hardwareFlowControl->currentText()));
    pc.SetSwFlowControl(swControlOptions.value(ui.softwareFlowControl->currentText()));

    if (!sp.open(ui.portName->text().toStdString(), pc))
    {
        QMessageBox msgBox;
        msgBox.setText(QString("failed to open port. errno = %1:")
                        .arg(errno));
        msgBox.setInformativeText(ui.portName->text());
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
    }
    else
    {
        ui.txtMessage->appendPlainText(QString("port opened successfully: %1")
                                       .arg(ui.portName->text()));
    }
}

void sptest::on_btnSend_clicked()
{
    int n = sp.Write(ui.txtSend->toPlainText().toStdString());
    ui.txtMessage->appendPlainText(QString("sp.Write returned: %1")
                                   .arg(n));
}

void sptest::on_btnReceive_clicked()
{
    std::string s;
    int n = sp.Read(s);
    ui.txtMessage->appendPlainText(QString("sp.Read returned: %1")
                                   .arg(n));
    if (n > 0)
    {
        ui.txtReceive->appendPlainText(QString(s.c_str()));
    }
}

void sptest::on_btnClose_clicked()
{
    sp.close();
    ui.txtMessage->appendPlainText(QString("port closed"));
}
