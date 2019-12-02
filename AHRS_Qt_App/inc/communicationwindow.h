#ifndef COMMUNICATIONWINDOW_H
#define COMMUNICATIONWINDOW_H

#include <QDialog>
#include <QDebug>
#include <QList>
#include <QSerialPortInfo>
#include <QSerialPort>
#include <QDateTime>
#include <QEvent>
#include <QMessageBox>
#include <QColorDialog>
#include <QSettings>

#include "bluetooth.h"

namespace Ui {
class CommunicationWindow;
}

class CommunicationWindow : public QDialog
{
    Q_OBJECT

public slots:

    void Disconnect_Slot();
    void Serial_Interface_Slot(Status_Codes status);

    void Send_Data_to_robot_Slot();
    void Parsed_frame_OK_Slot();

public:

    explicit CommunicationWindow(QWidget *parent = nullptr);
    ~CommunicationWindow() override;

    QString Get_PortName();
    void Fill_Data_to_robot(Data_to_Robot Data);

private slots:

    void on_pushButton_Search_clicked();
    void on_pushButton_Connect_clicked();
    void on_pushButton_Disconnect_clicked();
    void on_pushButton_Clear_clicked();
    void on_pushButton_Cancel_clicked();
    void on_pushButton_Continue_clicked();

    void on_comboBox_Baud_currentIndexChanged(int index);
    void on_comboBox_Bits_currentIndexChanged(int index);
    void on_comboBox_Parity_currentIndexChanged(int index);
    void on_comboBox_Stop_currentIndexChanged(int index);
    void on_comboBox_Control_currentIndexChanged(int index);

private:

    Ui::CommunicationWindow *ui;

    Bluetooth *BT = new Bluetooth;

    QString                     PortName;

    QSerialPort::BaudRate       baud;
    QSerialPort::DataBits       bits;
    QSerialPort::Parity         parity;
    QSerialPort::StopBits       stop;
    QSerialPort::FlowControl    control;

    QString m_sSettingsFile;

    bool Serial_is_open;

    void loadSettings();
    void saveSettings();

    void CommunicationWindow_set_default();
    void CommunicationWindow_addToLogs(QString message);
    void CommunicationWindow_sendMessageToDevice(QString message);

    void closeEvent(QCloseEvent *event) override;

signals:

    void Connection_OK_Signal();
    void Connection_FAIL_Signal();
    void Parsed_frame_OK_Signal(Data_from_Robot);
    void Timeout_Error();
};

#endif // COMMUNICATIONWINDOW_H
