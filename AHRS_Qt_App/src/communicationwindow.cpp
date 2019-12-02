#include "communicationwindow.h"
#include "ui_communicationwindow.h"

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::Disconnect_Slot()
{
    this->show();
    on_pushButton_Disconnect_clicked();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::Serial_Interface_Slot(Status_Codes status)
{
    switch (status) {

        case Open_connection_OK:

            Serial_is_open = true;
            this->CommunicationWindow_addToLogs("Otwarto port szeregowy");
            ui->label_ConnectionStatus->setText("<font color='green'>Połączono</font>");
            break;

        case Open_connection_FAIL:

            this->CommunicationWindow_addToLogs("Otwarcie portu szeregowego się nie powiodło !");
            ui->label_ConnectionStatus->setText("<font color='red'>Niepowodzenie</font>");
            break;

        case Close_connection_OK:

            Serial_is_open = false;
            this->CommunicationWindow_addToLogs("Zamknięto port szeregowy");
            ui->label_ConnectionStatus->setText("<font color='orange'>Rozłączono</font>");
            break;

        case Close_connection_FAIL:

            this->CommunicationWindow_addToLogs("Brak otwartego portu !");
            break;

        case Port_is_busy:

            this->CommunicationWindow_addToLogs("Port już jest otwarty !");
            break;

        case TimeoutError:

            emit Timeout_Error();
            break;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::Send_Data_to_robot_Slot()
{
    BT->Send_frame();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::Parsed_frame_OK_Slot()
{
    emit Parsed_frame_OK_Signal( BT->Get_DF_Robot() );
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CommunicationWindow::CommunicationWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CommunicationWindow)
{
    ui->setupUi(this);

    m_sSettingsFile = QApplication::applicationDirPath().left(1) + ":/settings.ini";

    loadSettings();

    // CONNECT:
    connect(BT, SIGNAL( Serial_Interface_Signal(Status_Codes) ), this, SLOT( Serial_Interface_Slot(Status_Codes) ));
    connect(BT, SIGNAL( Parsed_frame_OK_Signal() ), this, SLOT( Parsed_frame_OK_Slot() ));

    QPixmap Bluetooth(":/new/prefix1/png/Bluetooth.png");

    int w = ui->label_Bluetooth->width();
    int h = ui->label_Bluetooth->height();

    ui->label_Bluetooth->setPixmap( Bluetooth.scaled(w, h, Qt::KeepAspectRatio) );
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

CommunicationWindow::~CommunicationWindow()
{
    delete ui;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::Fill_Data_to_robot(Data_to_Robot Data)
{
    BT->Set_DT_Robot(Data);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Search_clicked()
{
    ui->comboBox_Devices->clear();

    this->CommunicationWindow_addToLogs("Szukam urządzeń...");

    QList<QSerialPortInfo> devices;
    devices = QSerialPortInfo::availablePorts();

    int repeat = 0;

    for(int i = 0; i < devices.count(); i++) {

        for (int j = 0; j < devices.count(); j++) {

            if( devices.at(i).portName() + "\t" + devices.at(i).description() == ui->comboBox_Devices->itemText(j) ) {

                repeat = 1;
                break;
            }
        }

        this->CommunicationWindow_addToLogs("Znalazłem urządzenie: " + devices.at(i).portName() + " " + devices.at(i).description());

        if( repeat == 0 ) {

            ui->comboBox_Devices->addItem(devices.at(i).portName() + "\t" + devices.at(i).description());
        }

        repeat = 0;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::CommunicationWindow_addToLogs(QString message)
{
    QString currentDateTime = QDateTime::currentDateTime().toString("yyyy.MM.dd hh:mm:ss");
    ui->textBrowser_Logs->append(currentDateTime + "\t" + message);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox messageBox(QMessageBox::Question,
                           tr("BBot"),
                           tr("Czy na pewno chcesz zakończyć ? \n"),
                           QMessageBox::Yes | QMessageBox::No);

    messageBox.setButtonText(QMessageBox::Yes, tr("Tak"));
    messageBox.setButtonText(QMessageBox::No,  tr("Nie"));

    if(messageBox.exec() != QMessageBox::Yes) {

        event->ignore();
    }
    else {

        exit(0);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Connect_clicked()
{
    if(ui->comboBox_Devices->count() == 0) {

      this->CommunicationWindow_addToLogs("Nie wykryto żadnych urządzeń!");
      return;
    }

    PortName = ui->comboBox_Devices->currentText().split("\t").first();

    BT->Open_connection(PortName);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Disconnect_clicked()
{
    BT->Close_connection();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Clear_clicked()
{
    ui->textBrowser_Logs->clear();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_comboBox_Baud_currentIndexChanged(int index)
{
    switch(index) {

        case 0:
            baud = QSerialPort::Baud1200;
            break;

        case 1:
            baud = QSerialPort::Baud2400;
            break;

        case 2:
            baud = QSerialPort::Baud4800;
            break;

        case 3:
            baud = QSerialPort::Baud9600;
            break;

        case 4:
            baud = QSerialPort::Baud19200;
            break;

        case 5:
            baud = QSerialPort::Baud38400;
            break;

        case 6:
            baud = QSerialPort::Baud57600;
            break;

        case 7:
            baud = QSerialPort::Baud115200;
            break;

        case 8:
            baud = QSerialPort::UnknownBaud;
            break;
    }

    saveSettings();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_comboBox_Bits_currentIndexChanged(int index)
{
    switch(index) {

        case 0:
            bits = QSerialPort::Data5;
            break;

        case 1:
            bits = QSerialPort::Data6;
            break;

        case 2:
            bits = QSerialPort::Data7;
            break;

        case 3:
            bits = QSerialPort::Data8;
            break;

        case 4:
            bits = QSerialPort::UnknownDataBits;
            break;
    }

    saveSettings();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_comboBox_Parity_currentIndexChanged(int index)
{
    switch(index) {

        case 0:
            parity = QSerialPort::NoParity;
            break;

        case 1:
            parity = QSerialPort::EvenParity;
            break;

        case 2:
            parity = QSerialPort::OddParity;
            break;

        case 3:
            parity = QSerialPort::MarkParity;
            break;

        case 4:
            parity = QSerialPort::SpaceParity;
            break;

        case 5:
            parity = QSerialPort::UnknownParity;
            break;
    }

    saveSettings();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_comboBox_Stop_currentIndexChanged(int index)
{
    switch(index) {

        case 0:
            stop = QSerialPort::OneStop;
            break;

        case 1:
            stop = QSerialPort::TwoStop;
            break;

        case 2:
            stop = QSerialPort::StopBits::OneAndHalfStop;
            break;

        case 3:
            stop = QSerialPort::StopBits::UnknownStopBits;
            break;
    }

    saveSettings();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_comboBox_Control_currentIndexChanged(int index)
{
    switch(index) {

        case 0:
            control = QSerialPort::FlowControl::NoFlowControl;
            break;

        case 1:
            control = QSerialPort::FlowControl::HardwareControl;
            break;

        case 2:
            control = QSerialPort::FlowControl::SoftwareControl;
            break;

        case 3:
            control = QSerialPort::FlowControl::UnknownFlowControl;
            break;
    }

    saveSettings();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::loadSettings()
{
    QSettings settings(m_sSettingsFile);

    int s_baud = settings.value("s_baud").toInt();
    int s_bits = settings.value("s_bits").toInt();
    int s_parity = settings.value("s_parity").toInt();
    int s_stop = settings.value("s_stop").toInt();
    int s_control = settings.value("s_control").toInt();

    ui->comboBox_Baud->setCurrentIndex(s_baud);
    ui->comboBox_Bits->setCurrentIndex(s_bits);
    ui->comboBox_Parity->setCurrentIndex(s_parity);
    ui->comboBox_Stop->setCurrentIndex(s_stop);
    ui->comboBox_Control->setCurrentIndex(s_control);

    qDebug() << "Wczytano baud: "    << s_baud;
    qDebug() << "Wczytano bits: "    << s_bits;
    qDebug() << "Wczytano parity: "  << s_parity;
    qDebug() << "Wczytano stop: "    << s_stop;
    qDebug() << "Wczytano control: " << s_control;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::saveSettings()
{
    QSettings settings(m_sSettingsFile);

    int s_baud    = ui->comboBox_Baud->currentIndex();
    int s_bits    = ui->comboBox_Bits->currentIndex();
    int s_parity  = ui->comboBox_Parity->currentIndex();
    int s_stop    = ui->comboBox_Stop->currentIndex();
    int s_control = ui->comboBox_Control->currentIndex();

    settings.setValue("s_baud",    s_baud);
    settings.setValue("s_bits",    s_bits);
    settings.setValue("s_parity",  s_parity);
    settings.setValue("s_stop",    s_stop);
    settings.setValue("s_control", s_control);

    qDebug() << "Zapisano baud: "    << s_baud;
    qDebug() << "Zapisano bits: "    << s_bits;
    qDebug() << "Zapisano parity: "  << s_parity;
    qDebug() << "Zapisano stop: "    << s_stop;
    qDebug() << "Zapisano control: " << s_control;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

QString CommunicationWindow::Get_PortName()
{
    return PortName;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Cancel_clicked()
{

    QMessageBox messageBox(QMessageBox::Question,
                           tr("BBot"),
                           tr("Czy na pewno chcesz zakończyć ? \n"),
                           QMessageBox::Yes | QMessageBox::No);

    messageBox.setButtonText(QMessageBox::Yes, tr("Tak"));
    messageBox.setButtonText(QMessageBox::No,  tr("Nie"));

    if(messageBox.exec() == QMessageBox::Yes) {

        exit(0);
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void CommunicationWindow::on_pushButton_Continue_clicked()
{

    if( Serial_is_open == true ) {

        this->hide();
        emit Connection_OK_Signal();
    }
    else {

        QMessageBox messageBox(QMessageBox::Information,
                               tr("BBot"),
                               tr("Najpierw połącz się z urządzeniem"),
                               QMessageBox::Ok);

        messageBox.exec();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
