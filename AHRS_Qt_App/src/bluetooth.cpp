#include "bluetooth.h"
#include <stdio.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

byte CRC8_SingleByte(byte CRC_prev, byte Data) {

    CRC_prev ^= Data;

    for(byte Idx = 0; Idx < 8; ++Idx) {

        if( (CRC_prev & 0x80) != 0 ) {

            CRC_prev = static_cast<byte>( (CRC_prev << 1) ^ POLYNOMIAL_9 );
        } else {

            CRC_prev = static_cast<byte>(CRC_prev << 1);
        }
    }

    return CRC_prev;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

byte CRC8_DataArray(byte *pData, byte Len) {

    byte CRC_final = static_cast<byte>(0xFF);

    for(byte Idx = 0; Idx < Len; ++Idx) {

        CRC_final = CRC8_SingleByte(CRC_final, pData[Idx]);
    }

    return CRC_final;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int16_t Merge_bytes(uint8_t _lower_byte, uint8_t _higher_byte) {

    int16_t dataBoth = 0x00;

    dataBoth = static_cast<int16_t>(_higher_byte << 8);
    dataBoth |= _lower_byte;

    return dataBoth;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int8_t Divide_bytes(int16_t data, uint8_t which_byte) {

    if(which_byte == 'L') {

        return static_cast<int8_t>(data & 0xFF);
    }
    else if(which_byte == 'H') {

        return ( data >> 8 );
    }

    return 0;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Bluetooth::Bluetooth()
{
    DT_Robot.Complementary_filter_weight = 0;
    DT_Robot.Kalman_procces_variance = 0;
    DT_Robot.Kalman_measure_variance = 0;
    DT_Robot.Madgwick_filter_beta = 0;
    DT_Robot.Which_filter = 0;

    DF_Robot.Complementary_w = 1;
    DF_Robot.Complementary_x = 0;
    DF_Robot.Complementary_y = 0;
    DF_Robot.Complementary_z = 0;

    DF_Robot.Kalman_w = 1;
    DF_Robot.Kalman_x = 0;
    DF_Robot.Kalman_y = 0;
    DF_Robot.Kalman_z = 0;

    DF_Robot.Madgwick_w = 1;
    DF_Robot.Madgwick_x = 0;
    DF_Robot.Madgwick_y = 0;
    DF_Robot.Madgwick_z = 0;

    DF_Robot.g_x_dgs = 0;
    DF_Robot.g_y_dgs = 0;
    DF_Robot.g_z_dgs = 0;
    DF_Robot.a_x_g = 0;
    DF_Robot.a_y_g = 0;
    DF_Robot.a_z_g = 0;
    DF_Robot.m_x_uT = 0;
    DF_Robot.m_y_uT = 0;
    DF_Robot.m_z_uT = 0;

    TimeoutTimer = new QTimer(this);

    connect(TimeoutTimer, SIGNAL( timeout() ), this, SLOT( Timeout_Test() ) );
    connect(Device, SIGNAL( readyRead() ), this, SLOT( Receive_frame() ));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Bluetooth::~Bluetooth()
{
    delete Device;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Open_connection(QString portName)
{
    this->Device->setPortName(portName);

    // OTWÓRZ I SKONFIGURUJ PORT:
    if( !Device->isOpen() ) {

        if( Device->open(QSerialPort::ReadWrite) ) {

            this->Device->setBaudRate(QSerialPort::BaudRate::Baud9600);
            this->Device->setDataBits(QSerialPort::DataBits::Data8);
            this->Device->setParity(QSerialPort::Parity::NoParity);
            this->Device->setStopBits(QSerialPort::StopBits::OneStop);
            this->Device->setFlowControl(QSerialPort::FlowControl::NoFlowControl);

            emit Serial_Interface_Signal(Open_connection_OK);
            TimeoutTimer->start(2000);
        }
        else {

            emit Serial_Interface_Signal(Open_connection_FAIL);
        }
    }
    else {

        emit Serial_Interface_Signal(Port_is_busy);

        return;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Close_connection()
{
    if(this->Device->isOpen()) {

        Device->close();

        emit Serial_Interface_Signal(Close_connection_OK);
        TimeoutTimer->stop();
    }
    else {

        emit Serial_Interface_Signal(Close_connection_FAIL);

        return;
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Receive_frame()
{
    //qDebug() << "Odbieram ! \n";

    if( Device->isOpen() && Device->isReadable()) {

        // Case 1: Read all data from Serial Port
        Device->flush();
        QByteArray array = Device->readAll();
        Device->waitForReadyRead();

        // Case 2: Copy this data to temporary frame
        for(int i = 0; i < DATA_FRAME_FROM_ROBOT_SIZE; i++) {

            Data_frame_from_robot[i] = array[i];
        }

        int8_t CRC_received = Data_frame_from_robot[DATA_FRAME_FROM_ROBOT_SIZE - 1];
        int8_t CRC_actual = CRC8_DataArray(Data_frame_from_robot, DATA_FRAME_FROM_ROBOT_SIZE - 1);

        int16_t test_int = Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[DATA_FRAME_FROM_ROBOT_SIZE-3]),
                                       static_cast<uint8_t>(Data_frame_from_robot[DATA_FRAME_FROM_ROBOT_SIZE-2]) );

        if( CRC_actual == CRC_received && CRC_received != 0 && test_int == -32768) {

            Parse_data_frame();
            TimeoutTimer->start(2000);
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Parse_data_frame()
{

    DF_Robot.Complementary_w = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[0]),
                                                                static_cast<uint8_t>(Data_frame_from_robot[1])  ) ) / 1000;
    DF_Robot.Complementary_x = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[2]),
                                                                static_cast<uint8_t>(Data_frame_from_robot[3])  ) ) / 1000;
    DF_Robot.Complementary_y = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[4]),
                                                                static_cast<uint8_t>(Data_frame_from_robot[5])  ) ) / 1000;
    DF_Robot.Complementary_z = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[6]),
                                                                static_cast<uint8_t>(Data_frame_from_robot[7])  ) ) / 1000;

    DF_Robot.Kalman_w = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[8]),
                                                         static_cast<uint8_t>(Data_frame_from_robot[9])  ) ) / 1000;
    DF_Robot.Kalman_x = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[10]),
                                                         static_cast<uint8_t>(Data_frame_from_robot[11])  ) ) / 1000;
    DF_Robot.Kalman_y = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[12]),
                                                         static_cast<uint8_t>(Data_frame_from_robot[13])  ) ) / 1000;
    DF_Robot.Kalman_z = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[14]),
                                                         static_cast<uint8_t>(Data_frame_from_robot[15])  ) ) / 1000;

    DF_Robot.Madgwick_w = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[16]),
                                                           static_cast<uint8_t>(Data_frame_from_robot[17])  ) ) / 1000;
    DF_Robot.Madgwick_x = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[18]),
                                                           static_cast<uint8_t>(Data_frame_from_robot[19])  ) ) / 1000;
    DF_Robot.Madgwick_y = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[20]),
                                                           static_cast<uint8_t>(Data_frame_from_robot[21])  ) ) / 1000;
    DF_Robot.Madgwick_z = static_cast<double>( Merge_bytes(static_cast<uint8_t>(Data_frame_from_robot[22]),
                                                           static_cast<uint8_t>(Data_frame_from_robot[23])  ) ) / 1000;

    emit Parsed_frame_OK_Signal();

    /*
    qDebug() << "Napiecie LiPol: "             << DF_Robot.Lipol_voltage;
    qDebug() << "Filtr Roll: "  << DF_Robot.Filter_roll;
    qDebug() << "Filtr Pitch: " << DF_Robot.Filter_pitch;
    qDebug() << "Filtr Yaw: "   << DF_Robot.Filter_yaw;
    qDebug() << "Predkosc lewego silnika: "    << DF_Robot.Left_engine_speed;
    qDebug() << "Predskoc prawego silnika: "   << DF_Robot.Right_engine_speed;
    qDebug() << "g_x_dgs: " << DF_Robot.g_x_dgs;
    qDebug() << "g_y_dgs: " << DF_Robot.g_y_dgs;
    qDebug() << "g_z_dgs: " << DF_Robot.g_z_dgs;
    qDebug() << "a_x_g: " << DF_Robot.a_x_g;
    qDebug() << "a_y_g: " << DF_Robot.a_y_g;
    qDebug() << "a_z_g: " << DF_Robot.a_z_g;
    qDebug() << "m_x_uT: " << DF_Robot.m_x_uT;
    qDebug() << "m_y_uT: " << DF_Robot.m_y_uT;
    qDebug() << "m_z_uT: " << DF_Robot.m_z_uT;
    */
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Send_frame()
{
    // Case 1: Fill data frame

    // Filters data
    int16_t Complementary_filter_weight = static_cast<int16_t>(DT_Robot.Complementary_filter_weight * 1000);
    int16_t Kalman_filter_process_variance = static_cast<int16_t>(DT_Robot.Kalman_procces_variance);
    int16_t Kalman_filter_measure_variance = static_cast<int16_t>(DT_Robot.Kalman_measure_variance);
    int16_t Madgwick_filter_beta = static_cast<int16_t>(DT_Robot.Madgwick_filter_beta * 1000);

    // Additional data
    int8_t Which_filter = static_cast<int8_t>(DT_Robot.Which_filter);

    int8_t Data_frame[DATA_FRAME_TO_ROBOT_SIZE];

    Data_frame[0] = Divide_bytes(Complementary_filter_weight, 'L');
    Data_frame[1] = Divide_bytes(Complementary_filter_weight, 'H');
    Data_frame[2] = Divide_bytes(Kalman_filter_process_variance, 'L');
    Data_frame[3] = Divide_bytes(Kalman_filter_process_variance, 'H');
    Data_frame[4] = Divide_bytes(Kalman_filter_measure_variance, 'L');
    Data_frame[5] = Divide_bytes(Kalman_filter_measure_variance, 'H');
    Data_frame[6] = Divide_bytes(Madgwick_filter_beta, 'L');
    Data_frame[7] = Divide_bytes(Madgwick_filter_beta, 'H');


    Data_frame[8] = Which_filter;

    Data_frame[9] = CRC8_DataArray(Data_frame, DATA_FRAME_TO_ROBOT_SIZE - 1);

    // Case 2: Save data frame as char array
    char Data[DATA_FRAME_TO_ROBOT_SIZE];

    for (int i = 0; i < DATA_FRAME_TO_ROBOT_SIZE; i++) {

        Data[i] = Data_frame[i];
    }

    // Case 3: Send data frame
    if( Device->isWritable() ) {

        /*
        qDebug() << "Wysyłam dane: ";
        qDebug() << "Kp (wychylenie): "             << Angle_Kp;
        qDebug() << "Ki (wychylenie): "             << Angle_Ki;
        qDebug() << "Kd (wychylenie): "             << Angle_Kd;
        qDebug() << "Kp (predkosc): "               << Speed_Kp;
        qDebug() << "Ki (predkosc): "               << Speed_Ki;
        qDebug() << "Kd (predkosc): "               << Speed_Kd;
        qDebug() << "Waga filtru: "                 << Complementary_filter_weight;
        qDebug() << "Wariancja procesu: "           << Kalman_filter_process_variance;
        qDebug() << "Wzmocnienie beta: "            << Madgwick_filter_beta;
        qDebug() << "Predkosc lewego silnika: "     << Left_engine_speed;
        qDebug() << "Predkosc prawego silnika: "    << Right_engine_speed;
        qDebug() << "Przycisk awaryjny: "           << Emergency_stop;
        qDebug() << "Ktory filtr: "                 << Which_filter;
        */


        Device->write( Data, DATA_FRAME_TO_ROBOT_SIZE);
        Device->waitForBytesWritten(20);
        Device->flush();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Timeout_Test()
{
    //Close_connection();
    /*
    emit Serial_Interface_Signal(TimeoutError);

    QMessageBox messageBox(QMessageBox::Information,
                           tr("BBot"),
                           tr("Nastąpiło automatyczne rozłączenie. Przekroczony limit czasu żądania ! \n"),
                           QMessageBox::Ok);

    if(messageBox.exec() == QMessageBox::Ok) {

        messageBox.close();
    }
    */
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void Bluetooth::Set_DT_Robot(Data_to_Robot Data)
{
    DT_Robot = Data;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Data_from_Robot Bluetooth::Get_DF_Robot()
{
    return DF_Robot;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
