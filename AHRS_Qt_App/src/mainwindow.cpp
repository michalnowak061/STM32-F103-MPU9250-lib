#include "mainwindow.h"
#include "ui_mainwindow.h"

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Default variables values
    Show_Gyroscope_X = true; Show_Gyroscope_Y = true; Show_Gyroscope_Z = true;
    Show_Accelerometer_X = true; Show_Accelerometer_Y = true; Show_Accelerometer_Z = true;
    Show_Magnetometer_X = true; Show_Magnetometer_Y = true; Show_Magnetometer_Z = true;

    Show_Filter_Roll = true; Show_Filter_Pitch = true; Show_Filter_Yaw = true;

    Complementary_Graph_Run = true;
    Kalman_Graph_Run = true;
    Madgwick_Graph_Run = true;

    Data_to.Complementary_filter_weight = 0;

    m_sSettingsFile = QApplication::applicationDirPath().left(1) + ":/main_settings.ini";
    loadSettings();

    //MainWindow_Default_View();
    MainWindow_Setup_Icons();

    // Setup real time graphs
    MainWindow_Setup_Complementary_Graph();
    MainWindow_Setup_Kalman_Graph();
    MainWindow_Setup_Madgwick_Graph();
    //MainWindow_Setup_Filter_Graph();

    // Connection with CommunicationWindow
    connect(this, SIGNAL( Disconnect_Signal() ), CW, SLOT( Disconnect_Slot() ) );
    connect(this, SIGNAL( Send_data_Signal() ), CW, SLOT( Send_Data_to_robot_Slot() ) );

    connect(CW, SIGNAL( Connection_OK_Signal() ), this, SLOT( Connection_OK_Slot() ) );
    connect(CW, SIGNAL( Parsed_frame_OK_Signal(Data_from_Robot) ), this, SLOT( MainWindow_realtimeDataSlot(Data_from_Robot) ));

    connect(CW, SIGNAL( Timeout_Error() ), this, SLOT( on_pushButton_ConnectDisconnect_clicked() ) );

    // Setup OpenGL visualisation
    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;
    QWidget *w = new QWidget;

    container->addWidget(GW);
    w->setLayout(container);
    mainLayout->addWidget(w);

    // Setup CommunicationWindow
    CW->exec();

    //Splitter_Position = ui->splitter->sizes();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

MainWindow::~MainWindow()
{
    delete ui;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::closeEvent(QCloseEvent *event)
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

        event->accept();
    }
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Complementary_Graph()
{
    ui->Complementary_Graph->addGraph(); // red line
    ui->Complementary_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Complementary_Graph->addGraph(); // green line
    ui->Complementary_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Complementary_Graph->addGraph(); // blue line
    ui->Complementary_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Complementary_Graph->xAxis->setTicker(timeTicker);
    ui->Complementary_Graph->axisRect()->setupFullAxesBox();
    ui->Complementary_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Complementary_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Complementary_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Complementary_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Complementary_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Kalman_Graph()
{
    ui->Kalman_Graph->addGraph(); // red line
    ui->Kalman_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Kalman_Graph->addGraph(); // green line
    ui->Kalman_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Kalman_Graph->addGraph(); // blue line
    ui->Kalman_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Kalman_Graph->xAxis->setTicker(timeTicker);
    ui->Kalman_Graph->axisRect()->setupFullAxesBox();
    ui->Kalman_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Kalman_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Kalman_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Kalman_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Kalman_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Madgwick_Graph()
{
    ui->Madgwick_Graph->addGraph(); // red line
    ui->Madgwick_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Madgwick_Graph->addGraph(); // green line
    ui->Madgwick_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Madgwick_Graph->addGraph(); // blue line
    ui->Madgwick_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Madgwick_Graph->xAxis->setTicker(timeTicker);
    ui->Madgwick_Graph->axisRect()->setupFullAxesBox();
    ui->Madgwick_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Madgwick_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Madgwick_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Madgwick_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Madgwick_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Display_IMU_data()
{
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed() / 1000.0; // time elapsed since start of demo, in seconds

    // Complementary filter
    QVector3D Complementary_euler = QQuaternion(-Data_from.Complementary_w, -Data_from.Complementary_y, -Data_from.Complementary_z, -Data_from.Complementary_x).toEulerAngles();
    double Complementary_Roll  = Complementary_euler.z();
    double Complementary_Pitch = Complementary_euler.x();
    double Complementary_Yaw   = Complementary_euler.y();

    if(Show_Gyroscope_X == true) ui->Complementary_Graph->graph(0)->addData(key, Complementary_Roll);
    if(Show_Gyroscope_Y == true) ui->Complementary_Graph->graph(1)->addData(key, Complementary_Pitch);
    if(Show_Gyroscope_Z == true) ui->Complementary_Graph->graph(2)->addData(key, Complementary_Yaw);

    ui->Complementary_Graph->graph(0)->rescaleValueAxis(true);
    ui->Complementary_Graph->graph(1)->rescaleValueAxis(true);
    ui->Complementary_Graph->graph(2)->rescaleValueAxis(true);

    ui->Complementary_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Complementary_Graph_Run == true ) {

        ui->Complementary_Graph->replot();

        ui->lcdNumber_Gyroscope_X->display(Complementary_Roll);
        ui->lcdNumber_Gyroscope_Y->display(Complementary_Pitch);
        ui->lcdNumber_Gyroscope_Z->display(Complementary_Yaw);
    }

    ui->Complementary_Visualisation->setQuaternion(Data_from.Complementary_w, Data_from.Complementary_x, Data_from.Complementary_y, Data_from.Complementary_z);
    ui->Complementary_Visualisation->show();

    ui->lcdNumber_Complementary_Roll->display(Complementary_Roll);
    ui->lcdNumber_Complementary_Pitch->display(Complementary_Pitch);
    ui->lcdNumber_Complementary_Yaw->display(Complementary_Yaw);

    // Kalman filter
    QVector3D Kalman_euler = QQuaternion(-Data_from.Kalman_w, -Data_from.Kalman_y, -Data_from.Kalman_z, -Data_from.Kalman_x).toEulerAngles();
    double Kalman_Roll  = Kalman_euler.z();
    double Kalman_Pitch = Kalman_euler.x();
    double Kalman_Yaw   = Kalman_euler.y();

    if(Show_Accelerometer_X == true) ui->Kalman_Graph->graph(0)->addData(key, Kalman_Roll);
    if(Show_Accelerometer_Y == true) ui->Kalman_Graph->graph(1)->addData(key, Kalman_Pitch);
    if(Show_Accelerometer_Z == true) ui->Kalman_Graph->graph(2)->addData(key, Kalman_Yaw);

    ui->Kalman_Graph->graph(0)->rescaleValueAxis(true);
    ui->Kalman_Graph->graph(1)->rescaleValueAxis(true);
    ui->Kalman_Graph->graph(2)->rescaleValueAxis(true);

    ui->Kalman_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Kalman_Graph_Run == true ) {

        ui->Kalman_Graph->replot();

        ui->lcdNumber_Accelerometer_X->display(Kalman_Roll);
        ui->lcdNumber_Accelerometer_Y->display(Kalman_Pitch);
        ui->lcdNumber_Accelerometer_Z->display(Kalman_Yaw);
    }

    ui->Kalman_Visualisation->setQuaternion(Data_from.Kalman_w, Data_from.Kalman_x, Data_from.Kalman_y, Data_from.Kalman_z);
    ui->Kalman_Visualisation->show();

    ui->lcdNumber_Kalman_Roll->display(Kalman_Roll);
    ui->lcdNumber_Kalman_Pitch->display(Kalman_Pitch);
    ui->lcdNumber_Kalman_Yaw->display(Kalman_Yaw);

    // Madgwick filter
    QVector3D Madgwick_euler = QQuaternion(-Data_from.Madgwick_w, -Data_from.Madgwick_y, -Data_from.Madgwick_z, -Data_from.Madgwick_x).toEulerAngles();
    double Madgwick_Roll  = Madgwick_euler.z();
    double Madgwick_Pitch = Madgwick_euler.x();
    double Madgwick_Yaw   = Madgwick_euler.y();

    if(Show_Magnetometer_X == true) ui->Madgwick_Graph->graph(0)->addData(key, Madgwick_Roll);
    if(Show_Magnetometer_Y == true) ui->Madgwick_Graph->graph(1)->addData(key, Madgwick_Pitch);
    if(Show_Magnetometer_Z == true) ui->Madgwick_Graph->graph(2)->addData(key, Madgwick_Yaw);

    ui->Madgwick_Graph->graph(0)->rescaleValueAxis(true);
    ui->Madgwick_Graph->graph(1)->rescaleValueAxis(true);
    ui->Madgwick_Graph->graph(2)->rescaleValueAxis(true);

    ui->Madgwick_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Madgwick_Graph_Run == true ) {

        ui->Madgwick_Graph->replot();

        ui->lcdNumber_Magnetometer_X->display(Madgwick_Roll);
        ui->lcdNumber_Magnetometer_Y->display(Madgwick_Pitch);
        ui->lcdNumber_Magnetometer_Z->display(Madgwick_Yaw);
    }

    ui->Madgwick_Visualisation->setQuaternion(Data_from.Madgwick_w,Data_from.Madgwick_x,Data_from.Madgwick_y,Data_from.Madgwick_z);
    ui->Madgwick_Visualisation->show();

    ui->lcdNumber_Madgwick_Roll->display(Madgwick_Roll);
    ui->lcdNumber_Madgwick_Pitch->display(Madgwick_Pitch);
    ui->lcdNumber_Madgwick_Yaw->display(Madgwick_Yaw);

    // Print data frame
    QString data_line = QString::number(++data_iterator) + " " +
                        QTime::currentTime().toString() + " " +
                        QString::number(data_time.elapsed()) + " " +
                        QString::number(Complementary_Roll) + " " + QString::number(Complementary_Pitch) + " " + QString::number(Complementary_Yaw) + " " +
                        QString::number(Kalman_Roll) + " " + QString::number(Kalman_Pitch) + " " + QString::number(Kalman_Yaw) + " " +
                        QString::number(Madgwick_Roll) + " " + QString::number(Madgwick_Pitch) + " " + QString::number(Madgwick_Yaw) + " " + "\n";

    ui->textBrowser_Data->append(data_line);
    Data_lines.push_back(data_line);

    qDebug() << '\n';
    qDebug() << "Complementary: "<< "w: " << Data_from.Complementary_w << "x: " << Data_from.Complementary_x << "y: " << Data_from.Complementary_y << "z: " << Data_from.Complementary_z;
    qDebug() << "Kalman: " << "      w: " << Data_from.Kalman_w << "       x: " << Data_from.Kalman_x << "       y: " << Data_from.Kalman_y << "       z: " << Data_from.Kalman_z;
    qDebug() << "Madgwick: " << "    w: " << Data_from.Madgwick_w << "     x: " << Data_from.Madgwick_x << "     y: " << Data_from.Madgwick_y << "     z: " << Data_from.Madgwick_z;
    qDebug() << '\n';
    qDebug() << "Accelerometer_velocity: " << Data_from.Madgwick_w << Data_from.Madgwick_x << Data_from.Madgwick_y;
    qDebug() << "Accelerometer_position: " << Data_from.Kalman_w << Data_from.Kalman_x << Data_from.Kalman_y;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MainWindow::MainWindow_realtimeDataSlot(Data_from_Robot data)
{        
    Data_from = data;

    MainWindow_Display_IMU_data();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::Connection_OK_Slot()
{
    this->showMaximized();

    ui->label_PortName->setText( CW->Get_PortName() );
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::loadSettings()
{
    QSettings settings(m_sSettingsFile);

    double Complementary_weight = settings.value("Complementary_weight").toDouble();

    settings.setValue("Complementary_weight", Complementary_weight);

    /*
    qDebug() << "Wczytano PID_Kp: " << PID_Kp;
    qDebug() << "Wczytano PID_Ki: " << PID_Ki;
    qDebug() << "Wczytano PID_Kd: " << PID_Kd;

    qDebug() << "Wczytano Speed_PID_Kp: " << Speed_PID_Kp;
    qDebug() << "Wczytano Speed_PID_Ki: " << Speed_PID_Ki;
    qDebug() << "Wczytano Speed_PID_Kd: " << Speed_PID_Kd;

    qDebug() << "Wczytano Complementary_weight: " << Complementary_weight;
    qDebug() << "Wczytano wariancje: "            << Variance;
    qDebug() << "Wczytano beta: "                 << Madgwick_beta;
    qDebug() << "Wczytano Which_filter: "         << Which_filter;
    qDebug() << "Wczytano Set_Speed: "            << Set_speed;
    */
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::saveSettings()
{
    QSettings settings(m_sSettingsFile);

    /*
    qDebug() << "Zapisano PID_Kp: "                 << PID_Kp;
    qDebug() << "Zapisano PID_Ki: "                 << PID_Ki;
    qDebug() << "Zapisano PID_Kd: "                 << PID_Kd;
    qDebug() << "Zapisano Speed_PID_Kp: "           << Speed_PID_Kp;
    qDebug() << "Zapisano Speed_PID_Ki: "           << Speed_PID_Ki;
    qDebug() << "Zapisano Speed_PID_Kd: "           << Speed_PID_Kd;
    qDebug() << "Zapisano Complementary_weight: "   << Complementary_weight;
    qDebug() << "Zapisano Variance: "               << Kalman_filter_process_variance;
    qDebug() << "Zapisano beta: "                   << Madgwick_beta;
    qDebug() << "Zapisano Which_filter: "           << Which_filter;
    qDebug() << "Zapisano Set_Speed: "              << Set_speed;
    */
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Icons()
{
    int w = 0;
    int h = 0;

    QPixmap Red_dot(":/new/prefix1/png/Red_dot.png");
    QPixmap Green_dot(":/new/prefix1/png/Green_dot.png");
    QPixmap Blue_dot(":/new/prefix1/png/Blue_dot.png");
    QPixmap Battery_null(":/new/prefix1/png/Battery_null.png");
    QPixmap Connection(":/new/prefix1/png/Bluetooth.png");
    QPixmap Stop(":/new/prefix1/png/Stop_button.png");
    QPixmap RPY(":/new/prefix1/png/RPY.png");
    QPixmap StepperMotor(":/new/prefix1/png/StepperMotor.png");

    QPixmap RedArrowUp(":/new/prefix1/png/RedArrowUp.png");
    QPixmap RedArrowDown(":/new/prefix1/png/RedArrowDown.png");
    QPixmap RedArrowLeft(":/new/prefix1/png/RedArrowLeft.png");
    QPixmap RedArrowRight(":/new/prefix1/png/RedArrowRight.png");

    QPixmap Speed(":/new/prefix1/png/Speed.png");
    QPixmap Angle(":/new/prefix1/png/Angle.png");
    QPixmap Fusion(":/new/prefix1/png/Fusion.png");
    QPixmap Variables(":/new/prefix1/png/Variables.png");

    w = ui->label_Connection->width();
    h = ui->label_Connection->height();
    ui->label_Connection->setPixmap( Connection.scaled(w, h, Qt::KeepAspectRatio) );

    // Gyroscope RGB dots
    w = ui->label_Gyroscope_X->width();
    h = ui->label_Gyroscope_X->height();
    ui->label_Gyroscope_X->setPixmap( Red_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Gyroscope_Y->width();
    h = ui->label_Gyroscope_Y->height();
    ui->label_Gyroscope_Y->setPixmap( Green_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Gyroscope_Z->width();
    h = ui->label_Gyroscope_Z->height();
    ui->label_Gyroscope_Z->setPixmap( Blue_dot.scaled(w, h, Qt::KeepAspectRatio) );

    // Accelerometer RGB dots
    w = ui->label_Accelerometer_X->width();
    h = ui->label_Accelerometer_X->height();
    ui->label_Accelerometer_X->setPixmap( Red_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Accelerometer_Y->width();
    h = ui->label_Accelerometer_Y->height();
    ui->label_Accelerometer_Y->setPixmap( Green_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Accelerometer_Z->width();
    h = ui->label_Accelerometer_Z->height();
    ui->label_Accelerometer_Z->setPixmap( Blue_dot.scaled(w, h, Qt::KeepAspectRatio) );

    // Magnetometer RGB dots
    w = ui->label_Magnetometer_X->width();
    h = ui->label_Magnetometer_X->height();
    ui->label_Magnetometer_X->setPixmap( Red_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Magnetometer_Y->width();
    h = ui->label_Magnetometer_Y->height();
    ui->label_Magnetometer_Y->setPixmap( Green_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Magnetometer_Z->width();
    h = ui->label_Magnetometer_Z->height();
    ui->label_Magnetometer_Z->setPixmap( Blue_dot.scaled(w, h, Qt::KeepAspectRatio) );
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_ConnectDisconnect_clicked()
{
    this->hide();
    emit Disconnect_Signal();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Exit_clicked()
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

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_doubleSpinBox_Complementary_filter_weight_valueChanged(double arg1)
{
    arg1 = 0;
    saveSettings();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_radioButton_Complementary_filter_toggled(bool checked)
{
    if( checked == true ) {

        Data_to.Which_filter = 0;
    }

    saveSettings();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_radioButton_Kalman_filter_toggled(bool checked)
{
    if( checked == true ) {

        Data_to.Which_filter = 1;
    }

    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_radioButton_Madgwick_filter_toggled(bool checked)
{
    if( checked == true ) {

        Data_to.Which_filter = 2;
    }

    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_doubleSpinBox_Madgwick_beta_valueChanged(double arg1)
{
    Data_to.Madgwick_filter_beta = arg1;
    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Gyroscope_X_toggled(bool checked)
{
    if( checked ) {

        Show_Gyroscope_X = true;
    }
    else {

        Show_Gyroscope_X = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Gyroscope_Y_toggled(bool checked)
{
    if( checked ) {

        Show_Gyroscope_Y = true;
    }
    else {

        Show_Gyroscope_Y = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Gyroscope_Z_toggled(bool checked)
{
    if( checked ) {

        Show_Gyroscope_Z = true;
    }
    else {

        Show_Gyroscope_Z = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Accelerometer_X_toggled(bool checked)
{
    if( checked ) {

        Show_Accelerometer_X = true;
    }
    else {

        Show_Accelerometer_X = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Accelerometer_Y_toggled(bool checked)
{
    if( checked ) {

        Show_Accelerometer_Y = true;
    }
    else {

        Show_Accelerometer_Y = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Accelerometer_Z_toggled(bool checked)
{
    if( checked ) {

        Show_Accelerometer_Z = true;
    }
    else {

        Show_Accelerometer_Z = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Magnetometer_X_toggled(bool checked)
{
    if( checked ) {

        Show_Magnetometer_X = true;
    }
    else {

        Show_Magnetometer_X = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Magnetometer_Y_toggled(bool checked)
{
    if( checked ) {

        Show_Magnetometer_Y= true;
    }
    else {

        Show_Magnetometer_Y = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Magnetometer_Z_toggled(bool checked)
{
    if( checked ) {

        Show_Magnetometer_Z = true;
    }
    else {

        Show_Magnetometer_Z = false;
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_radioButton_Mahony_filter_toggled(bool checked)
{
    if( checked == true ) {

        Data_to.Which_filter = 3;
    }

    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_doubleSpinBox_Kalman_filter_process_variance_valueChanged(double arg1)
{
    Data_to.Kalman_procces_variance = static_cast<int>(arg1);
    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_doubleSpinBox_Kalman_filter_measure_variance_valueChanged(double arg1)
{
    Data_to.Kalman_measure_variance = static_cast<int>(arg1);
    saveSettings();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Data_Clear_clicked()
{
    ui->textBrowser_Data->clear();
    Data_lines.clear();
    data_iterator = 0;
    data_time.restart();
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Data_Save_clicked()
{
    std::fstream file("data.txt", std::ios::out | std::ios::trunc);

    if( file.good() ) {

        int size_list = Data_lines.length();

        for(int i = 0; i < size_list; i++) {

            qDebug() << Data_lines.at(i);
            file << Data_lines.at(i).toStdString();
            file.flush();
        }

        file.close();
    }
    else {

        qDebug() << "Nie mozna utworzyc pliku !";
    }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
