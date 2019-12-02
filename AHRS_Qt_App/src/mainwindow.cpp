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

    Gyroscope_Graph_Run = true;
    Accelerometer_Graph_Run = true;
    Magnetometer_Graph_Run = true;
    Filter_Graph_Run = true;

    Data_to.Complementary_filter_weight = 0;

    m_sSettingsFile = QApplication::applicationDirPath().left(1) + ":/main_settings.ini";
    loadSettings();

    MainWindow_Default_View();
    MainWindow_Setup_Icons();

    // Setup real time graphs
    MainWindow_Setup_Gyroscope_Graph();
    MainWindow_Setup_Accelerometer_Graph();
    MainWindow_Setup_Magnetometer_Graph();
    MainWindow_Setup_Filter_Graph();

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

    Splitter_Position = ui->splitter->sizes();
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
double MainWindow::convert(double angle)
{
    return fmodf( angle + 360, 360 );
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Filter_Graph()
{
    ui->Filter_Graph->addGraph(); // red line
    ui->Filter_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Filter_Graph->addGraph(); // green line
    ui->Filter_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Filter_Graph->addGraph(); // blue line
    ui->Filter_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Filter_Graph->xAxis->setTicker(timeTicker);
    ui->Filter_Graph->axisRect()->setupFullAxesBox();
    ui->Filter_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Filter_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Filter_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Filter_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Filter_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Gyroscope_Graph()
{
    ui->Gyroscope_Graph->addGraph(); // red line
    ui->Gyroscope_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Gyroscope_Graph->addGraph(); // green line
    ui->Gyroscope_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Gyroscope_Graph->addGraph(); // blue line
    ui->Gyroscope_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Gyroscope_Graph->xAxis->setTicker(timeTicker);
    ui->Gyroscope_Graph->axisRect()->setupFullAxesBox();
    ui->Gyroscope_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Gyroscope_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Gyroscope_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Gyroscope_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Gyroscope_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Accelerometer_Graph()
{
    ui->Accelerometer_Graph->addGraph(); // red line
    ui->Accelerometer_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Accelerometer_Graph->addGraph(); // green line
    ui->Accelerometer_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Accelerometer_Graph->addGraph(); // blue line
    ui->Accelerometer_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Accelerometer_Graph->xAxis->setTicker(timeTicker);
    ui->Accelerometer_Graph->axisRect()->setupFullAxesBox();
    ui->Accelerometer_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Accelerometer_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Accelerometer_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Accelerometer_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Accelerometer_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Setup_Magnetometer_Graph()
{
    ui->Magnetometer_Graph->addGraph(); // red line
    ui->Magnetometer_Graph->graph(0)->setPen(QPen(QColor(255, 0, 0)));
    ui->Magnetometer_Graph->addGraph(); // green line
    ui->Magnetometer_Graph->graph(1)->setPen(QPen(QColor(0, 255, 0)));
    ui->Magnetometer_Graph->addGraph(); // blue line
    ui->Magnetometer_Graph->graph(2)->setPen(QPen(QColor(0, 0, 255)));

    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");

    ui->Magnetometer_Graph->xAxis->setTicker(timeTicker);
    ui->Magnetometer_Graph->axisRect()->setupFullAxesBox();
    ui->Magnetometer_Graph->yAxis->setRange(-1, 1);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(ui->Magnetometer_Graph->xAxis, SIGNAL(rangeChanged(QCPRange)), ui->Magnetometer_Graph->xAxis2, SLOT(setRange(QCPRange)));
    connect(ui->Magnetometer_Graph->yAxis, SIGNAL(rangeChanged(QCPRange)), ui->Magnetometer_Graph->yAxis2, SLOT(setRange(QCPRange)));
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::MainWindow_Display_IMU_data()
{
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed() / 1000.0; // time elapsed since start of demo, in seconds

    // Fusion data
    //double Filter_Roll  = convert(Data_from.Filter_roll);
    //double Filter_Pitch = convert(Data_from.Filter_pitch);
    //double Filter_Yaw   = convert(Data_from.Filter_yaw);

    double Filter_Roll  = Data_from.Filter_roll;
    double Filter_Pitch = Data_from.Filter_pitch;
    double Filter_Yaw   = Data_from.Filter_yaw;

    double Gyroscope_X = Data_from.g_x_dgs;
    double Gyroscope_Y = Data_from.g_y_dgs;
    double Gyroscope_Z = Data_from.g_z_dgs;

    double Accelerometer_X = Data_from.a_x_g;
    double Accelerometer_Y = Data_from.a_y_g;
    double Accelerometer_Z = Data_from.a_z_g;

    double Magnetometer_X = Data_from.m_x_uT;
    double Magnetometer_Y = Data_from.m_y_uT;
    double Magnetometer_Z = Data_from.m_z_uT;

    // add data to lines:
    if(Show_Gyroscope_X == true) ui->Gyroscope_Graph->graph(0)->addData(key, Gyroscope_X);
    if(Show_Gyroscope_Y == true) ui->Gyroscope_Graph->graph(1)->addData(key, Gyroscope_Y);
    if(Show_Gyroscope_Z == true) ui->Gyroscope_Graph->graph(2)->addData(key, Gyroscope_Z);

    if(Show_Accelerometer_X == true) ui->Accelerometer_Graph->graph(0)->addData(key, Accelerometer_X);
    if(Show_Accelerometer_Y == true) ui->Accelerometer_Graph->graph(1)->addData(key, Accelerometer_Y);
    if(Show_Accelerometer_Z == true) ui->Accelerometer_Graph->graph(2)->addData(key, Accelerometer_Z);

    if(Show_Magnetometer_X == true) ui->Magnetometer_Graph->graph(0)->addData(key, Magnetometer_X);
    if(Show_Magnetometer_Y == true) ui->Magnetometer_Graph->graph(1)->addData(key, Magnetometer_Y);
    if(Show_Magnetometer_Z == true) ui->Magnetometer_Graph->graph(2)->addData(key, Magnetometer_Z);

    if(Show_Filter_Roll  == true) ui->Filter_Graph->graph(0)->addData(key, Filter_Roll);
    if(Show_Filter_Pitch == true) ui->Filter_Graph->graph(1)->addData(key, Filter_Pitch);
    if(Show_Filter_Yaw   == true) ui->Filter_Graph->graph(2)->addData(key, Filter_Yaw);

    ui->Gyroscope_Graph->graph(0)->rescaleValueAxis(true);
    ui->Gyroscope_Graph->graph(1)->rescaleValueAxis(true);
    ui->Gyroscope_Graph->graph(2)->rescaleValueAxis(true);

    ui->Accelerometer_Graph->graph(0)->rescaleValueAxis(true);
    ui->Accelerometer_Graph->graph(1)->rescaleValueAxis(true);
    ui->Accelerometer_Graph->graph(2)->rescaleValueAxis(true);

    ui->Magnetometer_Graph->graph(0)->rescaleValueAxis(true);
    ui->Magnetometer_Graph->graph(1)->rescaleValueAxis(true);
    ui->Magnetometer_Graph->graph(2)->rescaleValueAxis(true);

    ui->Filter_Graph->graph(0)->rescaleValueAxis(true);
    ui->Filter_Graph->graph(1)->rescaleValueAxis(true);
    ui->Filter_Graph->graph(2)->rescaleValueAxis(true);

    // make key axis range scroll with the data (at a constant range size of 8):
    ui->Gyroscope_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Gyroscope_Graph_Run == true ) {

        ui->Gyroscope_Graph->replot();

        ui->lcdNumber_Gyroscope_X->display(Gyroscope_X);
        ui->lcdNumber_Gyroscope_Y->display(Gyroscope_Y);
        ui->lcdNumber_Gyroscope_Z->display(Gyroscope_Z);
    }

    ui->Filter_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    ui->Accelerometer_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Accelerometer_Graph_Run == true ) {

        ui->Accelerometer_Graph->replot();

        ui->lcdNumber_Accelerometer_X->display(Accelerometer_X);
        ui->lcdNumber_Accelerometer_Y->display(Accelerometer_Y);
        ui->lcdNumber_Accelerometer_Z->display(Accelerometer_Z);
    }

    ui->Accelerometer_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    ui->Magnetometer_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    if( Magnetometer_Graph_Run == true ) {

        ui->Magnetometer_Graph->replot();

        ui->lcdNumber_Magnetometer_X->display(Magnetometer_X);
        ui->lcdNumber_Magnetometer_Y->display(Magnetometer_Y);
        ui->lcdNumber_Magnetometer_Z->display(Magnetometer_Z);
    }

    ui->Filter_Graph->xAxis->setRange(key, 30, Qt::AlignRight);

    // OpenGL visualisation and plot
    if(Filter_Graph_Run == true) {

        if(ui->checkBox_Filter_Roll->isChecked()) {

            ui->lcdNumber_Filter_Roll->display(Filter_Roll);
            ui->widget_RPY_Visualisation->setZRotation(Filter_Roll);

        } else{
            ui->lcdNumber_Filter_Roll->display(0);
            ui->widget_RPY_Visualisation->setZRotation(0);
        }

        if(ui->checkBox_Filter_Pitch->isChecked()) {

            ui->widget_RPY_Visualisation->setXRotation(Filter_Pitch);
            ui->lcdNumber_Filter_Pitch->display(Filter_Pitch);

        } else {

            ui->lcdNumber_Filter_Pitch->display(0);
            ui->widget_RPY_Visualisation->setXRotation(0);
        }

        if(ui->checkBox_Filter_Yaw->isChecked()) {

            ui->widget_RPY_Visualisation->setYRotation(Filter_Yaw);
            ui->lcdNumber_Filter_Yaw->display(Filter_Yaw);

        } else {

            ui->lcdNumber_Filter_Yaw->display(0);
            ui->widget_RPY_Visualisation->setYRotation(0);
        }

        ui->Filter_Graph->replot();
    }
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
    int Process_variance = settings.value("ProcessVariance").toInt();
    int Measure_variance = settings.value("MeasureVariance").toInt();
    double Madgwick_beta = settings.value("Madgwick_beta").toDouble();

    int Which_filter = settings.value("Which_filter").toInt();

    settings.setValue("Complementary_weight", Complementary_weight);

    ui->doubleSpinBox_Complementary_filter_weight->setValue(Complementary_weight);
    ui->doubleSpinBox_Kalman_filter_process_variance->setValue(Process_variance);
    ui->doubleSpinBox_Kalman_filter_measure_variance->setValue(Measure_variance);
    ui->doubleSpinBox_Madgwick_beta->setValue(Madgwick_beta);

    if( Which_filter == 0 )      ui->radioButton_Complementary_filter->setChecked(true);
    else if( Which_filter == 1 ) ui->radioButton_Kalman_filter->setChecked(true);
    else if( Which_filter == 2 ) ui->radioButton_Madgwick_filter->setChecked(true);
    else if( Which_filter == 3 ) ui->radioButton_Mahony_filter->setChecked(true);

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

    double Complementary_weight = ui->doubleSpinBox_Complementary_filter_weight->value();
    int Kalman_filter_process_variance = static_cast<int>( ui->doubleSpinBox_Kalman_filter_process_variance->value() );
    int Kalman_filter_measure_variance = static_cast<int>( ui->doubleSpinBox_Kalman_filter_measure_variance->value() );
    double Madgwick_beta = ui->doubleSpinBox_Madgwick_beta->value();

    int Which_filter = 0;

    if( ui->radioButton_Complementary_filter->isChecked() ) Which_filter = 0;
    else if(ui->radioButton_Kalman_filter->isChecked() )    Which_filter = 1;
    else if(ui->radioButton_Madgwick_filter->isChecked() )  Which_filter = 2;
    else if(ui->radioButton_Mahony_filter->isChecked() )    Which_filter = 3;

    settings.setValue("Complementary_weight", Complementary_weight);
    settings.setValue("ProcessVariance", Kalman_filter_process_variance);
    settings.setValue("MeasureVariance", Kalman_filter_measure_variance);
    settings.setValue("Madgwick_beta", Madgwick_beta);

    settings.setValue("Which_filter", Which_filter);

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

void MainWindow::MainWindow_Default_View()
{
   if(Show_Gyroscope_X == true) ui->checkBox_Gyroscope_X->setChecked(true);
   if(Show_Gyroscope_Y == true) ui->checkBox_Gyroscope_Y->setChecked(true);
   if(Show_Gyroscope_Z == true) ui->checkBox_Gyroscope_Z->setChecked(true);

   if(Show_Accelerometer_X == true) ui->checkBox_Accelerometer_X->setChecked(true);
   if(Show_Accelerometer_Y == true) ui->checkBox_Accelerometer_Y->setChecked(true);
   if(Show_Accelerometer_Z == true) ui->checkBox_Accelerometer_Z->setChecked(true);

   if(Show_Magnetometer_X == true) ui->checkBox_Magnetometer_X->setChecked(true);
   if(Show_Magnetometer_Y == true) ui->checkBox_Magnetometer_Y->setChecked(true);
   if(Show_Magnetometer_Z == true) ui->checkBox_Magnetometer_Z->setChecked(true);

   if(Show_Filter_Roll  == true) ui->checkBox_Filter_Roll->setChecked(true);
   if(Show_Filter_Pitch == true) ui->checkBox_Filter_Pitch->setChecked(true);
   if(Show_Filter_Yaw   == true) ui->checkBox_Filter_Yaw->setChecked(true);

   ui->pushButton_Plots_Start_Stop->setText("Stop");
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

    // Filter RGB dots
    w = ui->label_Filter_Roll->width();
    h = ui->label_Filter_Roll->height();
    ui->label_Filter_Roll->setPixmap( Red_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Filter_Pitch->width();
    h = ui->label_Filter_Pitch->height();
    ui->label_Filter_Pitch->setPixmap( Green_dot.scaled(w, h, Qt::KeepAspectRatio) );

    w = ui->label_Filter_Yaw->width();
    h = ui->label_Filter_Yaw->height();
    ui->label_Filter_Yaw->setPixmap( Blue_dot.scaled(w, h, Qt::KeepAspectRatio) );
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Filter_Roll_clicked()
{
    if( ui->checkBox_Filter_Roll->isChecked() ) {

        Show_Filter_Roll = true;
    }
    else {

        Show_Filter_Roll = false;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Filter_Pitch_clicked()
{
    if( ui->checkBox_Filter_Pitch->isChecked() ) {

        Show_Filter_Pitch = true;
    }
    else {

        Show_Filter_Pitch = false;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_checkBox_Filter_Yaw_clicked()
{
    if( ui->checkBox_Filter_Yaw->isChecked() ) {

        Show_Filter_Yaw = true;
    }
    else {

        Show_Filter_Yaw = false;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_ConnectDisconnect_clicked()
{
    this->hide();
    emit Disconnect_Signal();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Send_clicked()
{
    // Filters data
    Data_to.Complementary_filter_weight = ui->doubleSpinBox_Complementary_filter_weight->value();
    Data_to.Kalman_procces_variance = ui->doubleSpinBox_Kalman_filter_process_variance->value();
    Data_to.Kalman_measure_variance = ui->doubleSpinBox_Kalman_filter_measure_variance->value();
    Data_to.Madgwick_filter_beta = ui->doubleSpinBox_Madgwick_beta->value();

    CW->Fill_Data_to_robot(Data_to);

    emit Send_data_Signal();
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

void MainWindow::on_pushButton_Plots_Center_clicked()
{
    ui->splitter->setSizes( Splitter_Position );
    ui->splitter->update();
    ui->splitter->showMaximized();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Plots_Start_Stop_clicked()
{

    if( ui->pushButton_Plots_Start_Stop->text() == "Start" ) {

        ui->pushButton_Plots_Start_Stop->setText("Stop");
    }
    else {

        ui->pushButton_Plots_Start_Stop->setText("Start");
    }

    if(Filter_Graph_Run == true) {

        Filter_Graph_Run = false;
    }
    else {

        Filter_Graph_Run = true;
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void MainWindow::on_pushButton_Reset_Plots_Range_clicked()
{
    ui->Filter_Graph->clearGraphs();

    MainWindow_Setup_Filter_Graph();
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
