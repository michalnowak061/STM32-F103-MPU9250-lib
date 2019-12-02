#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QEvent>
#include <QPixmap>
#include <QKeyEvent>
#include <QSettings>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <QTimer>

#include "qcustomplot.h"
#include "bluetooth.h"
#include "glwidget.h"
#include "communicationwindow.h"

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

namespace Ui {
class MainWindow;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

public slots:

    void MainWindow_realtimeDataSlot(Data_from_Robot data);
    void Connection_OK_Slot();

    void on_pushButton_ConnectDisconnect_clicked();

private:

    Ui::MainWindow *ui;

    GLWidget            *GW = new GLWidget;
    CommunicationWindow *CW = new CommunicationWindow;

    QString m_sSettingsFile;

    Data_to_Robot Data_to;
    Data_from_Robot Data_from;

    bool Show_Accelerometer_X, Show_Accelerometer_Y, Show_Accelerometer_Z;
    bool Show_Accelerometer_Roll, Show_Accelerometer_Pitch, Show_Accelerometer_Yaw;

    bool Show_Gyroscope_Roll, Show_Gyroscope_Pitch, Show_Gyroscope_Yaw;

    bool Show_Magnetometer_X, Show_Magnetometer_Y, Show_Magnetometer_Z;
    bool Show_Magnetometer_Roll, Show_Magnetometer_Pitch, Show_Magnetometer_Yaw;

    bool Show_Filter_Roll, Show_Filter_Pitch, Show_Filter_Yaw;

    bool Show_Gyroscope_X, Show_Gyroscope_Y, Show_Gyroscope_Z;

    bool Complementary_Graph_Run;
    bool Kalman_Graph_Run;
    bool Madgwick_Graph_Run;

    int Set_Speed;

    void loadSettings();
    void saveSettings();

    //void MainWindow_Default_View();
    void MainWindow_Setup_Icons();

    void MainWindow_Setup_Complementary_Graph();
    void MainWindow_Setup_Kalman_Graph();
    void MainWindow_Setup_Madgwick_Graph();
    //void MainWindow_Setup_Filter_Graph();

    void MainWindow_Display_IMU_data();

    void closeEvent(QCloseEvent *event) override;

    //QList<int> Splitter_Position;
    QList<QString> Data_lines;
    int data_iterator = 1;

private slots:

    //void on_checkBox_Filter_Roll_clicked();
    //void on_checkBox_Filter_Pitch_clicked();
    //void on_checkBox_Filter_Yaw_clicked();

    //void on_pushButton_Send_clicked();
    void on_pushButton_Exit_clicked();

    void on_doubleSpinBox_Complementary_filter_weight_valueChanged(double arg1);

    //void on_pushButton_Plots_Center_clicked();
    //void on_pushButton_Plots_Start_Stop_clicked();
    //void on_pushButton_Reset_Plots_Range_clicked();

    void on_radioButton_Complementary_filter_toggled(bool checked);
    void on_radioButton_Kalman_filter_toggled(bool checked);
    void on_radioButton_Madgwick_filter_toggled(bool checked);

    void on_doubleSpinBox_Madgwick_beta_valueChanged(double arg1);

    void on_checkBox_Gyroscope_X_toggled(bool checked);
    void on_checkBox_Gyroscope_Y_toggled(bool checked);
    void on_checkBox_Gyroscope_Z_toggled(bool checked);

    void on_checkBox_Accelerometer_X_toggled(bool checked);
    void on_checkBox_Accelerometer_Y_toggled(bool checked);
    void on_checkBox_Accelerometer_Z_toggled(bool checked);

    void on_checkBox_Magnetometer_X_toggled(bool checked);
    void on_checkBox_Magnetometer_Y_toggled(bool checked);
    void on_checkBox_Magnetometer_Z_toggled(bool checked);

    void on_radioButton_Mahony_filter_toggled(bool checked);

    void on_doubleSpinBox_Kalman_filter_process_variance_valueChanged(double arg1);
    void on_doubleSpinBox_Kalman_filter_measure_variance_valueChanged(double arg1);

    void on_pushButton_Data_Clear_clicked();
    void on_pushButton_Data_Save_clicked();

signals:

    void Disconnect_Signal();
    void Send_data_Signal();
};

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#endif // MAINWINDOW_H
