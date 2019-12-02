#-------------------------------------------------
#
# Project created by QtCreator 2019-06-25T23:55:51
#
#-------------------------------------------------

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = AHRS_app
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

INCLUDEPATH += inc\

SOURCES += \
        src/bluetooth.cpp \
        src/communicationwindow.cpp \
        src/glwidget.cpp \
        src/main.cpp \
        src/mainwindow.cpp \
        src/qcustomplot.cpp

HEADERS += \
        inc/bluetooth.h \
        inc/communicationwindow.h \
        inc/glwidget.h \
        inc/mainwindow.h \
        inc/qcustomplot.h

FORMS += \
        ui/communicationwindow.ui \
        ui/mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    zasoby.qrc \
    zasoby.qrc

DISTFILES += \
    png/Angle.png \
    png/Battery_full.png \
    png/Battery_low.png \
    png/Battery_medium.png \
    png/Battery_null.png \
    png/Blue_dot.png \
    png/Bluetooth.png \
    png/Fusion.png \
    png/GreenArrowDown.png \
    png/GreenArrowLeft.png \
    png/GreenArrowRight.png \
    png/GreenArrowUp.png \
    png/Green_dot.png \
    png/RedArrowDown.png \
    png/RedArrowLeft.png \
    png/RedArrowRight.png \
    png/RedArrowUp.png \
    png/Red_dot.png \
    png/Speed.png \
    png/Stop_button.png \
    png/Variables.png
