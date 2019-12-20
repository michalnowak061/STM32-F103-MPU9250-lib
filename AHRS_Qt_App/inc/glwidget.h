#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#include "geometryengine.h"

class GeometryEngine;

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

private:

    double m_xRot;
    double m_yRot;
    double m_zRot;

    double q_w;
    double q_x;
    double q_y;
    double q_z;

    QBasicTimer timer;
    QOpenGLShaderProgram program;
    GeometryEngine *geometries;

    QOpenGLTexture *texture;

    QMatrix4x4 projection;

    QVector2D mousePressPosition;
    QVector3D rotationAxis;
    qreal angularSpeed;
    QQuaternion rotation;

public:

    GLWidget(QWidget *parent = nullptr);
    ~GLWidget() Q_DECL_OVERRIDE;

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint()        const Q_DECL_OVERRIDE;

public slots:

    void setXRotation(double angle);
    void setYRotation(double angle);
    void setZRotation(double angle);

    void setQuaternion(double w, double x, double y, double z);

protected:

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void initShaders();
    void initTextures();
};

#endif
