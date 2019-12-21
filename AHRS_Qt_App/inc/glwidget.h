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

    double q_w;
    double q_x;
    double q_y;
    double q_z;

    QOpenGLShaderProgram program;
    GeometryEngine *geometries;
    QOpenGLTexture *texture;
    QMatrix4x4 projection;
    QVector3D rotationAxis;
    QQuaternion rotation;

public:

    GLWidget(QWidget *parent = nullptr);
    ~GLWidget() Q_DECL_OVERRIDE;

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint()        const Q_DECL_OVERRIDE;

public slots:

    void setQuaternion(double w, double x, double y, double z);

protected:

    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void initShaders();
    void initTextures();
};

#endif
