#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <GLUT/glut.h>

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

private:

    double m_xRot;
    double m_yRot;
    double m_zRot;

public:

    GLWidget(QWidget *parent = nullptr);
    ~GLWidget() Q_DECL_OVERRIDE;

    QSize minimumSizeHint() const Q_DECL_OVERRIDE;
    QSize sizeHint()        const Q_DECL_OVERRIDE;

public slots:

    void setXRotation(double angle);
    void setYRotation(double angle);
    void setZRotation(double angle);

protected:

    void initializeGL()                         Q_DECL_OVERRIDE;
    void paintGL()                              Q_DECL_OVERRIDE;
    void resizeGL(int width, int height)        Q_DECL_OVERRIDE;
};

#endif
