#include "glwidget.h"

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

GLWidget::~GLWidget()
{
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

QSize GLWidget::minimumSizeHint() const
{
    return QSize(200, 200);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

QSize GLWidget::sizeHint() const
{
    return QSize(200, 200);
}


// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::setQuaternion(double w, double x, double y, double z) {

    q_w = w;
    q_x = x;
    q_y = y;
    q_z = z;

    update();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLCreateCube(float x, float y, float z) {

    glPushMatrix();
    glScalef(x,y,z);

    // Yellow side
    glBegin(GL_POLYGON);
    glColor3f(1.0, 1.0, 0.0);
    glVertex3f(  0.5, -0.5, -0.5);
    glVertex3f(  0.5, 0.5, -0.5);
    glVertex3f( -0.5,  0.5, -0.5);
    glVertex3f( -0.5, -0.5, -0.5);
    glEnd();

    // White side
    glBegin(GL_POLYGON);
    glColor3f(1.0, 0.0, 1.0);
    glVertex3f(  0.5, -0.5,  0.5);
    glVertex3f(  0.5,  0.5,  0.5);
    glVertex3f( -0.5,  0.5,  0.5);
    glVertex3f( -0.5, -0.5,  0.5);
    glEnd();

    // Purple side
    glBegin(GL_POLYGON);
    glColor3f(1.0, 1.0, 1.0);
    glVertex3f(  0.5, -0.5, -0.5);
    glVertex3f(  0.5,  0.5, -0.5);
    glVertex3f(  0.5,  0.5,  0.5);
    glVertex3f(  0.5, -0.5,  0.5);
    glEnd();

    // Green side
    glBegin(GL_POLYGON);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f( -0.5, -0.5,  0.5);
    glVertex3f( -0.5,  0.5,  0.5);
    glVertex3f( -0.5,  0.5, -0.5);
    glVertex3f( -0.5, -0.5, -0.5);
    glEnd();

    // Blue side
    glBegin(GL_POLYGON);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(  0.5,  0.5,  0.5);
    glVertex3f(  0.5,  0.5, -0.5);
    glVertex3f( -0.5,  0.5, -0.5);
    glVertex3f( -0.5,  0.5,  0.5);
    glEnd();

    // Red side
    glBegin(GL_POLYGON);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(  0.5, -0.5, -0.5);
    glVertex3f(  0.5, -0.5,  0.5);
    glVertex3f( -0.5, -0.5,  0.5);
    glVertex3f( -0.5, -0.5, -0.5);
    glEnd();

    glPopMatrix();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::initializeGL() {

    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();
    initTextures();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

    geometries = new GeometryEngine;
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::initTextures()
{
    // Load cube.png image
    texture = new QOpenGLTexture(QImage(":/new/prefix1/png/cube.png").mirrored());

    // Set nearest filtering mode for texture minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);

    // Set bilinear filtering mode for texture magnification
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    texture->setWrapMode(QOpenGLTexture::Repeat);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::paintGL() {

    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    texture->bind();

    // Calculate model view transformation
    //rotationAxis = QVector3D(0,0,0);
    rotation = QQuaternion(q_w, -q_x, q_z, q_y);
    //update();

    QMatrix4x4 matrix;
    matrix.translate(0.0, 0.0, -5.0);
    matrix.rotate(rotation);

    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);

    // Use texture unit 0 which contains cube.png
    program.setUniformValue("texture", 0);

    // Draw cube geometry
    geometries->drawCubeGeometry(&program);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::resizeGL(int w, int h) {

    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 3.0, zFar = 7.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}

// -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
