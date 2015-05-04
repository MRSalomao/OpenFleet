#ifndef RENDERER_H
#define RENDERER_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include "mainwindow.h"

#define EPS 0.01f

class Renderer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    Renderer(QWidget* parent = 0);
    ~Renderer();

    QVector<QVector3D> trajectoryPos;
    QVector<QQuaternion> trajectoryRot;

    static Renderer* si;
    int frameCounter = 0;

    bool shouldUpdate = true;

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mouseMoveEvent(QMouseEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);


private:
    void DrawAllMeshes();
    void DrawMesh(unsigned int index, int apply_transformations);
    void InitMesh();

    GLuint *vbo;
    GLuint *vinx;

    float zoomTarget = 4;
    float thetaTarget = 0;
    float phiTarget = 0;

    float theta = 0;
    float phi = 0;
    float zoom = 4;

    float xOffset = 10;
    float yOffset = 10;
    float zOffset = 10;

    float satX = 0;
    float satY = 0;
    float satZ = 0;

    int lastX = 0;
    int lastY = 0;

    QVector<QOpenGLTexture*> textures;
};

#endif // RENDERER_H
