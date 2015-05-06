#ifndef RENDERER_H
#define RENDERER_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>
#include "mainwindow.h"

// camera system uses Euler angles - this is to avoid singularities
#define EPS 0.01f

class Renderer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    Renderer(QWidget* parent = 0);
    ~Renderer();

    static Renderer* si;

    QVector<QVector<QVector3D>> trajectoryPos;
    QVector<QVector<QQuaternion>> trajectoryRot;

    QVector<QVector3D> asteroidPos;
    QVector<QQuaternion> asteroidRot;
    QVector<QVector3D> asteroidVel;
    QVector<QQuaternion> asteroidAngVel;

    int frameCounter = 0;
    bool isPaused = false;

    bool shouldUpdate = true;

    QVector<int> objectsToRender;

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
    GLuint *ibo;

    float slowDown = 0.1f; // slow down the original speed, for easier visualization
    float originalFps = 0.1;

    float zoomTarget = 4;
    float thetaTarget = 0.7;
    float phiTarget = 0.4;

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
