#ifndef RENDERER_H
#define RENDERER_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLTexture>


class Renderer : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    Renderer(QWidget* parent = 0);
    ~Renderer();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

private:
    void DrawAllMeshes();
    void DrawMesh(unsigned int index, int apply_transformations);
    void InitMesh();

    GLuint *vbo;
    GLuint *vinx;

    QVector<QOpenGLTexture*> textures;
};

#endif // RENDERER_H
