#include "renderer.h"
#include <QtOpenGL>
#include <GL/glu.h>
#include "smallsat.h"
#include <iostream>
#include <QtMath>

using namespace scene;

Renderer* Renderer::si;

Renderer::Renderer(QWidget *parent) : QOpenGLWidget(parent)
{
    si = this;

    update();
}

Renderer::~Renderer()
{
    free(vbo);
    free(vinx);
}

void Renderer::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.1, 0.1, 0.1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    InitMesh();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(65.0, (float) this->width() / (float) this->height(), 0.15, 251.0);
    glViewport(0, 0, this->width(), this->height());

    for (int ID : meshIDs)
    {
        textures << new QOpenGLTexture(QImage("textures/" + textureNames[ID]).mirrored());
        textures.back()->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        textures.back()->setMagnificationFilter(QOpenGLTexture::Linear);
    }
}

void Renderer::paintGL()
{
    glClearColor(0.1, 0.1, 0.1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    theta += (thetaTarget - theta) * 0.1f;
    phi += (phiTarget - phi) * 0.1f;
    zoom += (zoomTarget - zoom) * 0.1f;

    xOffset = cos(phi) *cos(theta) * zoom;
    yOffset = cos(phi) *sin(theta) * zoom;
    zOffset = sin(phi) * zoom;


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(transformations[CUBESAT][12] + xOffset,
            transformations[CUBESAT][13] + yOffset,
            transformations[CUBESAT][14] + zOffset,
            transformations[CUBESAT][12], transformations[CUBESAT][13], transformations[CUBESAT][14],
            0, 0, 1);

    if (trajectoryPos.size() > 0)
    {
        int idx = (int(frameCounter) / 1) % trajectoryPos.size();

        QMatrix4x4 m;
        m.translate(trajectoryPos[idx]);
        m.rotate(trajectoryRot[idx]);
        m.scale(0.15f);

        for (int i=0; i<16; i++)
        {
            transformations[CUBESAT][i] = m.data()[i];
        }

        frameCounter++;

        satX += (trajectoryPos[idx].x() - satX) * 0.1;
        satY += (trajectoryPos[idx].y() - satY) * 0.1;
        satZ += (trajectoryPos[idx].z() - satZ) * 0.1;

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(satX + xOffset,
                  satY + yOffset,
                  satZ + zOffset,
                  satX, satY, satZ,
                  0, 0, 1);
    }

    glEnable(GL_TEXTURE_2D);
    DrawAllMeshes();

    if (MainWindow::si->drawTrajectories)
    {
        glPointSize(2);
        glEnable(GL_POINT_SMOOTH);
        glDisable(GL_TEXTURE_2D);
        glBegin(GL_POINTS);
        glColor3f(1,0,0);
        for (QVector3D pt : trajectoryPos)
        {
            //        if (a) std::cout << pt.x() << " " << pt.y() << " " << pt.z() << std::endl;
            glVertex3f(pt.x(), pt.y(), pt.z());
        }
        glColor3f(1,1,1);
        glEnd();
    }

    if (shouldUpdate) update();
}

void Renderer::resizeGL(int width, int height)
{

}

void Renderer::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() == Qt::LeftButton)
    {
        float dx = event->x() - lastX;
        float dy = event->y() - lastY;

        thetaTarget -= dx * 0.01f;
        phiTarget += dy * 0.01f;
        if (phiTarget < -M_PI_2 + EPS) phiTarget = -M_PI_2 + EPS;
        else if (phiTarget > M_PI_2 - EPS) phiTarget = M_PI_2 - EPS;

        //        qDebug() <<
        lastX = event->x();
        lastY = event->y();
    }

    if (event->buttons() == Qt::MiddleButton)
    {
        float dx = event->x() - lastX;
        float dy = event->y() - lastY;

        zoomTarget += dy * 0.1f;
        if (zoomTarget < 1) zoomTarget = 1;
        if (zoomTarget > 30) zoomTarget = 30;
    }

    lastX = event->x();
    lastY = event->y();
}

void Renderer::mousePressEvent(QMouseEvent *event)
{

}

void Renderer::mouseReleaseEvent(QMouseEvent *event)
{

}

// Adapted from: http://ksolek.fm.interiowo.pl/Blender/
void Renderer::InitMesh() {
    unsigned int i;

    vbo = (GLuint *)malloc(OBJECTS_COUNT * sizeof(GLuint));
    vinx = (GLuint *)malloc(OBJECTS_COUNT * sizeof(GLuint));

    glGenBuffers(OBJECTS_COUNT, vbo);

    for (i=0; i<OBJECTS_COUNT; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (struct vertex_struct) * vertex_count[i], &vertices[vertex_offset_table[i]], GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    glGenBuffers(OBJECTS_COUNT, vinx);
    for (i=0; i<OBJECTS_COUNT; i++) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vinx[i]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof (indexes[0]) * faces_count[i] * 3, &indexes[indices_offset_table[i]], GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }
}

#define BUFFER_OFFSET(x)((char *)NULL+(x))

void Renderer::DrawMesh(unsigned int index, int apply_transformations) {

    if (apply_transformations) {
        glPushMatrix();
        glMultMatrixf(transformations[index]);
    }

    glBindBuffer(GL_ARRAY_BUFFER, vbo[index]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vinx[index]);

    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof (struct vertex_struct), BUFFER_OFFSET(0));

    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, sizeof (struct vertex_struct), BUFFER_OFFSET(3 * sizeof (float)));

    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glTexCoordPointer(2, GL_FLOAT, sizeof (struct vertex_struct), BUFFER_OFFSET(6 * sizeof (float)));

    glDrawElements(GL_TRIANGLES, faces_count[index] * 3, INX_TYPE, BUFFER_OFFSET(0));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);

    if (apply_transformations) {
        glPopMatrix();
    }
}

void Renderer::DrawAllMeshes()
{
    unsigned int i;

    for (i=0; i<OBJECTS_COUNT; i++) {
        textures[i]->bind();
        //        if (i==1) continue;
        DrawMesh(i, 1);
    }
}

