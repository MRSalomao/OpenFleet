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

    // initially display just the skybox, ISS and a cubesat
    objectsToRender << SKYBOX << ISS << CUBESAT;
}

Renderer::~Renderer()
{
    free(vbo);
    free(ibo);
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
    glEnable(GL_TEXTURE_2D);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(65.0, (float) this->width() / (float) this->height(), 0.25, 451.0);
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
    // clear the screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // calculate new camera position and angle
    theta += (thetaTarget - theta) * 0.01f;
    phi += (phiTarget - phi) * 0.01f;
    zoom += (zoomTarget - zoom) * 0.01f;

    xOffset = cos(phi) *cos(theta) * zoom;
    yOffset = cos(phi) *sin(theta) * zoom;
    zOffset = sin(phi) * zoom;

    // apply it to the view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(satX + xOffset,
              satY + yOffset,
              satZ + zOffset,
              satX, satY, satZ,
              0, 0, 1);

    // correctly position our smallsats and render them
    if (trajectoryPos.size() > 0)
    {
        // bind its texture
        textures[CUBESAT]->bind();

        // reset framecounter if it reaches the end of the simulation
        if (int(floor(frameCounter * slowDown)) == trajectoryPos[0].size()-3) frameCounter = 0;

        // accumulate centers of mass of every sat and tell the camera to focus on that
        // (so we can see all of them at the same time)
        float posAccX = 0;
        float posAccY = 0;
        float posAccZ = 0;

        for (int i=0; i<trajectoryPos.size(); i++)
        {
            // perform postion and rotation interpolation
            float idx = frameCounter * slowDown;
            int cidx = int(ceil(frameCounter * slowDown));
            int fidx = int(floor(frameCounter * slowDown));

            float k1 = idx - fidx;
            float k2 = 1.0f - k1;

            QVector3D cpos = trajectoryPos[i][cidx];
            QVector3D fpos = trajectoryPos[i][fidx];

            QQuaternion crot = trajectoryRot[i][cidx];
            QQuaternion frot = trajectoryRot[i][fidx];

            fpos *= k2; crot *= k1;
            cpos *= k1; frot *= k2;

            fpos += cpos; frot += crot; frot.normalize();

            // update the model matrix
            QMatrix4x4 m;
            m.translate(fpos);
            m.rotate(frot);
            for (int j=0; j<16; j++)
            {
                transformations[CUBESAT][j] = m.data()[j];
            }

            // update the frame counter
            if (!isPaused) frameCounter++;

            // We want the average position, so divide by the number of small sats in the fleet
            posAccX += fpos.x() / float(trajectoryPos.size());
            posAccY += fpos.y() / float(trajectoryPos.size());
            posAccZ += fpos.z() / float(trajectoryPos.size());

            // draw them
            for (int i=0; i<trajectoryPos.size(); i++)
            {
                DrawMesh(CUBESAT, 1);
            }
        }

        // update the camera's focus point
        satX += (posAccX - satX) * 0.1;
        satY += (posAccY - satY) * 0.1;
        satZ += (posAccZ - satZ) * 0.1;
    }

    // update each asteroid position and draw them (if they are present in this simulation)
    if (asteroidPos.size() > 0)
    {
        textures[ASTEROID1]->bind();

        if (int(floor(frameCounter * slowDown)) == trajectoryPos.size()-3) frameCounter = 0;

        for (int i=0; i<asteroidPos.size(); i++)
        {
            float idx = frameCounter * slowDown * originalFps;

            QVector3D pos = asteroidPos[i] + idx * asteroidVel[i];

            pos.setX( pos.x() - (int(pos.x()) / 10) * 10 );
            pos.setY( pos.y() - (int(pos.y()) / 10) * 10 );
            pos.setZ( pos.z() - (int(pos.z()) / 10) * 10 );

            asteroidRot[i] *= QQuaternion::fromAxisAndAngle(.02, .5, 1.2, .9);

            QMatrix4x4 m;
            m.translate(pos);
            m.rotate(asteroidRot[i]);
            m.scale(0.25f);

            for (int j=0; j<16; j++)
            {
                transformations[asteroids[i]][j] = m.data()[j];
            }

            DrawMesh(asteroids[i], 1);
        }
    }

    // draw the trajectories of each smallsat
    if (MainWindow::si->drawTrajectories && trajectoryPos.size() > 0)
    {
        glDisable(GL_TEXTURE_2D);
        for (int i=0; i<trajectoryPos.size(); i++)
        {
            glPointSize(2);
            glEnable(GL_POINT_SMOOTH);
            glBegin(GL_POINTS);
            glColor3f(1,0,0);
            for (QVector3D pt : trajectoryPos[i])
            {
                glVertex3f(pt.x(), pt.y(), pt.z());
            }
            glColor3f(1,1,1);
            glEnd();
        }
        glEnable(GL_TEXTURE_2D);
    }


    // draw the remaining objects in the scene (skybox, ISS, SolarSail, etc)
    DrawAllMeshes();

    if (shouldUpdate) update();
}

void Renderer::resizeGL(int width, int height)
{

}

// use mouse input to control the camera
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

        lastX = event->x();
        lastY = event->y();
    }

    if (event->buttons() == Qt::MiddleButton)
    {
        float dx = event->x() - lastX;
        float dy = event->y() - lastY;

        zoomTarget += dy * 0.1f;
        if (zoomTarget < 1) zoomTarget = 1;
        if (zoomTarget > 60) zoomTarget = 60;
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
    ibo = (GLuint *)malloc(OBJECTS_COUNT * sizeof(GLuint));

    glGenBuffers(OBJECTS_COUNT, vbo);

    for (i=0; i<OBJECTS_COUNT; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, vbo[i]);
        glBufferData(GL_ARRAY_BUFFER, sizeof (struct vertex_struct) * vertex_count[i], &vertices[vertex_offset_table[i]], GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    glGenBuffers(OBJECTS_COUNT, ibo);
    for (i=0; i<OBJECTS_COUNT; i++) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo[i]);
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
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo[index]);

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

    for (int i : objectsToRender)
    {
        textures[i]->bind();
        DrawMesh(i, 1);
    }
}

