#include "renderer.h"
#include <QtOpenGL>
#include <GL/glu.h>
#include "smallsat.h"

using namespace scene;


Renderer::Renderer(QWidget *parent) : QOpenGLWidget(parent)
{
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
    gluPerspective(45.0, (float) this->width() / (float) this->height(), 0.15, 251.0);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_COLOR_BUFFER_BIT);


    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(-20, -20, 10,
              0, 0, 0,
              0, 0, 1);

    glEnable(GL_TEXTURE_2D);
    DrawAllMeshes();

    update();
}

void Renderer::resizeGL(int width, int height)
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
        DrawMesh(i, 1);
    }
}

