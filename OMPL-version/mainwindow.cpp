#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QVector3D>
#include <QQuaternion>
#include <QFileDialog>
#include <QDebug>

#define OBJECTS_COUNT 8
#define ASTEROID3 0
#define ASTEROID2 1
#define SOLARSAIL 2
#define SPACESHUTTLE 3
#define SKYBOX 4
#define ISS 5
#define CUBESAT 6
#define ASTEROID1 7

MainWindow* MainWindow::si;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    p = new SSPlanner();

    si = this;
}

MainWindow::~MainWindow()
{
    delete ui;
}

QString cleanStr(QTextStream &in)
{
    QString str;

    do
    {
        in >> str;
        str.remove('[');
        str.remove(']');
    }
    while (str.size()==0 && !in.atEnd());

    return str;
}

// adapted from http://www.cs.stanford.edu/~acoates/quaternion.h
QQuaternion eulerToQuat(float x, float y, float z)
{
    QQuaternion q;

    float c1 = cos(z * 0.5);
    float c2 = cos(y * 0.5);
    float c3 = cos(x * 0.5);
    float s1 = sin(z * 0.5);
    float s2 = sin(y * 0.5);
    float s3 = sin(x * 0.5);

    q.setScalar(c1*c2*s3 - s1*s2*c3);
    q.setX(c1*s2*c3 + s1*c2*s3);
    q.setY(s1*c2*c3 - c1*s2*s3);
    q.setZ(c1*c2*c3 + s1*s2*s3);

    return q;
}

void MainWindow::on_loadBtn_clicked()
{
    Renderer::si->shouldUpdate = false;
    // prompt the user to select the files to render
    QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Open path file"));
    Renderer::si->shouldUpdate = true;
    Renderer::si->update();

    Renderer::si->trajectoryPos.clear();
    Renderer::si->trajectoryRot.clear();

    Renderer::si->asteroidPos.clear();
    Renderer::si->asteroidRot.clear();
    Renderer::si->asteroidVel.clear();
    Renderer::si->asteroidAngVel.clear();

    for (QString fileName : fileNames)
    {
        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
        QTextStream in(&file);

        QString str;

        // ([px, py, pz, qt, qx, qy, qz, vx, vy, vz, wx, wy, wz])
        float px, py, pz;
        float qw, qx, qy, qz;


        // read file as a sat if it doesn't contain the word "asteroid"
        if (!fileName.contains("asteroid"))
        {
            Renderer::si->trajectoryPos.push_back(QVector<QVector3D>());
            Renderer::si->trajectoryRot.push_back(QVector<QQuaternion>());

            while (!in.atEnd())
            {
                str = cleanStr(in);

                px = str.toFloat(); str = cleanStr(in);
                py = str.toFloat(); str = cleanStr(in);
                pz = str.toFloat(); str = cleanStr(in);

                qw = str.toFloat(); str = cleanStr(in);
                qx = str.toFloat(); str = cleanStr(in);
                qy = str.toFloat(); str = cleanStr(in);
                qz = str.toFloat(); str = cleanStr(in);

                Renderer::si->trajectoryPos.back().push_back(QVector3D(px, py, pz));
                Renderer::si->trajectoryRot.back().push_back(QQuaternion(qw, qx, qy, qz));

                for (int i=0; i<5; i++) cleanStr(in); // Discard the rest
            }
        }
        // else read it as an asteroid
        else
        {
            str = cleanStr(in);

            px = str.toFloat(); str = cleanStr(in);
            py = str.toFloat(); str = cleanStr(in);
            pz = str.toFloat(); str = cleanStr(in);

            qw = str.toFloat(); str = cleanStr(in);
            qx = str.toFloat(); str = cleanStr(in);
            qy = str.toFloat(); str = cleanStr(in);
            qz = str.toFloat(); str = cleanStr(in);

            Renderer::si->asteroidPos.push_back(QVector3D(px, py, pz));
            Renderer::si->asteroidRot.push_back(QQuaternion(qw, qx, qy, qz));

            px = str.toFloat(); str = cleanStr(in);
            py = str.toFloat(); str = cleanStr(in);
            pz = str.toFloat(); str = cleanStr(in);

            Renderer::si->asteroidVel.push_back(QVector3D(px, py, pz));

            px = str.toFloat(); str = cleanStr(in);
            py = str.toFloat(); str = cleanStr(in);
            pz = str.toFloat(); str = cleanStr(in);

            Renderer::si->asteroidAngVel.push_back( eulerToQuat(px, py, pz) );
        }
        file.close();
    }

}

// draw trajectories? YES / NO
void MainWindow::on_dispTrajectoriesBox_clicked(bool checked)
{
    drawTrajectories = checked;
}

// start simulation using OMPL's KPIECE
void MainWindow::on_simulateBtn_clicked()
{
    p->simulate();
}


// choose the environment
void MainWindow::on_ISS_radio_clicked()
{
    Renderer::si->objectsToRender.clear();
    Renderer::si->objectsToRender << SKYBOX << ISS << CUBESAT;
}

void MainWindow::on_SolarSail_radio_clicked()
{
    Renderer::si->objectsToRender.clear();
    Renderer::si->objectsToRender << SKYBOX << SOLARSAIL << CUBESAT;
}

void MainWindow::on_SpaceShuttle_radio_clicked()
{
    Renderer::si->objectsToRender.clear();
    Renderer::si->objectsToRender << SKYBOX << SPACESHUTTLE << CUBESAT;
}

void MainWindow::on_Asteroid_radio_clicked()
{
    Renderer::si->objectsToRender.clear();
    Renderer::si->objectsToRender << SKYBOX << ASTEROID1 << ASTEROID2 << ASTEROID3 << CUBESAT;
}

// play button pressed
void MainWindow::on_playBtn_clicked()
{
    Renderer::si->isPaused = false;
}

// pause button pressed
void MainWindow::on_pauseBtn_clicked()
{
    Renderer::si->isPaused = true;
}
