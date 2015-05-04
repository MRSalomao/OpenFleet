#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QVector3D>
#include <QQuaternion>
#include <QFileDialog>
#include <QDebug>

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

    in >> str;
    str.remove('[');
    str.remove(']');

    return str;
}

void MainWindow::on_loadBtn_clicked()
{
    Renderer::si->shouldUpdate = false;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open path file"));
    Renderer::si->shouldUpdate = true;
    Renderer::si->update();

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);

    QString str;

    // ([px, py, pz, qt, qx, qy, qz, vx, vy, vz, wx, wy, wz])
    float px, py, pz;
    float qw, qx, qy, qz;

    Renderer::si->trajectoryPos.clear();
    Renderer::si->trajectoryRot.clear();

    while (!in.atEnd())
    {
        cleanStr(in); str = cleanStr(in);

        px = str.toFloat(); str = cleanStr(in);
        py = str.toFloat(); str = cleanStr(in);
        pz = str.toFloat(); str = cleanStr(in);

        qw = str.toFloat(); str = cleanStr(in);
        qx = str.toFloat(); str = cleanStr(in);
        qy = str.toFloat(); str = cleanStr(in);
        qz = str.toFloat(); str = cleanStr(in);

        Renderer::si->trajectoryPos.push_back(QVector3D(px, py, pz));
        Renderer::si->trajectoryRot.push_back(QQuaternion(qw, qx, qy, qz));

//        qDebug() << Renderer::si->trajectoryPos.back() << " " << Renderer::si->trajectoryRot.back() ;

        for (int i=0; i<5; i++) cleanStr(in); // Discard the rest
    }

    file.close();


}

void MainWindow::on_dispTrajectoriesBox_clicked(bool checked)
{
    drawTrajectories = checked;
}

void MainWindow::on_simulateBtn_clicked()
{
    p->simulate();
}
