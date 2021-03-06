#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ssplanner.h"

namespace Ui {
class MainWindow;
}

class SSPlanner;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    SSPlanner* p;

    static MainWindow *si;

    bool drawTrajectories = true;

private slots:

    void on_loadBtn_clicked();

    void on_dispTrajectoriesBox_clicked(bool checked);

    void on_simulateBtn_clicked();

    void on_ISS_radio_clicked();

    void on_SolarSail_radio_clicked();

    void on_SpaceShuttle_radio_clicked();

    void on_Asteroid_radio_clicked();

    void on_playBtn_clicked();

    void on_pauseBtn_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
