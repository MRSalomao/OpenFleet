/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.4.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>
#include "renderer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGroupBox *groupBox;
    QPushButton *loadBtn;
    QCheckBox *dispTrajectoriesBox;
    QRadioButton *ISS_radio;
    QRadioButton *SpaceShuttle_radio;
    QRadioButton *Asteroid_radio;
    QLabel *label_2;
    QFrame *line;
    QFrame *line_2;
    QPushButton *playBtn;
    QPushButton *pauseBtn;
    QPushButton *simulateBtn;
    QFrame *line_3;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label_3;
    QDoubleSpinBox *doubleSpinBox;
    QRadioButton *SolarSail_radio;
    QLabel *label;
    Renderer *renderer;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(810, 571);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 80, 161, 481));
        loadBtn = new QPushButton(groupBox);
        loadBtn->setObjectName(QStringLiteral("loadBtn"));
        loadBtn->setGeometry(QRect(10, 444, 141, 31));
        dispTrajectoriesBox = new QCheckBox(groupBox);
        dispTrajectoriesBox->setObjectName(QStringLiteral("dispTrajectoriesBox"));
        dispTrajectoriesBox->setGeometry(QRect(0, 253, 171, 22));
        dispTrajectoriesBox->setChecked(true);
        ISS_radio = new QRadioButton(groupBox);
        ISS_radio->setObjectName(QStringLiteral("ISS_radio"));
        ISS_radio->setGeometry(QRect(0, 70, 171, 51));
        ISS_radio->setChecked(true);
        SpaceShuttle_radio = new QRadioButton(groupBox);
        SpaceShuttle_radio->setObjectName(QStringLiteral("SpaceShuttle_radio"));
        SpaceShuttle_radio->setGeometry(QRect(0, 156, 171, 31));
        Asteroid_radio = new QRadioButton(groupBox);
        Asteroid_radio->setObjectName(QStringLiteral("Asteroid_radio"));
        Asteroid_radio->setGeometry(QRect(0, 192, 171, 31));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(24, 44, 111, 20));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        label_2->setFont(font);
        line = new QFrame(groupBox);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(-3, 227, 171, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(groupBox);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(0, 281, 161, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        playBtn = new QPushButton(groupBox);
        playBtn->setObjectName(QStringLiteral("playBtn"));
        playBtn->setGeometry(QRect(10, 404, 61, 31));
        pauseBtn = new QPushButton(groupBox);
        pauseBtn->setObjectName(QStringLiteral("pauseBtn"));
        pauseBtn->setGeometry(QRect(80, 404, 71, 31));
        simulateBtn = new QPushButton(groupBox);
        simulateBtn->setObjectName(QStringLiteral("simulateBtn"));
        simulateBtn->setGeometry(QRect(10, 364, 141, 31));
        line_3 = new QFrame(groupBox);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setGeometry(QRect(0, 340, 161, 20));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(15, 304, 131, 31));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout->addWidget(label_3);

        doubleSpinBox = new QDoubleSpinBox(layoutWidget);
        doubleSpinBox->setObjectName(QStringLiteral("doubleSpinBox"));
        doubleSpinBox->setDecimals(0);
        doubleSpinBox->setMinimum(1);
        doubleSpinBox->setMaximum(10);
        doubleSpinBox->setValue(1);

        horizontalLayout->addWidget(doubleSpinBox);

        SolarSail_radio = new QRadioButton(groupBox);
        SolarSail_radio->setObjectName(QStringLiteral("SolarSail_radio"));
        SolarSail_radio->setGeometry(QRect(0, 123, 171, 31));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(0, 9, 181, 81));
        QFont font1;
        font1.setPointSize(23);
        font1.setBold(true);
        font1.setWeight(75);
        label->setFont(font1);
        label->setAlignment(Qt::AlignCenter);
        renderer = new Renderer(centralWidget);
        renderer->setObjectName(QStringLiteral("renderer"));
        renderer->setGeometry(QRect(180, 10, 621, 551));
        renderer->setMouseTracking(true);
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Simulator", 0));
        groupBox->setTitle(QString());
        loadBtn->setText(QApplication::translate("MainWindow", "Load from file", 0));
        dispTrajectoriesBox->setText(QApplication::translate("MainWindow", "Display Trajectories", 0));
        ISS_radio->setText(QApplication::translate("MainWindow", "International \n"
"Space Station", 0));
        SpaceShuttle_radio->setText(QApplication::translate("MainWindow", "Space Shuttle", 0));
        Asteroid_radio->setText(QApplication::translate("MainWindow", "Asteroid Field", 0));
        label_2->setText(QApplication::translate("MainWindow", "Environment:", 0));
        playBtn->setText(QApplication::translate("MainWindow", "Play", 0));
        pauseBtn->setText(QApplication::translate("MainWindow", "Pause", 0));
        simulateBtn->setText(QApplication::translate("MainWindow", "Simulate", 0));
        label_3->setText(QApplication::translate("MainWindow", "Fleet Size:", 0));
        SolarSail_radio->setText(QApplication::translate("MainWindow", "Solar Sail", 0));
        label->setText(QApplication::translate("MainWindow", "SmallSat \n"
"Simulator", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
