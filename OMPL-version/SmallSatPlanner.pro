#-------------------------------------------------
#
# Project created by QtCreator 2015-04-28T06:48:27
#
#-------------------------------------------------

QT += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SmallSatPlanner
TEMPLATE = app

LIBS += -lGLU -lompl -lode -lGL -lGLU -lX11 -lboost_system -lboost_thread -lboost_program_options

QMAKE_CXXFLAGS += -DdDOUBLE

CONFIG += c++11

SOURCES += main.cpp\
        mainwindow.cpp \
    renderer.cpp \
    ssplanner.cpp

HEADERS  += mainwindow.h \
    renderer.h \
    ssplanner.h \
    smallsat.h

FORMS    += mainwindow.ui
