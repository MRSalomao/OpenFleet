TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    displayOpenDE.cpp

HEADERS += \
    displayOpenDE.h

OTHER_FILES += \
    OMPLEnvironment.inc \
    OMPLSetup.inc \
    OpenDEWorld.inc


LIBS += -lompl -lode -ldrawstuff -lGL -lGLU -lX11 -lboost_system -lboost_thread -lboost_program_options

QMAKE_CXXFLAGS += -DdDOUBLE
