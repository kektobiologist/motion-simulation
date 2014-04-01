#-------------------------------------------------
#
# Project created by QtCreator 2013-09-12T18:19:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = motion-simulation
TEMPLATE = app
CONFIG += link_pkgconfig
PKGCONFIG += opencv

SOURCES += main.cpp\
        dialog.cpp \
    pose.cpp \
    renderarea.cpp \
    controllers.cpp

HEADERS  += dialog.h \
    pose.h \
    geometry.h \
    renderarea.h \
    controllers.h
LIBS += -lgsl -lgslcblas
FORMS    += dialog.ui
