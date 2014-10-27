#-------------------------------------------------
#
# Project created by QtCreator 2014-10-25T21:35:21
#
#-------------------------------------------------

QT       += core gui

TARGET = log-viewer
TEMPLATE = app


SOURCES += main.cpp\
        dialog.cpp \
    ../proto/cpp/logging.pb.cc \
    firarenderarea.cpp \
    ../beliefstate.cpp \
    ../pose.cpp

HEADERS  += dialog.h \
    ../proto/cpp/logging.pb.h \
    firarenderarea.h \
    ../beliefstate.h \
    ../pose.h \
    ../geometry.h

FORMS    += dialog.ui
