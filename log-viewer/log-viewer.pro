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
    ../beliefstate.cpp \
    ../pose.cpp \
    firarenderarea2.cpp

HEADERS  += dialog.h \
    ../proto/cpp/logging.pb.h \
    ../beliefstate.h \
    ../pose.h \
    ../geometry.h \
    firarenderarea2.h

INCLUDEPATH += ../ \
               ../proto/cpp

FORMS    += dialog.ui

LIBS += -lprotobuf
