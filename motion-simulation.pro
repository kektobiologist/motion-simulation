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
QMAKE_CXXFLAGS += -std=c++0x

PROTOFILES_CPP = proto/cpp/messages_robocup_ssl_detection.pb.cc proto/cpp/messages_robocup_ssl_geometry.pb.cc proto/cpp/messages_robocup_ssl_refbox_log.pb.cc proto/cpp/messages_robocup_ssl_wrapper.pb.cc proto/cpp/logging.pb.cc
PROTOFILES_H = proto/cpp/messages_robocup_ssl_detection.pb.h proto/cpp/messages_robocup_ssl_geometry.pb.h proto/cpp/messages_robocup_ssl_refbox_log.pb.h proto/cpp/messages_robocup_ssl_wrapper.pb.h proto/cpp/logging.pb.h

NETFILES_H = net/netraw.h net/robocup_ssl_client.h
NETFILES_CPP = net/netraw.cpp net/robocup_ssl_client.cpp

INCLUDEPATH += net/ proto/cpp/

SOURCES += main.cpp\
        dialog.cpp \
    pose.cpp \
    renderarea.cpp \
    controllers.cpp \
    $${PROTOFILES_CPP} \
    $${NETFILES_CPP} \
    visionworker.cpp \
    beliefstate.cpp \
    firarenderarea.cpp \
    serial.cpp \
    vision-velocity.cpp \
    logging.cpp \
    trajectory-drawing.cpp \
    tracking.cpp \
    controller-wrapper.cpp \
    simulation.cpp \
    trajectory.cpp \
    velocity-profile.cpp \
    arclength-param.cpp

HEADERS  += dialog.h \
    pose.h \
    geometry.h \
    renderarea.h \
    controllers.h \
    $${PROTOFILES_H} \
    $${NETFILES_H} \
    visionworker.h \
    beliefstate.h \
    firarenderarea.h \
    serial.h \
    vision-velocity.hpp \
    attacker.hpp \
    attacker.hpp \
    constants.h \
    logging.hpp \
    trajectory-drawing.hpp \
    tracking.hpp \
    trajectory-generators.hpp \
    controller-wrapper.hpp \
    simulation.hpp \
    trajectory.hpp \
    velocity-profile.hpp \
    arclength-param.hpp

LIBS += -lgsl -lgslcblas -lprotobuf
FORMS    += dialog.ui
