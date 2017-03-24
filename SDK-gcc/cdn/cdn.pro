TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt
DEFINES += _DEBUG
SOURCES += \
    cdn.cpp \
    deploy.cpp \
    io.cpp \
    simplex.cpp

DISTFILES +=

HEADERS += \
    deploy.h \
    simplex.h

DESTDIR = $$OUT_PWD/../bin
INCLUDEPATH += $$PWD/lib


