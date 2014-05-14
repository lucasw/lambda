TEMPLATE = app
TARGET = Lambda
DEPENDPATH += .
INCLUDEPATH += . /opt/X11/include /usr/include/malloc /opt/local/include
CONFIG += qt
LIBS += -L. -lrevel -lxvidcore -lX11 -L/opt/X11/lib -L/opt/local/lib


# Input
HEADERS += CImg.h lambda.h revel.h
SOURCES += lambda.cpp main.cpp

installfiles.files += Lambda.app
installfiles.path = /Applications
INSTALL += installfiles

# Icon

ICON = Icon.icns     
#QMAKE_INFO_PLIST = Info.plist
#OTHER_FILES += Info.plist
