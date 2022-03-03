QT += widgets serialport
QT += multimedia

CONFIG += gui
CONFIG += c++14
HEADERS    = \
    absolutepostracker.h \
    networkclient.h \
    networkplugin.h \
    networkserver.h \
    window.h \
    graphics.h \
    vec2d.h \
    vec3d.h \
    plugin.h \
    segment.h \
    world.h \
    robot.h \
    obstacle.h \
    areatree.h \
    node.h \
    area.h \
    packet.h \
    serialportreader.h \
    visualobject.h \
    viewport.h \
    waypoint.h \
    geometry.h \
    physics.h

SOURCES     = \
    absolutepostracker.cpp \
    main.cpp \
    graphics.cpp \
    networkclient.cpp \
    networkplugin.cpp \
    networkserver.cpp \
    vec2d.cpp \
    vec3d.cpp \
    plugin.cpp \
    segment.cpp \
    world.cpp \
    robot.cpp \
    obstacle.cpp \
    areatree.cpp \
    node.cpp \
    area.cpp \
    packet.cpp \
    serialportreader.cpp \
    visualobject.cpp \
    viewport.cpp \
    waypoint.cpp \
    geometry.cpp \
    physics.cpp

DISTFILES += \
    notes.txt


