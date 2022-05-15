QT += widgets serialport
QT += multimedia

CONFIG += gui
CONFIG += c++14
HEADERS    = \
    CmdLink.h \
    Lidar/lidar.h \
    Lidar/rplidar_cmd.h \
    Lidar/rplidar_protocol.h \
    Lidar/sl_crc.h \
    Lidar/sl_lidar_cmd.h \
    Lidar/sl_lidar_driver.h \
    Lidar/sl_lidar_protocol.h \
    Lidar/sl_types.h \
    absolutepostracker.h \
    botcomm.h \
    botcommmonitor.h \
    botcommserial.h \
    botconnection.h \
    clipper.h \
    networkclient.h \
    networkplugin.h \
    networkserver.h \
    polyops.h \
    simplecrc.h \
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
    CmdLink.cpp \
    Lidar/lidar.cpp \
    Lidar/sl_crc.cpp \
    Lidar/sl_lidar_driver.cpp \
    absolutepostracker.cpp \
    botcomm.cpp \
    botcommmonitor.cpp \
    botcommserial.cpp \
    botconnection.cpp \
    clipper.cpp \
    main.cpp \
    graphics.cpp \
    networkclient.cpp \
    networkplugin.cpp \
    networkserver.cpp \
    polyops.cpp \
    simplecrc.cpp \
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


