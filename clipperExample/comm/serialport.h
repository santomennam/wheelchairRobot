#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <sstream>
#include <vector>
#include "libserialport.h"

// https://sourceforge.net/projects/com0com/

class SerialPort
{
    sp_port *port;
    std::stringstream out;
    std::vector<char> inputBuff;
public:
    SerialPort();
   ~SerialPort();

    bool open(std::string portName, int baud);
    void close();

    std::string name() { return sp_get_port_name(port); }
    std::string description() { return sp_get_port_description(port); }
    sp_transport transport() { return sp_get_port_transport(port); }
    std::string manufacturer() { return sp_get_port_usb_manufacturer(port); }
    std::string product() { return sp_get_port_usb_product(port); }
    std::string serial() { return sp_get_port_usb_serial(port); }

    std::string read();
    void write(std::string data);
    void write(char c);
    void write(int value);
    void write(double value);
    bool stillWriting();
    std::string readDelimited(char startChar, char endChar);
};

class SerialPorts
{
    sp_port **port_list{nullptr};
    int numPorts{0};
public:
    SerialPorts();
   ~SerialPorts();
    int refresh();
    int size();
    std::string operator[](int idx);
};


#endif // SERIALPORT_H
