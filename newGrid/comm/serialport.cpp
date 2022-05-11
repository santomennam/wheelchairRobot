#include "serialport.h"
#include <iostream>

using namespace std;

void throwOnError(enum sp_return result);

SerialPort::SerialPort()
{
    port = nullptr;
}

SerialPort::~SerialPort()
{
    close();
}

void SerialPort::close()
{
    if (port) {
        sp_close(port);
        sp_free_port(port);
        port = nullptr;
    }
}

bool SerialPort::open(std::string portName, int baud)
{
    if (sp_get_port_by_name(portName.c_str(), &port) != SP_OK) {
        return false;
    }

    if (sp_open(port, SP_MODE_READ_WRITE)) {
        return false;
    }

    struct sp_port_config *port_config;
    throwOnError(sp_new_config(&port_config));
    throwOnError(sp_set_config_baudrate(port_config, baud));
    throwOnError(sp_set_config_bits(port_config, 8));
    throwOnError(sp_set_config_parity(port_config, SP_PARITY_NONE));
    throwOnError(sp_set_config_stopbits(port_config, 1));
    throwOnError(sp_set_config_flowcontrol(port_config, SP_FLOWCONTROL_NONE));

    throwOnError(sp_set_config(port, port_config));
    sp_free_config(port_config);

    return true;
}

string SerialPort::read()
{
    int bytes_waiting = static_cast<int>(sp_input_waiting(port));
    if (bytes_waiting > 0) {
        string buff(bytes_waiting, 0);
        throwOnError(sp_nonblocking_read(port, buff.data(), bytes_waiting));
        return buff;
    }
    else if (bytes_waiting < 0) {
        throwOnError(static_cast<sp_return>(bytes_waiting));
    }
    return string{};
}

void SerialPort::write(std::string data)
{
    throwOnError(sp_nonblocking_write(port, data.data(), data.size()));
}

void SerialPort::write(char c)
{
    throwOnError(sp_nonblocking_write(port, &c, 1));
}

void SerialPort::write(int value)
{
    out << value;
    write(out.str());
    out.str("");
}

void SerialPort::write(double value)
{
    out << value;
    write(out.str());
    out.str("");
}

bool SerialPort::stillWriting()
{
    return sp_input_waiting(port) > 0;
}

string SerialPort::readDelimited(char startChar, char endChar)
{
    int bytes_waiting = static_cast<int>(sp_input_waiting(port));
    if (bytes_waiting > 0) {
        int offset = inputBuff.size();
        inputBuff.resize(offset + bytes_waiting);
        throwOnError(sp_nonblocking_read(port, &inputBuff[offset], bytes_waiting));

        int startIdx = -1;
        int endIdx = -1;

        for (int i = 0; i < inputBuff.size(); i++) {
            if (inputBuff[i] == startChar) {
                startIdx = i;
                break;
            }
        }

        if (startIdx < 0) {
            return "";
        }

        for (int i = startIdx + 1; i < inputBuff.size(); i++) {
            if (inputBuff[i] == endChar) {
                endIdx = i;
                break;
            }
        }

        if (endIdx < 0) {
            return "";
        }

        string data = string(&inputBuff[startIdx+1], endIdx-startIdx-1);

        inputBuff.erase(inputBuff.begin(), inputBuff.begin()+endIdx+1);

        return data;
    }
    else if (bytes_waiting < 0) {
        throwOnError(static_cast<sp_return>(bytes_waiting));
    }

    return string{};
}

SerialPorts::SerialPorts()
{
    refresh();
}

SerialPorts::~SerialPorts()
{
    sp_free_port_list(port_list);
}

int SerialPorts::refresh()
{
    if (!port_list) {
        sp_free_port_list(port_list);
        port_list = nullptr;
    }

    numPorts = 0;

    sp_return result = sp_list_ports(&port_list);

    if (result != SP_OK) {
        cout << "sp_list_ports() failed!" << endl;
        return 0;
    }

    /* Iterate through the ports. When port_list[i] is NULL
     * this indicates the end of the list. */
    for (int i = 0; port_list[i] != nullptr; i++) {

        numPorts++;

        sp_port *port = port_list[i];

        /* Get the name of the port. */
        string port_name = sp_get_port_name(port);

        cout << "Found Port: " << port_name << endl;
    }

    return numPorts;
}

int SerialPorts::size()
{
    return numPorts;
}

string SerialPorts::operator[](int idx)
{
    if (idx >= numPorts) {
        return "";
    }
    return sp_get_port_name(port_list[idx]);
}

string serialPortLastError()
{
    char *error_message = sp_last_error_message();
    string msg = string("SerialPort Error: ") + error_message;
    sp_free_error_message(error_message);
    return msg;
}

/* Helper function for error handling. */
void throwOnError(enum sp_return result)
{
   switch (result) {
   case SP_ERR_ARG:
       throw logic_error("Serial Port: Invalid Argument");
   case SP_ERR_FAIL:
       throw runtime_error(serialPortLastError());
   case SP_ERR_SUPP:
       throw runtime_error("Serial Port: Not Supported");
   case SP_ERR_MEM:
       throw runtime_error("Serial Port: Memory Alloc Error");
   case SP_OK:
       break;
   }
}

