
#include "serialportreader.h"

#include <QCoreApplication>
#include <iostream>

QT_USE_NAMESPACE

using namespace std;


SerialPortReader::SerialPortReader(QObject *parent, const std::string& serialPortName, QSerialPort::BaudRate baud) :
    Plugin(parent), serialPortFailed(false)
{
    serialPort.setPortName(QString::fromStdString(serialPortName));
    serialPort.setBaudRate(baud);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);

    if (!serialPort.open(QIODevice::ReadWrite))
    {
        cout << "Error opening serial port" << endl;
        cout << serialPort.errorString().toStdString() << endl;
        serialPortFailed = true;
        serialPortError  = "Error opening serial port";
        return;
    }

   // serialPort.setDataTerminalReady(true);


    connect(&serialPort, SIGNAL(readyRead()), this, SLOT(handleReadyRead()));
    connect(&serialPort, SIGNAL(error(QSerialPort::SerialPortError)), SLOT(handleError(QSerialPort::SerialPortError)));
}

SerialPortReader::~SerialPortReader()
{
}

void SerialPortReader::handleReadyRead()
{
    readData.append(serialPort.readAll());
}


void SerialPortReader::handleError(QSerialPort::SerialPortError serialPortError)
{
    std::cout << "Some Kinda SerialPort Error" << endl;
    std::cout << serialPort.errorString().toStdString() << endl;
    switch (serialPortError) {
    case QSerialPort::NoError:                   cout << "::NoError:                   " << endl; break;
    case QSerialPort::DeviceNotFoundError:       cout << "::DeviceNotFoundError:       " << endl; break;
    case QSerialPort::PermissionError:           cout << "::PermissionError:           " << endl; break;
    case QSerialPort::OpenError:                 cout << "::OpenError:                 " << endl; break;
    case QSerialPort::ParityError:               cout << "::ParityError:               " << endl; break;
    case QSerialPort::FramingError:              cout << "::FramingError:              " << endl; break;
    case QSerialPort::BreakConditionError:       cout << "::BreakConditionError:       " << endl; break;
    case QSerialPort::WriteError:                cout << "::WriteError:                " << endl; break;
    case QSerialPort::ReadError:                 cout << "::ReadError:                 " << endl; break;
    case QSerialPort::ResourceError:             cout << "::ResourceError:             " << endl; break;
    case QSerialPort::UnsupportedOperationError: cout << "::UnsupportedOperationError: " << endl; break;
    case QSerialPort::UnknownError:              cout << "::UnknownError:              " << endl; break;
    case QSerialPort::TimeoutError:              cout << "::TimeoutError:              " << endl; break;
    case QSerialPort::NotOpenError:              cout << "::NotOpenError:              " << endl; break;
    }

}

bool SerialPortReader::shouldDelete()
{
    return false;
}

void SerialPortReader::call(int arg1, int arg2, const std::string& arg3)
{
    switch (static_cast<Command>(arg1)) {
    case Command::send:
       // std::cout << "Sending data: " << arg3<< std::endl;
//        serialPort.clearError();
        serialPort.write(arg3.data(), arg3.size());
        serialPort.flush();
        sentCount++;
        break;
    case Command::close:
        break;
    }
    
}

void SerialPortReader::update(std::function<void(int, int, int, const std::string&)> sendEvent)
{
    if (sentCount > 0) {
        sentCount = 0;
       // sendEvent(2, 0, 0, "Sent");
    }
    if (readData.size() > 0)
    {
        sendEvent(1, 0, 0, readData.toStdString());
        readData.clear();
    }
}
