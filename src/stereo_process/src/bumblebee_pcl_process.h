///////////////////////////////////////////////////////////////////////////////////////////////
//// Name         : detection_behavior.h                                                   ////
//// Description  : Header file for detection_behavior.cpp                                 ////
//// Function     : This header file contains assistive definitions and functions for use  ////
////                in the .cpp file, mainly for handling the serial port interaction      ////
//// Dependencies : QtExtSerialPort, for the communication with the serial port            ////
//// Maintainer   : Stasinopoulos Sotirios     email:sotstas@gmail.com                     ////
//// Last update  : 2014.08.27                                                             ////
///////////////////////////////////////////////////////////////////////////////////////////////
//#ifndef DETECTION_BEHAVIOR_H
//#define DETECTION_BEHAVIOR_H

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sstream>
#include <vector>
#include <time.h>
#include <sensor_msgs/LaserScan.h>
#include "QDebug"
#include "QCoreApplication"
#include <QtExtSerialPort/qextserialport.h>
//using namespace std;

class SerialPort
{
public:
QextSerialPort *m_Port;
QString m_PortName;
int m_BaudRate;
QByteArray serial_input_data;
char* data;
//qint64 read_byte_number;
bool data_read_flag;


SerialPort()
{
    m_PortName = QLatin1String("serial_to_controller");
    m_BaudRate=115200;
    //read_byte_number = 55;
    data_read_flag = false;
}

~SerialPort()
{
    delete m_Port;
}


int SetPortBaud(int baud)
{
    BaudRateType brt;
    switch(baud)
    {
    case 9600:brt=BAUD9600;break;
    case 115200:brt=BAUD115200;break;
    default :brt=BAUD1152000;break;
    }
    m_Port->setBaudRate(brt);
    return baud;
}

bool OpenPort(QString port_name, int baud)
{
    m_PortName=port_name;    //QLatin1String->more efficient
    m_Port = new QextSerialPort(m_PortName, QextSerialPort::EventDriven);

//    connect(m_Port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
//    m_Port->connect(m_Port, SIGNAL(readyRead()), this, SLOT(onDataAvailable()));
    //set parameter
    SetPortBaud(baud);
    m_Port->setFlowControl(FLOW_OFF);
    m_Port->setParity(PAR_NONE);
    m_Port->setDataBits(DATA_8);
    m_Port->setStopBits(STOP_1);

    //open port
    if (m_Port->open(QIODevice::ReadWrite) == true)
     {
            qDebug() << "listening for data on" << m_Port->portName();
            return true;
     }
    else
     {
            qDebug() << "device failed to open:" << m_Port->errorString();
            return false;
     }

}

void ClosePort()
{
        m_Port->close();
}

int SendWriteCommand(QByteArray cmd)
{
    return  m_Port->write(cmd,cmd.size());
}

//void onDataAvailable()
//{
//    QByteArray serial_input_data_remainder;
//    int a;
//    if (a=m_Port->bytesAvailable())
//    {
//        //serial_input_data.resize(a);
//        serial_input_data.resize(read_byte_number);
//        //serial_input_data = m_Port->readAll();
//        m_Port->read(serial_input_data.data(),read_byte_number);
//        data_read_flag = true;
//        // look if I want to flush it to avoid delay
//        //m_Port->flush();
//        serial_input_data_remainder = m_Port->readAll();
//    }
//    else
//    {
//        qDebug() << "Ready to read, but no bytes available";
//        data_read_flag = false;
//    }
//}
};

// Interface
class LowpassFilter {
private:
    int m;  // circular buffer length
    double x[10]; // input vector
    double y[10]; // output vector
    int n;    // pointer to the current array index

public:

// Implementation

// filter coefficients a and b
    double a[4];
    double b[4];


LowpassFilter() //Constructor
{
    m=10;
    for (int i=0;i<m;i++)
    {
        x[m]=0;
        y[m]=0;
    };
    n=0;
    a[0] = 1.0;
    a[1] = -0.577240524806303;
    a[2] = 0.421787048689562;
    a[3] = -0.056297236491843;
    b[0] = 0.098531160923927;
    b[1] = 0.295593482771781;
    b[2] = 0.295593482771781;
    b[3] = 0.098531160923927;
}

double filter(double sample)
{
    x[n] = sample;
    y[n] = b[0] * x[n] + b[1] * x[(n-1+m)%m] + b[2] * x[(n-2+m)%m] + b[3] * x[(n-3+m)%m]
                       - a[1] * y[(n-1+m)%m] - a[2] * y[(n-2+m)%m] - a[3] * y[(n-3+m)%m];

    double result = y[n];
    n = (n + 1) % 10; // new pointer index
    return result;
}
};
