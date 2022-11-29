#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <string>
#include <vector>
#include <sys/ioctl.h>
#include <termios.h>

// serial port related code is based on https://github.com/gbmhunter/CppLinuxSerial

namespace sp
{
    enum class BaudRate
    {
        B_0,
        B_50,
        B_75,
        B_110,
        B_134,
        B_150,
        B_200,
        B_300,
        B_600,
        B_1200,
        B_1800,
        B_2400,
        B_4800,
        B_9600,
        B_19200,
        B_38400,
        B_57600,
        B_115200,
        B_230400,
        B_460800,
    };

    enum class NumDataBits
    {
        FIVE,
        SIX,
        SEVEN,
        EIGHT,
    };

    enum class Parity
    {
        NONE,
        EVEN,
        ODD,
    };

    enum class NumStopBits
    {
        ONE,
        TWO,
    };

    enum class HardwareFlowControl
    {
        OFF,
        ON,
    };

    enum class SoftwareFlowControl
    {
        OFF,
        ON,
    };

    class PortConfig
    {
    public:
        BaudRate m_baudRate;
        NumDataBits m_numDataBits;
        Parity m_parity;
        NumStopBits m_numStopBits;
        int m_timeout;
        HardwareFlowControl m_hardwareFlowControl;
        SoftwareFlowControl m_softwareFlowControl;

        PortConfig();
        void SetBaudRate(sp::BaudRate baudRate);
        void SetNumDataBits(sp::NumDataBits numDataBits);
        void SetParity(sp::Parity parity);
        void SetNumStopBits(sp::NumStopBits numStopBits);
        void SetTimeout(int timeout);
        void SetHwFlowControl(sp::HardwareFlowControl hardwareFlowControl);
        void SetSwFlowControl(sp::SoftwareFlowControl softwareFlowControl);
    };

    class SerialPort
    {
    private:
        int m_fileDesc;
        bool ConfigurePort(PortConfig pc);
        termios GetTermios();
        void SetTermios(termios tty);
        std::vector<char> m_readBuffer;
        static constexpr unsigned int BUFSIZE = 255;
    public:
        SerialPort();
        SerialPort(const SerialPort&) = delete;
        SerialPort(SerialPort&& other);
        SerialPort& operator=(const SerialPort& other) = delete;
        SerialPort& operator=(SerialPort&& other);
        bool open(std::string name, PortConfig pc);
        void close();
        int Write(const std::string& data);
        int WriteBinary(const std::vector<uint8_t>& data);
        int Read(std::string& data);
        int ReadBinary(std::vector<uint8_t>& data);
    };
}

#endif // SERIALPORT_H
