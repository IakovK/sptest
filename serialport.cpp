#include <fcntl.h>
#include <unistd.h>

#include "serialport.h"

namespace sp
{
    PortConfig::PortConfig()
    {
        m_baudRate = BaudRate::B_57600;
        m_numDataBits = NumDataBits::EIGHT;
        m_parity = Parity::NONE;
        m_numStopBits = NumStopBits::ONE;
        m_timeout = 60000;
        m_hardwareFlowControl = HardwareFlowControl::OFF;
        m_softwareFlowControl = SoftwareFlowControl::OFF;
    }

    void PortConfig::SetBaudRate(sp::BaudRate baudRate)
    {
        m_baudRate = baudRate;
    }

    void PortConfig::SetNumDataBits(sp::NumDataBits numDataBits)
    {
        m_numDataBits = numDataBits;
    }

    void PortConfig::SetParity(sp::Parity parity)
    {
        m_parity = parity;
    }

    void PortConfig::SetNumStopBits(sp::NumStopBits numStopBits)
    {
        m_numStopBits = numStopBits;
    }

    void PortConfig::SetTimeout(int timeout)
    {
        m_timeout = timeout;
    }

    void PortConfig::SetHwFlowControl(sp::HardwareFlowControl hardwareFlowControl)
    {
        m_hardwareFlowControl = hardwareFlowControl;
    }

    void PortConfig::SetSwFlowControl(sp::SoftwareFlowControl softwareFlowControl)
    {
        m_softwareFlowControl = softwareFlowControl;
    }

    SerialPort::SerialPort()
    {
        m_fileDesc = -1;
        m_readBuffer.reserve(BUFSIZE);
    }

    SerialPort::SerialPort(SerialPort&& other)
    {
        m_fileDesc = other.m_fileDesc;
        m_readBuffer = other.m_readBuffer;
        other.m_fileDesc = -1;
    }

    SerialPort& SerialPort::operator=(SerialPort&& other)
    {
        if (this != &other)
        {
            close();
            m_fileDesc = other.m_fileDesc;
            m_readBuffer = other.m_readBuffer;
            other.m_fileDesc = -1;
        }
        return *this;
    }


    bool SerialPort::open(std::string name, PortConfig pc)
    {
        if(m_fileDesc != -1)
        {
            close();
        }

        m_fileDesc = ::open(name.c_str(), O_RDWR);

        if(m_fileDesc == -1)
        {
            return false;
        }

        if (ConfigurePort(pc))
            return true;
        else
        {
            close();
            return false;
        }
    }

    void SerialPort::close()
    {
        if (m_fileDesc != -1)
        {
            ::close(m_fileDesc);
            m_fileDesc = -1;
        }
    }

    termios SerialPort::GetTermios()
    {
        struct termios term;
        ioctl(m_fileDesc, TCGETS, &term);
        return term;
    }

    void SerialPort::SetTermios(termios tty)
    {
        ioctl(m_fileDesc, TCSETS, &tty);
    }

    bool SetNumBits(termios &tty, const PortConfig &pc)
    {
        tty.c_cflag     &=  ~CSIZE;			// CSIZE is a mask for the number of bits per character
        switch(pc.m_numDataBits)
        {
        case NumDataBits::FIVE:
            tty.c_cflag     |=  CS5;
            break;
        case NumDataBits::SIX:
            tty.c_cflag     |=  CS6;
            break;
        case NumDataBits::SEVEN:
            tty.c_cflag     |=  CS7;
            break;
        case NumDataBits::EIGHT:
            tty.c_cflag     |=  CS8;
            break;
        default:
            return false;
        }
        return true;
    }

    bool SetParity(termios &tty, const PortConfig &pc)
    {
        switch(pc.m_parity)
        {
        case Parity::NONE:
            tty.c_cflag     &=  ~PARENB;
            break;
        case Parity::EVEN:
            tty.c_cflag 	|=   PARENB;
            tty.c_cflag		&=	 ~PARODD; // Clearing PARODD makes the parity even
            break;
        case Parity::ODD:
            tty.c_cflag     |=   PARENB;
            tty.c_cflag		|=	 PARODD;
            break;
        default:
            return false;
        }
        return true;
    }

    bool SetStopBits(termios &tty, const PortConfig &pc)
    {
        switch(pc.m_numStopBits)
        {
        case NumStopBits::ONE:
            tty.c_cflag     &=  ~CSTOPB;
            break;
        case NumStopBits::TWO:
            tty.c_cflag     |=  CSTOPB;
            break;
        default:
            return false;
        }
        return true;
    }

    bool SetHwControl(termios &tty, const PortConfig &pc)
    {
        // Configure flow control
        switch(pc.m_hardwareFlowControl)
        {
        case HardwareFlowControl::OFF:
            tty.c_cflag &= ~CRTSCTS;
            break;

        case HardwareFlowControl::ON: // Hardware flow control (RTS/CTS)
            tty.c_cflag |= CRTSCTS;
            break;

        default:
            return false;
        }

        tty.c_cflag     |=  CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
        return true;
    }

    bool SetBaudRate(termios &tty, const PortConfig &pc)
    {
        tty.c_cflag |= CBAUD;
        switch(pc.m_baudRate)
        {
            case BaudRate::B_0:
                 cfsetispeed(&tty, B0);
                 cfsetospeed(&tty, B0);
                break;
            case BaudRate::B_50:
                 cfsetispeed(&tty, B50);
                 cfsetospeed(&tty, B50);
                break;
            case BaudRate::B_75:
                 cfsetispeed(&tty, B75);
                 cfsetospeed(&tty, B75);
                break;
            case BaudRate::B_110:
                 cfsetispeed(&tty, B110);
                 cfsetospeed(&tty, B110);
                break;
            case BaudRate::B_134:
                 cfsetispeed(&tty, B134);
                 cfsetospeed(&tty, B134);
                break;
            case BaudRate::B_150:
                 cfsetispeed(&tty, B150);
                 cfsetospeed(&tty, B150);
                break;
            case BaudRate::B_200:
                 cfsetispeed(&tty, B200);
                 cfsetospeed(&tty, B200);
                break;
            case BaudRate::B_300:
                 cfsetispeed(&tty, B300);
                 cfsetospeed(&tty, B300);
                break;
            case BaudRate::B_600:
                 cfsetispeed(&tty, B600);
                 cfsetospeed(&tty, B600);
                break;
            case BaudRate::B_1200:
                 cfsetispeed(&tty, B1200);
                 cfsetospeed(&tty, B1200);
                break;
            case BaudRate::B_1800:
                 cfsetispeed(&tty, B1800);
                 cfsetospeed(&tty, B1800);
                break;
            case BaudRate::B_2400:
                 cfsetispeed(&tty, B2400);
                 cfsetospeed(&tty, B2400);
                break;
            case BaudRate::B_4800:
                 cfsetispeed(&tty, B4800);
                 cfsetospeed(&tty, B4800);
                break;
            case BaudRate::B_9600:
                 cfsetispeed(&tty, B9600);
                 cfsetospeed(&tty, B9600);
                break;
            case BaudRate::B_19200:
                 cfsetispeed(&tty, B19200);
                 cfsetospeed(&tty, B19200);
                break;
            case BaudRate::B_38400:
                 cfsetispeed(&tty, B38400);
                 cfsetospeed(&tty, B38400);
                break;
            case BaudRate::B_57600:
                 cfsetispeed(&tty, B57600);
                 cfsetospeed(&tty, B57600);
                break;
            case BaudRate::B_115200:
                 cfsetispeed(&tty, B115200);
                 cfsetospeed(&tty, B115200);
                break;
            case BaudRate::B_230400:
                 cfsetispeed(&tty, B230400);
                 cfsetospeed(&tty, B230400);
                break;
            case BaudRate::B_460800:
                 cfsetispeed(&tty, B460800);
                 cfsetospeed(&tty, B460800);
                break;
            default:
                return false;
        }
        return true;
    }

    bool SetTimeout(termios &tty, const PortConfig &pc)
    {
        //================= CONTROL CHARACTERS (.c_cc[]) ==================//

        // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
        // Only meaningful when port is set to non-canonical mode
        // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
        // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
        // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
        // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME
        //                      after first character has elapsed
        // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
        // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode

        if (pc.m_timeout == -1)
        {
            // Always wait for at least one byte, this could
            // block indefinitely
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 1;
        }
        else if(pc.m_timeout == 0)
        {
            // Setting both to 0 will give a non-blocking read
            tty.c_cc[VTIME] = 0;
            tty.c_cc[VMIN] = 0;
        }
        else if(pc.m_timeout > 0)
        {
            tty.c_cc[VTIME] = (cc_t)(pc.m_timeout / 100);    // 0.5 seconds read timeout
            tty.c_cc[VMIN] = 0;
        }
        return true;
    }

    bool SetSwControl(termios &tty, const PortConfig &pc)
    {
        switch(pc.m_softwareFlowControl)
        {
        case SoftwareFlowControl::OFF:
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            break;

        case SoftwareFlowControl::ON:
            tty.c_iflag |= (IXON | IXOFF | IXANY);
            break;
        default:
            return false;
        }

        tty.c_iflag 	&= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        return true;
    }

    bool SerialPort::ConfigurePort(PortConfig pc)
    {
        termios tty = GetTermios();
        if (!SetNumBits(tty, pc)) return false;
        if (!SetParity(tty, pc)) return false;
        if (!SetStopBits(tty, pc)) return false;
        if (!SetHwControl(tty, pc)) return false;
        if (!SetBaudRate(tty, pc)) return false;

        //===================== (.c_oflag) =================//

        tty.c_oflag     =   0;              // No remapping, no delays
        tty.c_oflag     &=  ~OPOST;         // Make raw

        if (!SetTimeout(tty, pc)) return false;
        if (!SetSwControl(tty, pc)) return false;
        //=========================== LOCAL MODES (c_lflag) =======================//

        // Canonical input is when read waits for EOL or EOF characters before returning. In non-canonical mode, the rate at which
        // read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
        tty.c_lflag		&= ~ICANON;    // Turn off canonical input, which is suitable for pass-through
        tty.c_lflag     &= ~(ECHO);
        tty.c_lflag		&= ~ECHOE;     // Turn off echo erase (echo erase only relevant if canonical input is active)
        tty.c_lflag		&= ~ECHONL;    //
        tty.c_lflag		&= ~ISIG;      // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters


        SetTermios(tty);
        return true;
    }

    int SerialPort::Write(const std::string& data)
    {
        if (m_fileDesc == -1)
            return -1;

        int writeResult = write(m_fileDesc, data.c_str(), data.size());
        return writeResult;
    }

    int SerialPort::WriteBinary(const std::vector<uint8_t>& data)
    {
        if (m_fileDesc == -1)
            return -1;

        int writeResult = write(m_fileDesc, data.data(), data.size());
        return writeResult;
    }

    int SerialPort::Read(std::string& data)
    {
        data.clear();

        if (m_fileDesc == -1)
            return -1;

        ssize_t n = read(m_fileDesc, m_readBuffer.data(), m_readBuffer.capacity());

        // Error Handling
        if(n < 0)
            return -1;

        if(n > 0)
            data = std::string(m_readBuffer.data(), n);
        return n;
    }

    int SerialPort::ReadBinary(std::vector<uint8_t>& data)
    {
        data.clear();

        if (m_fileDesc == -1)
            return -1;

        ssize_t n = read(m_fileDesc, m_readBuffer.data(), m_readBuffer.capacity());

        // Error Handling
        if(n < 0)
            return -1;

        if(n > 0)
            copy(m_readBuffer.begin(), m_readBuffer.begin() + n, back_inserter(data));

        return n;
    }
}
