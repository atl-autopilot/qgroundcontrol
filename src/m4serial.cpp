/**
 * @file SerialComm.cc
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "m4serial.h"
#include "m4util.h"
#include "m4lib.h"

#if defined(__androidx86__)
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>

#include <cstring>
#include <sstream>


//-----------------------------------------------------------------------------
M4SerialComm::M4SerialComm(HelperInterface& helper)
    : _helper(helper)
{

}

//-----------------------------------------------------------------------------
M4SerialComm::~M4SerialComm()
{
    close();
}

//-----------------------------------------------------------------------------
bool
M4SerialComm::init(std::string port, int baud)
{
    _uart_name = port;
    _baudrate  = baud;
    return true;
}

//-----------------------------------------------------------------------------
bool
M4SerialComm::open()
{
    if(_serialPortStatus != SerialPortState::CLOSED || _fd >= 0) {
        return false;
    }
    _fd = _openPort(_uart_name.c_str());
    if(_fd < 0) {
        _helper.logWarn("SERIAL: Could not open port" + _uart_name);
        return false;
    }
    if(!_setupPort(_baudrate)) {
        if (_fd >= 0) {
            ::close(_fd);
        }
        _fd = -1;
        return false;
    }
    _serialPortStatus = SerialPortState::OPEN;
    return true;
}

//-----------------------------------------------------------------------------
void
M4SerialComm::close()
{
    _serialPortStatus = SerialPortState::CLOSED;
    if(_fd >= 0) {
        ::close(_fd);
    }
    //if(!wait(1000)) {
    //    _helper.logDebug("SERIAL: Timeout waiting for thread to end");
    //}
    _fd = -1;
}

//-----------------------------------------------------------------------------
bool M4SerialComm::write(std::vector<uint8_t> data, bool debug)
{
    if(debug) {
        // TODO: fix
        //_helper.logDebug(data.toHex());
    }
    return _writePort(data.data(), data.size()) == int(data.size());
}

//-----------------------------------------------------------------------------
bool M4SerialComm::write(void* data, int length)
{
    return _writePort(data, length) == length;
}

//-----------------------------------------------------------------------------
void
M4SerialComm::tryRead()
{
    if(_serialPortStatus == SerialPortState::OPEN) {
        uint8_t b;
        if(::read(_fd, &b, 1) == 1) {
            switch (_currentPacketStatus) {
                case PacketState::NONE:
                    if(b == 0x55) {
                        _currentPacketStatus = PacketState::FIRST_ID;
                    }
                    break;
                case PacketState::FIRST_ID:
                    if(b == 0x55) {
                        _currentPacketStatus = PacketState::SECOND_ID;
                    } else {
                        _currentPacketStatus = PacketState::NONE;
                    }
                    break;
                case PacketState::SECOND_ID:
                    _readPacket(b);
                    _currentPacketStatus = PacketState::NONE;
                    break;
            }
        }
    }
}

//-----------------------------------------------------------------------------
void
M4SerialComm::setBytesReadyCallback(std::function<void(std::vector<uint8_t>)> callback)
{
    _bytesReadyCallback = callback;
}

//-----------------------------------------------------------------------------
bool
M4SerialComm::_readData(void *buffer, int len)
{
    int tries = 0;
    int left  = len;
    uint8_t* ptr = reinterpret_cast<uint8_t*>(buffer);
    while(left > 0) {
        int count = ::read(_fd, ptr, left);
        if(count < 0 || _serialPortStatus != SerialPortState::OPEN || _fd < 0) {
            return false;
        }
        left -= count;
        ptr  += count;
        if(++tries > 5) {
            return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------
void
M4SerialComm::_readPacket(uint8_t length)
{
    if(length > 1) {
        length--; //-- Skip CRC from count
        uint8_t buffer[260];
        if(_readData(buffer, length)) {
            //-- CRC is appended to end of data block
            uint8_t iCRC;
            if(::read(_fd, &iCRC, 1) == 1) {
                uint8_t oCRC = crc8(buffer, length);
                if(iCRC == oCRC) {
                    std::vector<uint8_t> data(buffer, buffer + length);
                    if (_bytesReadyCallback) {
                        _bytesReadyCallback(data);
                    }
                } else {
                    std::stringstream ss;
                    ss << "Bad CRC" << length << iCRC << oCRC;
                    _helper.logDebug(ss.str());
                }
            } else {
                _helper.logDebug("Missed CRC");
            }
        } else {
            _helper.logDebug("Missed message payload");
        }
    }
}

//-----------------------------------------------------------------------------
int
M4SerialComm::_openPort(const char* port)
{
    int fd = ::open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd >= 0) {
        fcntl(fd, F_SETFL, 0);
    }
    return fd;
}

//-----------------------------------------------------------------------------
bool
M4SerialComm::_setupPort(int baud)
{
    struct termios2 config;
    std::memset(&config, 0, sizeof(config));
    if (ioctl(_fd, TCGETS2, &config) < 0) {
        std::stringstream ss;
        ss << "Could not get termios2: " << strerror(errno);
        _helper.logError(ss.str());
        return false;
    }
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG | TOSTOP);
    config.c_cflag &= ~(CSIZE | PARENB | CBAUD | CRTSCTS);
    config.c_cflag |= CS8 | BOTHER;
    config.c_cc[VMIN]   = 1; // We want at least 1 byte to be available.
    config.c_cc[VTIME]  = 0; // We don't timeout but wait indefinitely.
    config.c_ispeed = baud;
    config.c_ospeed = baud;
    if (ioctl(_fd, TCSETS2, &config) == -1) {
        std::stringstream ss;
        ss << "Could not set terminal attributes: " << strerror(errno);
        _helper.logError(ss.str());
        return false;
    }
    if (ioctl(_fd, TCFLSH, TCIOFLUSH) == -1) {
        std::stringstream ss;
        ss << "Could not flush terminal: " << strerror(errno);
        _helper.logError(ss.str());
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
int
M4SerialComm::_writePort(void* buffer, int len)
{
    int written = ::write(_fd, buffer, len);
    if(written != len && written >= 0) {
        std::stringstream ss;
        ss << "SERIAL: Wrote only " << written << " bytes out of " << len << " bytes";
        _helper.logWarn(ss.str());
    }
    return written;
}

#endif // defined(__androidx86__)
