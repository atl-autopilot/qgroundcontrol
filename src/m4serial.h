/**
 * @file m4serial.h
 *
 * Originally based on SerialComm.h of qgroundcontrol.
 *
 * @author Gus Grubba <mavlink@grubba.com>
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

// Only to be compiled on ST16 hardware (Android x86)
#if defined(__androidx86__)

#include <functional>
#include <vector>
#include <string>
#include <atomic>

class HelperInterface;

class M4SerialComm{
public:
    M4SerialComm  (HelperInterface& helper);
    ~M4SerialComm ();
    bool        init    (std::string port, int baud);
    bool        open    ();
    void        close   ();
    bool        write   (std::vector<uint8_t> data, bool debug = false);
    bool        write   (void* data, int length);
    void        tryRead ();
    void        setBytesReadyCallback(std::function<void(std::vector<uint8_t>)> callback);

private:
    int         _openPort       (const char* port);
    int         _writePort      (void *buffer, int len);
    bool        _setupPort      (int baud);
    void        _readPacket     (uint8_t length);
    bool        _readData       (void *buffer, int len);

private:
    enum class PacketState {
        NONE,
        FIRST_ID,
        SECOND_ID
    };

    HelperInterface& _helper;
    std::atomic<bool> _shouldExit {false};
    int _fd {-1};
    int _baudrate {230400};
    std::string _uart_name {};
    PacketState _currentPacketStatus {PacketState::NONE};
    std::function<void(std::vector<uint8_t>)> _bytesReadyCallback {nullptr};
};

#endif
