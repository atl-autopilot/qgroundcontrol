#pragma once

#if defined(__androidx86__)
#include "m4serial.h"
#include "m4util.h"
#endif

#include <functional>
#include <vector>


// In order to use M4Lib, this interface needs to be implemented
// and injected when constructing M4Lib.
class TimerInterface {
public:
    virtual void start(int time_ms) = 0;
    virtual void stop() = 0;
    virtual void setCallback(std::function<void()> callback) = 0;
    virtual ~TimerInterface() = default;
};

// In order to use M4Lib, this interface needs to be implemented
// and injected when constructing M4Lib.
class HelperInterface {
public:
    virtual void msleep(int duration_ms) = 0;
    virtual void logDebug(std::string text) = 0;
    virtual void logInfo(std::string text) = 0;
    virtual void logWarn(std::string text) = 0;
    virtual void logError(std::string text) = 0;
    virtual ~HelperInterface() = default;
};


class M4Lib
{
public:

    enum class M4State {
        NONE           = 0,
        AWAIT          = 1,
        BIND           = 2,
        CALIBRATION    = 3,
        SETUP          = 4,
        RUN            = 5,
        SIM            = 6,
        FACTORY_CAL    = 7
    };

    /* Structure to save binding information.
    *  The default values were dump from ST16S, except reseted panId,nodeId,txAddr.
    */
    struct RxBindInfo {
        enum class Type {
            NUL    = -1,
            SR12S  = 0,
            SR12E  = 1,
            SR24S  = 2,
            RX24   = 3,
            SR19P  = 4,
        };
        int mode = 107; // To store Type.
        int panId = 0;
        int nodeId = 0;
        int aNum = 10;
        int aBit = 12;
        int trNum = 0;
        int trBit = 0;
        int swNum = 2;
        int swBit = 2;
        int monitNum = 0;
        int monitBit = 12;
        int extraNum = 0;
        int extraBit = 16;
        int txAddr = 0;
        std::vector<uint8_t> achName {1,2,3,4,5,6,7,8,9,10};
        std::vector<uint8_t> trName {};
        std::vector<uint8_t> swName {11, 102};
        std::vector<uint8_t> monitName {};
        std::vector<uint8_t> extraName {};
    };

    struct ControllerLocation {
        /**
         * Longitude in degrees [-180..180]
         */
        double longitude = 0.0;
        /**
         * Latitude in degrees [-90..90]
         */
        double latitude = 0.0;
        /**
         * Altitude of remote-controller above mean sea level (AMSL) in meters.
         */
        double altitude = 0.0;
        /**
         * The number of satellite used for navigation solution.
         */
        int satelliteCount = 0;

        /**
         * Position dilution of precision of navigation solution (unitless).
         */
        float pdop = 0.0f;

        /**
         * Speed of remote-controller (uBlox ground speed in meters/second).
         */
        float speed = 0.0f;

        /**
         * Heading of movement of remote-controller in degrees [-180..180].
         */
        float heading = 0.0f;
    };

    enum class SwitchId {
        FLIGHT_MODE,
        OBSTACLE_AVOIDANCE
    };

    enum class SwitchState {
        OFF,
        CENTER,
        ON
    };

    enum class ButtonId {
        POWER,
        AUX,
        CAMERA_SHUTTER,
        VIDEO_SHUTTER,
    };
    enum class ButtonState {
        NORMAL,
        PRESSED
    };

    enum class TrimState
    {
        INCREASE_PRESSED,
        DECREASE_PRESSED,
        RELEASED
    };

    enum class InitChannelState {
        NONE,
        INPROGRESS,
        FINISH
    };

    void init();
    void deinit();

    // This callback needs to send the mavlink RX_PAIR command.
    // Note that it is important that this command gets retransmitted in case of
    // timeout. Otherwise, the pairing can fail for the case where the command does
    // not arrive on the autopilot side.
    void setPairCommandCallback(std::function<void()> callback);
    void setSwitchStateChangedCallback(std::function<void(SwitchId, SwitchState)> callback);
    void setButtonStateChangedCallback(std::function<void(ButtonId, ButtonState)> callback);
    void setTrimStateChangedCallback(std::function<void(int, TrimState)> callback);
    void setRcActiveChangedCallback(std::function<void()> callback);
    void setCalibrationCompleteChangedCallback(std::function<void()> callback);
    void setCalibrationStateChangedCallback(std::function<void()> callback);
    void setRawChannelsChangedCallback(std::function<void()> callback);
    void setMixedChannelsChangedCallback(std::function<void()> callback);
    void setControllerLocationChangedCallback(std::function<void()> callback);
    void setM4StateChangedCallback(std::function<void()> callback);
    void setSaveSettingsCallback(std::function<void(const RxBindInfo& rxBindInfo, int rxBindInfoCrc)> callback);
    void setSettings(const RxBindInfo& rxBindInfo, int storageCrc);
    void setVersionCallback(std::function<void(int, int, int)> callback);

    void tryRead();

    M4State getM4State();

    bool getRcActive();
    void setRcActive(bool rcActive);

    bool getRcCalibrationComplete();

    void setVehicleConnected(bool vehicleConnected);

    std::vector<uint16_t> getRawChannels();
    std::vector<uint16_t> getMixedChannels();

    const ControllerLocation& getControllerLocation();

    // TODO: Check if we really don't need this.
    //       If possible we don't want to leak this information.
    //bool getSoftReboot() { return _softReboot; }

    void resetBind();
    void enterBindMode(bool skipPairCommand = false);

    void enterSimulation ();           //disable zigbee send rc
    void exitSimulation  ();           //enable zigbee send rc

    bool isVehicleReady();
    void checkVehicleReady();
    void tryStartCalibration();
    void tryStopCalibration();

    void softReboot();

    std::string m4StateStr();

    bool setPowerKey(int function);
    int calChannel(int index);

#if defined(__androidx86__)
    // These need to be ifdefd, otherwise we get linking errors.
    M4Lib(
        TimerInterface& timer,
        HelperInterface& helper);
    ~M4Lib();

private:
    void _bytesReady(std::vector<uint8_t> data);
    void _initSequence();
    void _stateManager();

    bool _write(std::vector<uint8_t> data, bool debug);
    void _tryEnterBindMode();
    bool _exitToAwait();
    bool _enterRun();
    bool _enterSimulation();
    bool _exitSimulation();
    bool _exitRun();
    bool _enterBind();
    bool _enterFactoryCalibration();
    bool _exitFactoryCalibration();
    bool _sendRecvBothCh();
    bool _sendRecvRawCh();
    bool _exitBind();
    bool _startBind();
    bool _bind(int rxAddr);
    bool _setChannelSetting();
    bool _unbind();
    bool _queryBindState();
    bool _syncMixingDataDeleteAll();
    bool _syncMixingDataAdd();
    bool _sendTableDeviceLocalInfo(TableDeviceLocalInfo_t localInfo);
    bool _sendTableDeviceChannelInfo(TableDeviceChannelInfo_t channelInfo);
    bool _sendTableDeviceChannelNumInfo(ChannelNumType_t channelNumType);
    bool _sendRxResInfo();

    bool _sendPassthroughMessage(std::vector<uint8_t> message);

    bool _generateTableDeviceChannelNumInfo(TableDeviceChannelNumInfo_t* channelNumInfo, ChannelNumType_t channelNumType, unsigned int& num);
    bool _fillTableDeviceChannelNumMap       (TableDeviceChannelNumInfo_t *channelNumInfo, unsigned int num, std::vector<uint8_t> list);
    void _generateTableDeviceLocalInfo       (TableDeviceLocalInfo_t *localInfo);
    bool _generateTableDeviceChannelInfo     (TableDeviceChannelInfo_t *channelInfo);

    void _handleBindResponse                 ();
    void _handleQueryBindResponse            (std::vector<uint8_t> data);
    bool _handleNonTypePacket                (m4Packet& packet);
    void _handleRxBindInfo                   (m4Packet& packet);
    void _handleChannel                      (m4Packet& packet);
    bool _handleCommand                      (m4Packet& packet);
    void _switchChanged                      (m4Packet& packet);
    void _calibrationStateChanged            (m4Packet& packet);
    void _handleMixedChannelData             (m4Packet& packet);
    void _handleRawChannelData               (m4Packet& packet);
    void _handleControllerFeedback           (m4Packet& packet);
    void _handlePassThroughPacket            (m4Packet& packet);
    std::string _getRxBindInfoFeedbackName   ();
    bool _tryGetVersion                      ();
    void _resetBindedId                      ();
    uint8_t _calculateBindInfoCrc            (const RxBindInfo& rxBindInfo);

    static  int     _byteArrayToInt  (std::vector<uint8_t> data, unsigned int offset, bool isBigEndian = false);
    static  short   _byteArrayToShort(std::vector<uint8_t> data, unsigned int offset, bool isBigEndian = false);

    M4SerialComm* _commPort;

    TimerInterface& _timer;
    HelperInterface& _helper;

    enum class InternalM4State {
        NONE,
        ENTER_BIND_ERROR,
        GET_VERSION,
        EXIT_RUN,
        ENTER_BIND,
        START_BIND,
        UNBIND,
        BIND,
        QUERY_BIND,
        EXIT_BIND,
        SET_CHANNEL_SETTINGS,
        MIX_CHANNEL_DELETE,
        MIX_CHANNEL_ADD,
        RECV_BOTH_CH,
        RECV_RAW_CH_ONLY,
        SEND_RX_INFO,
        ENTER_RUN,
        ENTER_SIMULATION,
        EXIT_SIMULATION,
        RUNNING_SIMULATION,
        ENTER_FACTORY_CAL,
        EXIT_FACTORY_CAL,
        RUNNING_FACTORY_CAL,
        RUNNING
    };

    std::function<void()>   _pairCommandCallback = nullptr;
    std::function<void(SwitchId, SwitchState)> _switchStateChangedCallback = nullptr;
    std::function<void(ButtonId, ButtonState)> _buttonStateChangedCallback = nullptr;
    std::function<void(int, TrimState)> _trimStateChangedCallback = nullptr;
    std::function<void()> _rcActiveChangedCallback = nullptr;
    std::function<void()> _calibrationCompleteChangedCallback = nullptr;
    std::function<void()> _calibrationStateChangedCallback = nullptr;
    std::function<void()> _rawChannelsChangedCallback = nullptr;
    std::function<void()> _mixedChannelsChangedCallback = nullptr;
    std::function<void()> _controllerLocationChangedCallback = nullptr;
    std::function<void()> _m4StateChangedCallback = nullptr;
    std::function<void(const RxBindInfo&, int rxBindInfoCrc)> _saveSettingsCallback = nullptr;
    // A version of -1.-1.-1 means the request timed out.
    std::function<void(int, int, int)> _versionCallback = nullptr;
    int                     _responseTryCount;
    M4State                 _m4State;
    M4State                 _m4IntentState;
    InternalM4State         _internalM4State;
    uint8_t                 _channelNumIndex;
    RxBindInfo              _rxBindInfoFeedback;
    int                     _currentChannelAdd;
    uint8_t                 _rxLocalIndex;
    uint8_t                 _rxchannelInfoIndex;
    bool                    _sendRxInfoEnd;
    bool                    _softReboot;
    bool                    _rcActive;
    uint16_t                _rawChannelsCalibration[CalibrationHwIndexMax];
    bool                    _rcCalibrationComplete;
    bool                    _vehicleConnected;
    bool                    _simulationMode;
    std::vector<uint16_t>   _rawChannels;
    std::vector<uint16_t>   _mixedChannels;
    ControllerLocation      _controllerLocation;
    int                     _m4Version;
    InitChannelState        _initChannelMappingState;

#ifdef DISABLE_ZIGBEE
    bool                    _skipBind;
#endif

#endif // defined(__androidx86__)
};

