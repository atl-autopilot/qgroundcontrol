#include "m4lib.h"

#include "m4util.h"
#include "m4def.h"
#include "m4serial.h"

#if defined(__androidx86__)

#include <cstring>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>

// RC Channel data provided by Yuneec
#include "m4channeldata.h"

static const char* kUartName        = "/dev/ttyMFD0";

#define UNUSED(x_) (void)(x_)

#define DEBUG_DATA_DUMP             false
#define COMMAND_WAIT_INTERVAL       250
#define SEND_INTERVAL               60
#define COMMAND_RESPONSE_TRIES      4
#define VERSION_WAIT_INTERVAL       250
#define GET_VERSION_TRIES           40 // At 4 Hz that's 10 seconds.
#define SCAN_INTERVAL               4000
/*
 * Original source was ported from:
 * DroneFly/droneservice/src/main/java/com/yuneec/droneservice/parse/St16Controller.java
 * by Gus Grubba <mavlink@grubba.com>
 *
 * All comments within the command send functions came from the original file above.
 * The functions themselves have been completely rewriten from scratch.
 *
 * Then the source was refactored into this M4Lib class.
 * The state before can be accessed viewed at:
 * https://github.com/YUNEEC/qgroundcontrol/blob/\
 * 90cfa2408dc605b0cc36a083c91c3346949a8e31/custom/src/TyphoonHM4Interface.cc
 * by Julian Oes <julian@oes.ch>
 */

static std::string
toHex(std::vector<uint8_t> data)
{
    std::stringstream ss;
    for (auto it = data.begin(); it != data.end(); ++it) {
        ss << " ";
        ss << std::setfill('0') << std::setw(2) << std::hex << int(*it);
    }
    return ss.str();
}


M4Lib::M4Lib(
        TimerInterface& timer,
        TimerInterface& versionTimer,
        HelperInterface& helper)
    : _timer(timer)
    , _versionTimer(versionTimer)
    , _helper(helper)
    , _responseTryCount(0)
    , _m4State(M4State::NONE)
    , _m4IntentState(M4State::AWAIT)
    , _internalM4State(InternalM4State::NONE)
    , _currentChannelAdd(0)
    , _rxLocalIndex(0)
    , _sendRxInfoEnd(false)
    , _rcActive(false)
    , _rcCalibrationComplete(true)
    , _vehicleConnected(false)
    , _slaveMode(false)
    , _initChannelState(InitChannelMappingState::NONE)
    #ifdef DISABLE_ZIGBEE
    , _skipBind(false)
    #endif
{
    _commPort = new M4SerialComm(_helper);
    _commPort->setBytesReadyCallback(std::bind(&M4Lib::_bytesReady, this, std::placeholders::_1));

    _rxchannelInfoIndex = 2;
    _channelNumIndex    = 6;

    _timer.setCallback(std::bind(&M4Lib::_stateManager, this));
    _versionTimer.setCallback(std::bind(&M4Lib::_versionTimeout, this));
    std::memset(_rawChannelsCalibration, 0, sizeof(_rawChannelsCalibration));

    //-- We don't use zigbee in some projects, so skip binding zigbee.
    #ifdef DISABLE_ZIGBEE
    _skipBind = true;
    #endif
}

M4Lib::~M4Lib()
{
    _timer.stop();
    _versionTimer.stop();
    if(_commPort) {
        _commPort->close();
        delete _commPort;
    }
}

M4Lib::M4State
M4Lib::getM4State()
{
    return _m4State;
}

bool
M4Lib::getRcActive()
{
    return _rcActive;
}

bool
M4Lib::getRcCalibrationComplete()
{
    return _rcCalibrationComplete;
}

void
M4Lib::setVehicleConnected(bool vehicleConnected)
{
    _vehicleConnected = vehicleConnected;
}

std::vector<uint16_t>
M4Lib::getRawChannels()
{
    return _rawChannels;
}

std::vector<uint16_t>
M4Lib::getMixedChannels()
{
    return _mixedChannels;
}

const M4Lib::ControllerLocation&
M4Lib::getControllerLocation()
{
    return _controllerLocation;
}

void
M4Lib::setRcActive(bool rcActive)
{
    if (_rcActive != rcActive) {
        _rcActive = rcActive;
        if (_rcActiveChangedCallback) {
            _rcActiveChangedCallback();
        }
    }

    if (rcActive) {
        //-- If we were in softReboot, we can exit now because we have received RC.
        if (_softReboot) {
            _softReboot = false;
        }
    }

    //-- This means there was a RC timeout while the vehicle was connected.
    if (!rcActive && _vehicleConnected && !_slaveMode) {
        //-- If we are in run state after binding and we don't have RC, bind it again.
        if (_vehicleConnected && _softReboot && _m4State == M4State::RUN) {
            _softReboot = false;
            _helper.logInfo("RC bind again");
            enterBindMode();
        }
    }
}

void
M4Lib::init()
{
#if defined(__androidx86__)
    if(!_commPort || !_commPort->init(kUartName, 230400) || !_commPort->open()) {
        //-- TODO: If this ever happens, we need to do something about it
        _helper.logWarn("Could not start serial communication with M4");
    }
#endif
}

void
M4Lib::deinit()
{
    //Nothing need to do.
}

void
M4Lib::setPairCommandCallback(std::function<void()> callback)
{
    _pairCommandCallback = callback;
}

void
M4Lib::setSwitchStateChangedCallback(std::function<void(SwitchId, SwitchState)> callback)
{
    _switchStateChangedCallback = callback;
}

void
M4Lib::setButtonStateChangedCallback(std::function<void(ButtonId, ButtonState)> callback)
{
    _buttonStateChangedCallback = callback;
}

void
M4Lib::setRcActiveChangedCallback(std::function<void()> callback)
{
    _rcActiveChangedCallback = callback;
}

void
M4Lib::setCalibrationCompleteChangedCallback(std::function<void()> callback)
{
    _calibrationCompleteChangedCallback = callback;
}

void
M4Lib::setCalibrationStateChangedCallback(std::function<void()> callback)
{
    _calibrationStateChangedCallback = callback;
}

void
M4Lib::setRawChannelsChangedCallback(std::function<void()> callback)
{
    _rawChannelsChangedCallback = callback;
}

void
M4Lib::setMixedChannelsChangedCallback(std::function<void()> callback)
{
    _mixedChannelsChangedCallback = callback;
}

void
M4Lib::setVersionCallback(std::function<void(int, int, int)> callback)
{
    _versionCallback = callback;
}

bool
M4Lib::getVersion()
{
    if (_getVersionState == GetVersionState::GETTING_VERSION) {
        _helper.logWarn("already trying to get version.");
    }

    if (!_versionCallback) {
        _helper.logWarn("callback to get version not set.");
    }

    _tryGetVersionCount = 0;
    _getVersionState = GetVersionState::GETTING_VERSION;
    _versionTimer.start(VERSION_WAIT_INTERVAL);
    ++_tryGetVersionCount;
    return _tryGetVersion();
}

bool
M4Lib::_tryGetVersion()
{
    m4Command getVersionCmd(Yuneec::CMD_GET_M4_VERSION);
    std::vector<uint8_t> cmd = getVersionCmd.pack();
    return _write(cmd, true);
}

void
M4Lib::setControllerLocationChangedCallback(std::function<void()> callback)
{
    _controllerLocationChangedCallback = callback;
}

void
M4Lib::setM4StateChangedCallback(std::function<void()> callback)
{
    _m4StateChangedCallback = callback;
}

void
M4Lib::setSaveSettingsCallback(std::function<void(const RxBindInfo& rxBindInfo)> callback)
{
    _saveSettingsCallback = callback;
}

void
M4Lib::setSettings(const RxBindInfo& rxBindInfo)
{
    _rxBindInfoFeedback = rxBindInfo;
    _sendRxInfoEnd = false;
}

bool
M4Lib::_write(std::vector<uint8_t> data, bool debug)
{
    return _commPort->write(data, debug);
}

void
M4Lib::tryRead()
{
    return _commPort->tryRead();
}

void
M4Lib::resetBind()
{
    _rxBindInfoFeedback = {};
    _m4IntentState = M4State::AWAIT;
    _exitToAwait();
}

void
M4Lib::enterSlaveMode()
{
    _slaveMode = true;
//If we disable zigbee, we don't need to do anything for entering slave mode.
#ifndef DISABLE_ZIGBEE
    //If we would to make m4 enter simulate state to enable mixed channel,
    //we can set "_m4IntentState" to "M4State::SIM".
    //_m4IntentState = M4State::SIM;
    _m4IntentState = M4State::RUN;
    _exitToAwait();
#endif
}

void
M4Lib::exitSlaveMode()
{
    _slaveMode = false;
#ifndef DISABLE_ZIGBEE
    _m4IntentState = M4State::AWAIT;
    _exitToAwait();
#endif
}

void
M4Lib::enterBindMode(bool skipPairCommand)
{
    if(_slaveMode) {
        _helper.logWarn("enterBindMode() called while in slave mode.");
        return;
    }
    std::stringstream ss;
    ss << "enterBindMode() Current Mode: " << int(_m4State);

//-- TODO:skip binding zigbee
#ifdef DISABLE_ZIGBEE
    if(_skipBind) {
       _helper.logDebug("Binding is not required, skip");
       return;
    }
#endif

    _helper.logInfo(ss.str());
    if(!skipPairCommand) {
        if (!_pairCommandCallback) {
            _helper.logWarn("pairCommandCallback not set.");
            return;
        }

        _pairCommandCallback();
    }
    _tryEnterBindMode();
}

void
M4Lib::_tryEnterBindMode()
{
    //-- Set M4 into bind mode
    _rxBindInfoFeedback = {};
    //Before entering bind mode, we need to exit to await state, then enter to bind mode in _initSequence().
    _m4IntentState = M4State::BIND;
    _exitToAwait();
}

bool
M4Lib::isVehicleReady()
{
    // This is a helper function to determine if we need to run isVehicleRead().
    return (_m4State == M4State::RUN && _rcActive);
}

void
M4Lib::checkVehicleReady()
{
    if(_m4State == M4State::RUN && !_rcActive) {
        _helper.logInfo("In RUN mode but no RC yet");
        // TODO: check this, it seems the delay is not needed.
        //QTimer::singleShot(2000, this, &M4Lib::_initAndCheckBinding);
        _initAndCheckBinding();
    } else {
        if(_m4State != M4State::RUN) {
            //-- The M4 is not initialized
            _helper.logInfo("M4 not yet initialized");
            _initAndCheckBinding();
        }
    }
}

void
M4Lib::tryStartCalibration()
{
    //-- Ignore it if already in factory cal mode
    if(_m4State != M4State::FACTORY_CAL) {
        std::memset(_rawChannelsCalibration, 0, sizeof(_rawChannelsCalibration));
        _rcCalibrationComplete = false;
        if (_calibrationCompleteChangedCallback) {
            _calibrationCompleteChangedCallback();
        }
        if (_calibrationStateChangedCallback) {
            _calibrationStateChangedCallback();
        }
        _m4IntentState = M4State::FACTORY_CAL;
        _exitToAwait();
    }
}

void
M4Lib::tryStopCalibration()
{
    if(_m4State == M4State::FACTORY_CAL) {
        _m4IntentState = M4State::AWAIT;
        _exitToAwait();
    }
}


void
M4Lib::softReboot()
{
#if defined(__androidx86__)
    _helper.logInfo("softReboot()");
    if(_rcActive) {
        _helper.logDebug("softReboot() -> Already bound. Skipping it...");
    } else {
        deinit();
        _helper.msleep(SEND_INTERVAL);
        _internalM4State    = InternalM4State::NONE;
        _responseTryCount   = 0;
        _currentChannelAdd  = 0;
        _m4State            = M4State::NONE;
        _m4IntentState      = M4State::RUN;
        _rxLocalIndex       = 0;
        _sendRxInfoEnd      = false;
        _rxchannelInfoIndex = 2;
        _channelNumIndex    = 6;
        _initChannelState   = InitChannelMappingState::NONE;
        _helper.msleep(SEND_INTERVAL);
        init();
    }
    // We want to recheck if we are really bound, so we set RC inactive and wait to
    // to get notified about RC being active again.
    setRcActive(false);
    _softReboot = true;
#endif
}

/**
 * Exit to await state.
 */
bool
M4Lib::_exitToAwait()
{
    bool result = false;
    std::stringstream ss;
    ss << "Exit to await state, Current State:" << m4StateStr() << "(" << int(_m4State) << ")";
    _helper.logDebug(ss.str());
    switch (_m4State) {
        case M4State::BIND:
            _internalM4State = InternalM4State::EXIT_BIND;
            result = _exitBind();
            _responseTryCount = 0;
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case M4State::RUN:
            _internalM4State = InternalM4State::EXIT_RUN;
            result = _exitRun();
            _responseTryCount = 0;
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case M4State::FACTORY_CAL:
            _internalM4State = InternalM4State::EXIT_FACTORY_CAL;
            result = _exitFactoryCalibration();
            _responseTryCount = 0;
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case M4State::SIM:
            _internalM4State = InternalM4State::EXIT_SIMULATION;
            result = _exitSimulation();
            _responseTryCount = 0;
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case M4State::AWAIT:
            _initSequence();
            result = true;
            break;
        default:
            result = false;
    }
    return result;
}

/**
 * This command is used for entering the progress of binding aircraft.
 * This command is the first step of the progress of binging aircraft.
 * The next command you will send may be {@link _startBind}.
 */
bool
M4Lib::_enterRun()
{
    _helper.logDebug("Sending: CMD_ENTER_RUN");
    m4Command enterRunCmd(Yuneec::CMD_ENTER_RUN);
    std::vector<uint8_t> cmd = enterRunCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

bool
M4Lib::_enterSimulation()
{
    _helper.logDebug("Sending: CMD_ENTER_SIMULATOR");
    m4Command enterRunCmd(Yuneec::CMD_ENTER_SIMULATOR);
    std::vector<uint8_t> cmd = enterRunCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for stopping control aircraft.
 */
bool
M4Lib::_exitRun()
{
    _helper.logDebug("Sending: CMD_EXIT_RUN");
    m4Command exitRunCmd(Yuneec::CMD_EXIT_RUN);
    std::vector<uint8_t> cmd = exitRunCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

bool
M4Lib::_exitSimulation()
{
    _helper.logDebug("Sending: CMD_EXIT_SIMULATOR");
    m4Command exitRunCmd(Yuneec::CMD_EXIT_SIMULATOR);
    std::vector<uint8_t> cmd = exitRunCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for entering the progress of binding aircraft.
 * This command is the first step of the progress of binging aircraft.
 * The next command you will send may be {@link _startBind}.
 */
bool
M4Lib::_enterBind()
{
    _helper.logDebug("Sending: CMD_ENTER_BIND");
    m4Command enterBindCmd(Yuneec::CMD_ENTER_BIND);
    std::vector<uint8_t> cmd = enterBindCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for calibrating joysticks and knobs of M4.
 * Generally, it is using by factory when the board of st16 was producted at first.
 */
bool
M4Lib::_enterFactoryCalibration()
{
    _helper.logDebug("Sending: CMD_ENTER_FACTORY_CAL");
    m4Command enterFactoryCaliCmd(Yuneec::CMD_ENTER_FACTORY_CAL);
    std::vector<uint8_t> cmd = enterFactoryCaliCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for exit calibration.
 */
bool
M4Lib::_exitFactoryCalibration()
{
    _helper.logDebug("Sending: CMD_EXIT_FACTORY_CALI");
    m4Command exitFacoryCaliCmd(Yuneec::CMD_EXIT_FACTORY_CAL);
    std::vector<uint8_t> cmd = exitFacoryCaliCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * Use this command to set the type of channel receive original hardware signal values and encoding values.
 */
//-- TODO: Do we really need raw data? Maybe CMD_RECV_MIXED_CH_ONLY would be enough.
bool
M4Lib::_sendRecvBothCh()
{
    _helper.logDebug("Sending: CMD_RECV_BOTH_CH");
    m4Command enterRecvCmd(Yuneec::CMD_RECV_BOTH_CH);
    std::vector<uint8_t> cmd = enterRecvCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

bool
M4Lib::_sendRecvRawCh()
{
    _helper.logDebug("Sending: CMD_RECV_RAW_CH_ONLY");
    m4Command enterRecvCmd(Yuneec::CMD_RECV_RAW_CH_ONLY);
    std::vector<uint8_t> cmd = enterRecvCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for exiting the progress of binding.
 */
bool
M4Lib::_exitBind()
{
    _helper.logDebug("Sending: CMD_EXIT_BIND");
    m4Command exitBindCmd(Yuneec::CMD_EXIT_BIND);
    std::vector<uint8_t> cmd = exitBindCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * After {@link _enterBind} response rightly, send this command to get a list of aircraft which can be bind.
 * The next command you will send may be {@link _bind}.
 */
bool
M4Lib::_startBind()
{
    _helper.logDebug("Sending: CMD_START_BIND");
    m4Message startBindMsg(Yuneec::CMD_START_BIND, Yuneec::TYPE_BIND);
    std::vector<uint8_t> msg = startBindMsg.pack();
    return _write(msg, DEBUG_DATA_DUMP);
}

/**
 * Use this command to bind specified aircraft.
 * After {@link _startBind} response rightly, you get a list of aircraft which can be bound.
 * Then you send this command and {@link _queryBindState} repeatedly several times until get a right
 * response from {@link _queryBindState}. If not bind successful after sending commands several times,
 * the progress of bind exits.
 */
bool
M4Lib::_bind(int rxAddr)
{
    _helper.logDebug("Sending: CMD_BIND");
    m4Message bindMsg(Yuneec::CMD_BIND, Yuneec::TYPE_BIND);
    bindMsg.data[4] = uint8_t(rxAddr & 0xff);
    bindMsg.data[5] = uint8_t((rxAddr & 0xff00) >> 8);
    bindMsg.data[6] = 5; //-- Gotta love magic numbers
    bindMsg.data[7] = 15;
    std::vector<uint8_t> msg = bindMsg.pack();
    return _write(msg, DEBUG_DATA_DUMP);
}

/**
 * This command is used for setting the number of analog channel and switch channel.
 * After binding aircraft successful, you need to send this command.
 * Analog channel represent which controller has a smooth step changed value, like a rocker.
 * Switch channel represent which controller has two or three state of value, like flight mode switcher.
 * The next command you will send may be {@link _syncMixingDataDeleteAll}.
 */
bool
M4Lib::_setChannelSetting()
{
    _helper.logDebug("Sending: CMD_SET_CHANNEL_SETTING");
    m4Command setChannelSettingCmd(Yuneec::CMD_SET_CHANNEL_SETTING);
    std::vector<uint8_t> payload;
    payload.resize(2);
    std::fill(payload.begin(), payload.end(), 0);
    payload[0] = uint8_t(_rxBindInfoFeedback.aNum  & 0xff);
    payload[1] = uint8_t(_rxBindInfoFeedback.swNum & 0xff);
    std::vector<uint8_t> cmd = setChannelSettingCmd.pack(payload);
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for setting if power key is working.
 * Parameter of {@link #_setPowerKey(int)}, represent the function
 * of power key is working. For example, when you click power key, the screen will light up or
 * go out if set the value {@link BaseCommand#BIND_KEY_FUNCTION_PWR}.
 */
bool
M4Lib::setPowerKey(int function)
{
    _helper.logDebug("Sending: CMD_SET_BINDKEY_FUNCTION");
    m4Command setPowerKeyCmd(Yuneec::CMD_SET_BINDKEY_FUNCTION);
    std::vector<uint8_t> payload;
    payload.resize(1);
    payload[0] = uint8_t(function & 0xff);
    std::vector<uint8_t> cmd = setPowerKeyCmd.pack(payload);
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for disconnecting the bound aircraft.
 * Suggest to use this command before using {@link _bind} first time.
 */
bool
M4Lib::_unbind()
{
    _helper.logDebug("Sending: CMD_UNBIND");
    m4Command unbindCmd(Yuneec::CMD_UNBIND);
    std::vector<uint8_t> cmd = unbindCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for querying the state of whether bind was succeed.
 * This command always be sent follow {@link _bind} with a transient time.
 * The next command you will send may be {@link _setChannelSetting}.
 */
bool
M4Lib::_queryBindState()
{
    _helper.logDebug("Sending: CMD_QUERY_BIND_STATE");
    m4Command queryBindStateCmd(Yuneec::CMD_QUERY_BIND_STATE);
    std::vector<uint8_t> cmd = queryBindStateCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for deleting all channel formula data synchronously.
 * See {@link _syncMixingDataAdd}.
 */
bool
M4Lib::_syncMixingDataDeleteAll()
{
    _helper.logDebug("Sending: CMD_SYNC_MIXING_DATA_DELETE_ALL");
    m4Command syncMixingDataDeleteAllCmd(Yuneec::CMD_SYNC_MIXING_DATA_DELETE_ALL);
    std::vector<uint8_t> cmd = syncMixingDataDeleteAllCmd.pack();
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This command is used for adding channel formula data synchronously.
 * You need to send all the channel formula data successful,
 * if not, send {@link _syncMixingDataDeleteAll} again.
 * Channel formula make the same hardware signal values to the different values we get finally. (What?)
 */
bool
M4Lib::_syncMixingDataAdd()
{
    /*
     *  "Mixing Data" is an array of NUM_CHANNELS (25) sets of CHANNEL_LENGTH (96)
     *  magic bytes each. Each set is sent using this command. The documentation states
     *  that if there is an error you should send the CMD_SYNC_MIXING_DATA_DELETE_ALL
     *  command again and start over.
     *  I have not seen a way to identify an error other than getting no response once
     *  the command is sent. There doesn't appear to be a "NAK" type response.
     */
    _helper.logDebug("Sending: CMD_SYNC_MIXING_DATA_ADD");
    m4Command syncMixingDataAddCmd(Yuneec::CMD_SYNC_MIXING_DATA_ADD);
    std::vector<uint8_t> payload(reinterpret_cast<const char*>(&channel_data[_currentChannelAdd]),
                                 reinterpret_cast<const char*>(&channel_data[_currentChannelAdd]) + CHANNEL_LENGTH);
    std::vector<uint8_t> cmd = syncMixingDataAddCmd.pack(payload);
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This funtion is used for sending the Local information to aircraft
 * Local information such as structure TableDeviceLocalInfo_t
 */
bool
M4Lib::_sendTableDeviceLocalInfo(TableDeviceLocalInfo_t localInfo)
{
    m4Command sendRxResInfoCmd(Yuneec::CMD_SEND_RX_RESINFO);
    std::vector<uint8_t> payload;
    unsigned int len = 11;
    payload.resize(len);
    std::fill(payload.begin(), payload.end(), 0);
    payload[0]  = localInfo.index;
    payload[1]  = uint8_t(localInfo.mode     & 0xff);
    payload[2]  = uint8_t((localInfo.mode    & 0xff00) >> 8);
    payload[3]  = uint8_t(localInfo.nodeId   & 0xff);
    payload[4]  = uint8_t((localInfo.nodeId  & 0xff00) >> 8);
    payload[5]  = localInfo.parseIndex;
    payload[7]  = uint8_t(localInfo.panId    & 0xff);
    payload[8]  = uint8_t((localInfo.panId   & 0xff00) >> 8);
    payload[9]  = uint8_t(localInfo.txAddr   & 0xff);
    payload[10] = uint8_t((localInfo.txAddr  & 0xff00) >> 8);
    std::vector<uint8_t> cmd = sendRxResInfoCmd.pack(payload);
#if DEBUG_DATA_DUMP==true
    std::stringstream ss;
    ss << "_sendTableDeviceLocalInfo" << toHex(cmd);
    _helper.logDebug(ss.str());
#endif
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This funtion is used for sending the Channel information to aircraft
 * Channel information such as structure TableDeviceChannelInfo_t
 */
bool
M4Lib::_sendTableDeviceChannelInfo(TableDeviceChannelInfo_t channelInfo)
{
    m4Command sendRxResInfoCmd(Yuneec::CMD_SEND_RX_RESINFO);
    std::vector<uint8_t> payload;
    unsigned int len = sizeof(channelInfo);
    payload.resize(len);
    std::fill(payload.begin(), payload.end(), 0);
    payload[0]  = channelInfo.index;
    payload[1]  = channelInfo.aNum;
    payload[2]  = channelInfo.aBits;
    payload[3]  = channelInfo.trNum;
    payload[4]  = channelInfo.trBits;
    payload[5]  = channelInfo.swNum;
    payload[6]  = channelInfo.swBits;
    payload[7]  = channelInfo.replyChannelNum;
    payload[8]  = channelInfo.replyChannelBits;
    payload[9]  = channelInfo.requestChannelNum;
    payload[10] = channelInfo.requestChannelBits;
    payload[11] = channelInfo.extraNum;
    payload[12] = channelInfo.extraBits;
    payload[13] = channelInfo.analogType;
    payload[14] = channelInfo.trimType;
    payload[15] = channelInfo.switchType;
    payload[16] = channelInfo.replyChannelType;
    payload[17] = channelInfo.requestChannelType;
    payload[18] = channelInfo.extraType;
    std::vector<uint8_t> cmd = sendRxResInfoCmd.pack(payload);

#if DEBUG_DATA_DUMP==true
    std::stringstream ss;
    ss << "_sendTableDeviceChannelInfo" << toHex(cmd);
    _helper.logDebug(ss.str());
#endif
    return _write(cmd, DEBUG_DATA_DUMP);
}

/**
 * This funtion is used for sending the Channel number information to aircraft
 * Channel information such as structure TableDeviceChannelNumInfo_t
 * This feature is distributed according to enum ChannelNumType_t
 */
bool
M4Lib::_sendTableDeviceChannelNumInfo(ChannelNumType_t channelNumType)
{
    TableDeviceChannelNumInfo_t channelNumInfo;
    std::memset(&channelNumInfo, 0, sizeof(TableDeviceChannelNumInfo_t));
    unsigned int num =  0;
    if(_generateTableDeviceChannelNumInfo(&channelNumInfo, channelNumType, num)) {
        m4Command sendRxResInfoCmd(Yuneec::CMD_SEND_RX_RESINFO);
        std::vector<uint8_t> payload;
        unsigned int len = num + 1;
        payload.resize(len);
        std::fill(payload.begin(), payload.end(), 0);
        payload[0] = channelNumInfo.index;
        for(unsigned int i = 0; i < num; i++) {
            payload[i + 1] = channelNumInfo.channelMap[i];
        }
        std::vector<uint8_t> cmd = sendRxResInfoCmd.pack(payload);
#if DEBUG_DATA_DUMP==true
        std::stringstream ss;
        ss << "channelNumType" << toHex(cmd);
        _helper.logDebug(ss.str());
#endif
        return _write(cmd, DEBUG_DATA_DUMP);
    }
    return true;
}

/**
 * This feature is based on different types of fill information
 *
 */
bool
M4Lib::_generateTableDeviceChannelNumInfo(TableDeviceChannelNumInfo_t* channelNumInfo, ChannelNumType_t channelNumType, unsigned int& num)
{
    switch(channelNumType) {
        case ChannelNumAanlog:
            num = static_cast<unsigned int>(_rxBindInfoFeedback.aNum);
            if(!_fillTableDeviceChannelNumMap(channelNumInfo, num, _rxBindInfoFeedback.achName)) {
                return false;
            }
            break;
        case ChannelNumTrim:
            num = static_cast<unsigned int>(_rxBindInfoFeedback.trNum);
            if(!_fillTableDeviceChannelNumMap(channelNumInfo, num, _rxBindInfoFeedback.trName)) {
                return false;
            }
            break;
        case ChannelNumSwitch:
            num = static_cast<unsigned int>(_rxBindInfoFeedback.swNum);
            if(!_fillTableDeviceChannelNumMap(channelNumInfo, num, _rxBindInfoFeedback.swName)) {
                return false;
            }
            break;
        case ChannelNumMonitor:
            num = static_cast<unsigned int>(_rxBindInfoFeedback.monitNum);
            if(num <= 0) {
                return false;
            }
            if(!_fillTableDeviceChannelNumMap(channelNumInfo, num, _rxBindInfoFeedback.monitName)) {
                return false;
            }
            break;
        case ChannelNumExtra:
            num = static_cast<unsigned int>(_rxBindInfoFeedback.extraNum);
            if(num <= 0) {
                return false;
            }
            if(!_fillTableDeviceChannelNumMap(channelNumInfo, num, _rxBindInfoFeedback.extraName)) {
                return false;
            }
            break;
        default:
            return false;
    }
    return true;
}

/**
 * This funtion is used for filling TableDeviceChannelNumInfo_t with the channel number information
 * information from RxBindInfo
 */
bool
M4Lib::_fillTableDeviceChannelNumMap(TableDeviceChannelNumInfo_t* channelNumInfo, unsigned int num, std::vector<uint8_t> list)
{
    bool res = false;
    if(num) {
        if(num <= list.size()) {
            channelNumInfo->index = _channelNumIndex;
            for(unsigned int i = 0; i < num; i++) {
                channelNumInfo->channelMap[i] = uint8_t(list[i]);
            }
            res = true;
        } else {
            std::stringstream ss;
            ss << "_fillTableDeviceChannelNumMap() called with mismatching list size. Num =" << num << "List =" << list.size();
            _helper.logError(ss.str());
        }
    }
    _channelNumIndex++;
    return res;
}

/**
 * This function is used to initialize the M4, it must be called at AWAIT state.
 * Actually, all configs should be set only at AWAIT state, if we want to change
 * the config at other states, we should exit to AWAIT state, then apply the new config.
 */
void
M4Lib::_initSequence()
{
    //We need to config channel mapping first, and it only need to be set once.
    if (_initChannelState == InitChannelMappingState::NONE) {
        _initChannelState = InitChannelMappingState::INPROGRESS;
        _helper.logWarn("M4 hasn't initialized.");
        //-- Initialize M4 channel mapping table.
        _responseTryCount = 0;
        _internalM4State = InternalM4State::SET_CHANNEL_SETTINGS;
        _setChannelSetting();
        _timer.start(COMMAND_WAIT_INTERVAL);
        return;
    }else if (_initChannelState == InitChannelMappingState::INPROGRESS){
        _helper.logWarn("M4 is initialing.");
        return;
    }

    switch (_m4IntentState) {
    case M4State::AWAIT:
        _helper.logDebug("Intent to await(idle) state.");
        break;
    case M4State::RUN:
        _helper.logDebug("Intent to RUN state.");
        _responseTryCount = 0;
        if (_internalM4State == InternalM4State::SEND_RX_INFO) {
            _internalM4State = InternalM4State::ENTER_RUN;
            _enterRun();
        }else {
            _internalM4State = InternalM4State::RECV_BOTH_CH;
            _sendRecvBothCh();
        }
        _timer.start(COMMAND_WAIT_INTERVAL);
        break;
    case M4State::FACTORY_CAL:
    case M4State::SIM:
        _responseTryCount = 0;
        _internalM4State = InternalM4State::RECV_BOTH_CH;
        _sendRecvBothCh();
        _timer.start(COMMAND_WAIT_INTERVAL);
        break;
    case M4State::BIND:
        _helper.logDebug("Intent to BIND state.");
        if (_rxBindInfoFeedback.nodeId) {
            _m4IntentState = M4State::RUN;
            _initSequence();
            return;
        }else {
            //If no specific zigbee, then we will scan available zigbee.
            _responseTryCount = 0;
            _internalM4State = InternalM4State::ENTER_BIND;
            _enterBind();
            _timer.start(COMMAND_WAIT_INTERVAL);
        }
        break;
    default:
        break;
    }
}

/*
 * This handles the sequence of events/commands sent to the MCU. A following
 * command is only sent once we receive an response from the previous one. If
 * no response is received, the command is sent again.
 *
 */
void
M4Lib::_stateManager()
{
    switch(_internalM4State) {
        case InternalM4State::EXIT_RUN:
            _helper.logInfo("STATE_EXIT_RUN Timeout");
            if(_responseTryCount > COMMAND_RESPONSE_TRIES || _m4State == M4State::AWAIT) {
                _helper.logWarn("Too many STATE_EXIT_RUN Timeouts or it had been AWAIT state.");
                _initSequence();
            } else {
                _exitRun();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::ENTER_BIND:
            _helper.logInfo("STATE_ENTER_BIND Timeout");
            if(_responseTryCount > COMMAND_RESPONSE_TRIES) {
                _helper.logWarn("Too many STATE_ENTER_BIND Timeouts.");
                if(_rxBindInfoFeedback.nodeId) {
                    _responseTryCount = 0;
                    _internalM4State = InternalM4State::SEND_RX_INFO;
                    _sendRxResInfo();
                    _timer.start(COMMAND_WAIT_INTERVAL);
                } else {
                    //-- We're stuck. Wen can't enter bind and we have no binding info.
                    _helper.logError("Cannot enter binding mode");
                    _internalM4State = InternalM4State::ENTER_BIND_ERROR;
                }
            } else {
                if(_slaveMode) {
                    _internalM4State = InternalM4State::NONE;
                } else {
                    _enterBind();
                    _timer.start(COMMAND_WAIT_INTERVAL);
                    _responseTryCount++;
                }
            }
            break;
        case InternalM4State::START_BIND:
            _helper.logInfo("STATE_START_BIND Timeout");
            if(_responseTryCount > COMMAND_RESPONSE_TRIES) {
                _helper.logError("Too many STATE_START_BIND Timeouts. Giving up...");

                _internalM4State = InternalM4State::EXIT_BIND;
                _exitBind();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount = 0;
            } else {
                _startBind();
                //-- Wait a bit longer as there may not be anyone listening
                _timer.start(SCAN_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::UNBIND:
            _helper.logInfo("STATE_UNBIND Timeout");
            if(_responseTryCount > COMMAND_RESPONSE_TRIES) {
                _helper.logWarn("Too many STATE_UNBIND Timeouts.");
                _initSequence();
            } else {
                _unbind();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::BIND:
            _helper.logInfo("STATE_BIND Timeout");
            _bind(_rxBindInfoFeedback.nodeId);
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::QUERY_BIND:
            _helper.logInfo("STATE_QUERY_BIND Timeout");
            _queryBindState();
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::EXIT_BIND:
            _helper.logInfo("STATE_EXIT_BIND Timeout");
            _exitBind();
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::SET_CHANNEL_SETTINGS:
            _helper.logInfo("STATE_SET_CHANNEL_SETTINGS Timeout");
            _setChannelSetting();
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::MIX_CHANNEL_DELETE:
            _helper.logInfo("STATE_MIX_CHANNEL_DELETE Timeout");
            _syncMixingDataDeleteAll();
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::MIX_CHANNEL_ADD:
            _helper.logInfo("STATE_MIX_CHANNEL_ADD Timeout");
            //-- We need to delete and send again
            _internalM4State = InternalM4State::MIX_CHANNEL_DELETE;
            _syncMixingDataDeleteAll();
            _timer.start(COMMAND_WAIT_INTERVAL);
            //-- TODO: This can't wait for ever...
            break;
        case InternalM4State::RECV_BOTH_CH:
            _helper.logInfo("CMD_RECV_BOTH_CH Timeout");
            _sendRecvBothCh();
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case InternalM4State::RECV_RAW_CH_ONLY:
            _helper.logInfo("CMD_RECV_RAW_CH_ONLY Timeout");
            _sendRecvRawCh();
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case InternalM4State::SEND_RX_INFO:
            _helper.logInfo("STATE_SEND_RX_INFO Timeout");
            _sendRxResInfo();
            _timer.start(COMMAND_WAIT_INTERVAL);
            break;
        case InternalM4State::ENTER_RUN:
            if(_responseTryCount > COMMAND_RESPONSE_TRIES || _m4State == M4State::RUN) {
                _helper.logWarn("Too many STATE_ENTER_RUN Timeouts or it had been RUN state.");
            } else {
                _helper.logInfo("STATE_ENTER_RUN Timeout");
                _enterRun();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::ENTER_SIMULATION:
            if(_responseTryCount > COMMAND_RESPONSE_TRIES || _m4State == M4State::BIND) {
                _helper.logWarn("Too many STATE_ENTER_SIMULATION Timeouts or it had been BIND state.");
            } else {
                _helper.logInfo("STATE_ENTER_SIMULATION Timeout");
                _enterSimulation();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::EXIT_SIMULATION:
            if(_responseTryCount > COMMAND_RESPONSE_TRIES || _m4State == M4State::SIM) {
                _helper.logWarn("Too many STATE_EXIT_SIMULATION Timeouts or it had been SIM state.");
                _initSequence();
            } else {
                _helper.logInfo("STATE_EXIT_SIMULATION Timeout");
                _exitSimulation();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        case InternalM4State::EXIT_FACTORY_CAL:
            if(_responseTryCount > COMMAND_RESPONSE_TRIES || _m4State == M4State::FACTORY_CAL) {
                _helper.logWarn("Too many EXIT_FACTORY_CAL Timeouts or it had been FACTORY_CAL state.");
                _initSequence();
            } else {
                _helper.logInfo("EXIT_FACTORY_CAL Timeout");
                _exitFactoryCalibration();
                _timer.start(COMMAND_WAIT_INTERVAL);
                _responseTryCount++;
            }
            break;
        default:
            std::stringstream ss;
            ss << "Timeout:" << static_cast<int>(_internalM4State);
            _helper.logDebug(ss.str()) ;
            break;
    }
}

void
M4Lib::_versionTimeout()
{
    switch (_getVersionState) {
        case GetVersionState::NONE:
            break;
        case GetVersionState::GETTING_VERSION:
            if (_tryGetVersionCount > GET_VERSION_TRIES) {
                _helper.logWarn("Too many GET_VERSION tries. Giving up...");
                _getVersionState = GetVersionState::NONE;
                if (_versionCallback) {
                    _versionCallback(-1, -1, -1);
                }
            } else if (_tryGetVersionCount == 10 ||
                       _tryGetVersionCount == 20 ||
                       _tryGetVersionCount == 30) {
                // Sometimes in the beginning, we just can't get the version, unless we
                // stop it all by doing exitRun.
                _helper.logError("We are trying to do exitRun to get the version");
                _versionTimer.start(COMMAND_WAIT_INTERVAL);
                _exitRun();
                ++_tryGetVersionCount;

            } else {
                _helper.logInfo("GET_VERSION Timeout, trying again");
                _versionTimer.start(COMMAND_WAIT_INTERVAL);
                ++_tryGetVersionCount;
                _tryGetVersion();
            }
            break;
    }
}

/**
 * This command is used for sending the information we got from bound aircraft to the bound aircraft
 * to make a confirmation.
 * If send successful, the you can send {@link _enterRun}.
 */
bool
M4Lib::_sendRxResInfo()
{
    _sendRxInfoEnd = false;
    TableDeviceChannelInfo_t channelInfo ;
    std::memset(&channelInfo, 0, sizeof(TableDeviceChannelInfo_t));
    TableDeviceLocalInfo_t localInfo;
    std::memset(&localInfo, 0, sizeof(TableDeviceLocalInfo_t));
    if(!_generateTableDeviceChannelInfo(&channelInfo)) {
        return _sendRxInfoEnd;
    }
    if(!_sendTableDeviceChannelInfo(channelInfo)) {
        return _sendRxInfoEnd;
    }
    _helper.msleep(SEND_INTERVAL);
    _generateTableDeviceLocalInfo(&localInfo);
    if(!_sendTableDeviceLocalInfo(localInfo)) {
        return _sendRxInfoEnd;
    }
    _sendRxInfoEnd = true;
    return _sendRxInfoEnd;
}

/**
 * This funtion is used for filling TableDeviceLocalInfo_t with the Local information
 * information from RxBindInfo
 */
void
M4Lib::_generateTableDeviceLocalInfo(TableDeviceLocalInfo_t* localInfo)
{
    localInfo->index        = _rxLocalIndex;
    localInfo->mode         = static_cast<uint16_t>(_rxBindInfoFeedback.mode);
    localInfo->nodeId       = static_cast<uint16_t>(_rxBindInfoFeedback.nodeId);
    localInfo->parseIndex   = _rxchannelInfoIndex - 1;
    localInfo->panId        = static_cast<uint16_t>(_rxBindInfoFeedback.panId);
    localInfo->txAddr       = static_cast<uint16_t>(_rxBindInfoFeedback.txAddr);
    _rxLocalIndex++;
}

/**
 * This funtion is used for filling TableDeviceChannelInfo_t with the channel information
 * information from RxBindInfo
 */
bool
M4Lib::_generateTableDeviceChannelInfo(TableDeviceChannelInfo_t* channelInfo)
{
    channelInfo->index              = _rxchannelInfoIndex;
    channelInfo->aNum               = static_cast<uint8_t>(_rxBindInfoFeedback.aNum);
    channelInfo->aBits              = static_cast<uint8_t>(_rxBindInfoFeedback.aBit);
    channelInfo->trNum              = static_cast<uint8_t>(_rxBindInfoFeedback.trNum);
    channelInfo->trBits             = static_cast<uint8_t>(_rxBindInfoFeedback.trBit);
    channelInfo->swNum              = static_cast<uint8_t>(_rxBindInfoFeedback.swNum);
    channelInfo->swBits             = static_cast<uint8_t>(_rxBindInfoFeedback.swBit);
    channelInfo->replyChannelNum    = static_cast<uint8_t>(_rxBindInfoFeedback.monitNum);
    channelInfo->replyChannelBits   = static_cast<uint8_t>(_rxBindInfoFeedback.monitBit);
    channelInfo->requestChannelNum  = static_cast<uint8_t>(_rxBindInfoFeedback.monitNum);
    channelInfo->requestChannelBits = static_cast<uint8_t>(_rxBindInfoFeedback.monitBit);
    channelInfo->extraNum           = static_cast<uint8_t>(_rxBindInfoFeedback.extraNum);
    channelInfo->extraBits          = static_cast<uint8_t>(_rxBindInfoFeedback.extraBit);
    if(!_sendTableDeviceChannelNumInfo(ChannelNumAanlog)) {
        return false;
    }
    _helper.msleep(SEND_INTERVAL);
    channelInfo->analogType = _channelNumIndex - 1;
    if(!_sendTableDeviceChannelNumInfo(ChannelNumTrim)) {
        return false;
    }
    _helper.msleep(SEND_INTERVAL);
    channelInfo->trimType = _channelNumIndex - 1;
    if(!_sendTableDeviceChannelNumInfo(ChannelNumSwitch)) {
        return false;
    }
    _helper.msleep(SEND_INTERVAL);
    channelInfo->switchType = _channelNumIndex - 1;
    // generate reply channel map
    if(!_sendTableDeviceChannelNumInfo(ChannelNumMonitor)) {
        return false;
    }
    _helper.msleep(SEND_INTERVAL);
    channelInfo->replyChannelType   = _channelNumIndex - 1;
    channelInfo->requestChannelType = _channelNumIndex - 1;
    // generate extra channel map
    if(!_sendTableDeviceChannelNumInfo(ChannelNumExtra)) {
        return false;
    }
    channelInfo->extraType = _channelNumIndex - 1;
    _helper.msleep(SEND_INTERVAL);
    _rxchannelInfoIndex++;
    return true;
}

void
M4Lib::_initAndCheckBinding()
{
#if defined(__androidx86__)
    if(!_slaveMode) {
        //-- First boot, not bound
        if(_m4State != M4State::RUN) {
            enterBindMode();
        //-- RC is bound to something. Is it bound to whoever we are connected?
        } else if(!_rcActive) {
            enterBindMode();
        } else {
            _helper.logInfo("In RUN mode and RC ready");
        }
    }else {
       _m4IntentState = M4State::RUN;
       _exitToAwait();
    }
#endif
}

/**
 * This command is used for sending messages to aircraft pass through ZigBee.
 */
bool
M4Lib::_sendPassthroughMessage(std::vector<uint8_t> message)
{
    _helper.logDebug("Sending: pass through message");
    m4PassThroughCommand passThroughCommand;
    std::vector<uint8_t> cmd = passThroughCommand.pack(message);
    return _write(cmd, DEBUG_DATA_DUMP);
}

/*
 *
 * A full, validated data packet has been received. The data argument contains
 * only the data portion. The 0x55, 0x55 header and length (3 bytes) have been
 * removed as well as the trailing CRC (1 byte).
 *
 * Code largely based on original Java code found in the St16Controller class.
 * The main difference is we also handle the machine state here.
 */
void
M4Lib::_bytesReady(std::vector<uint8_t> data)
{
    m4Packet packet(data);
    int type = packet.type();
    //-- Some Yuneec voodoo
    type = (type & 0x1c) >> 2;
    if(_handleNonTypePacket(packet)) {
        return;
    }
    switch(type) {
        case Yuneec::TYPE_BIND:
            switch(uint8_t(data[3])) {
                case 2:
                    _handleRxBindInfo(packet);
                    break;
                case 4:
                    _handleBindResponse();
                    break;
                default:
                    _timer.stop();
                    std::stringstream ss;
                    ss << "Received: TYPE_BIND Unknown:" << toHex(data);
                    _helper.logDebug(ss.str());
                    break;
            }
            break;
        case Yuneec::TYPE_CHN:
            _handleChannel(packet);
            break;
        case Yuneec::TYPE_CMD:
            _handleCommand(packet);
            break;
        case Yuneec::TYPE_RSP:
            switch(packet.commandID()) {
                case Yuneec::CMD_QUERY_BIND_STATE:
                    //-- Response from _queryBindState()
                    _handleQueryBindResponse(data);
                    break;
                case Yuneec::CMD_EXIT_RUN:
                    //-- Response from _exitRun()
                    _helper.logDebug("Received TYPE_RSP: CMD_EXIT_RUN");
                    if(_internalM4State == InternalM4State::EXIT_RUN) {
                        //-- Now we start initsequence
                        _initSequence();
                    }
                    break;
                case Yuneec::CMD_EXIT_SIMULATOR:
                    //-- Response from _exitSimulation()
                    _helper.logDebug("Received TYPE_RSP: CMD_EXIT_SIMULATOR");
                    if(_internalM4State == InternalM4State::EXIT_SIMULATION) {
                        //-- Now we start initsequence
                        _initSequence();
                    }
                    break;
                case Yuneec::CMD_ENTER_BIND:
                    //-- Response from _enterBind()
                    _helper.logDebug("Received TYPE_RSP: CMD_ENTER_BIND");
                    if(_internalM4State == InternalM4State::ENTER_BIND) {
                        //-- Now we start scanning
                        _responseTryCount = 0;
                        _internalM4State = InternalM4State::START_BIND;
                        _startBind();
                        _timer.start(SCAN_INTERVAL);
                    }
                    break;
                case Yuneec::CMD_UNBIND:
                    _helper.logDebug("Received TYPE_RSP: CMD_UNBIND");
                    if(_internalM4State == InternalM4State::UNBIND) {
                        _timer.stop();
                    }
                    break;
                case Yuneec::CMD_EXIT_BIND:
                    //-- Response from _exitBind()
                    _helper.logDebug("Received TYPE_RSP: CMD_EXIT_BIND");
                    if(_internalM4State == InternalM4State::EXIT_BIND) {
                        _initSequence();
                    }
                    break;
                case Yuneec::CMD_RECV_BOTH_CH:
                case Yuneec::CMD_RECV_RAW_CH_ONLY:
                    {
                        switch (_m4IntentState) {
                        case M4State::RUN:
                            _responseTryCount = 0;
#ifdef DISABLE_ZIGBEE
                            if(_skipBind || _slaveMode) {
#else
                            if (_slaveMode){
#endif
                                _internalM4State = InternalM4State::ENTER_RUN;
                                _enterRun();
                            }else if(_rxBindInfoFeedback.nodeId) {
                                _internalM4State = InternalM4State::SEND_RX_INFO;
                                _sendRxResInfo();
                            }else {
                                _internalM4State = InternalM4State::ENTER_RUN;
                                _enterRun();
                            }
                            _timer.start(COMMAND_WAIT_INTERVAL);
                            break;
                        case M4State::FACTORY_CAL:
                            _responseTryCount = 0;
                            _internalM4State = InternalM4State::ENTER_FACTORY_CAL;
                            _enterFactoryCalibration();
                            _timer.start(COMMAND_WAIT_INTERVAL);
                            break;
                        case M4State::SIM:
                            _responseTryCount = 0;
                            _internalM4State = InternalM4State::ENTER_SIMULATION;
                            _enterSimulation();
                            _timer.start(COMMAND_WAIT_INTERVAL);
                            break;
                        default:
                            break;
                        }
                    }
                    break;
                case Yuneec::CMD_SET_CHANNEL_SETTING:
                    //-- Response from _setChannelSetting()
                    _helper.logDebug("Received TYPE_RSP: CMD_SET_CHANNEL_SETTING");
                    if(_internalM4State == InternalM4State::SET_CHANNEL_SETTINGS) {
                        _internalM4State = InternalM4State::MIX_CHANNEL_DELETE;
                        _syncMixingDataDeleteAll();
                        _timer.start(COMMAND_WAIT_INTERVAL);
                    }
                    break;
                case Yuneec::CMD_SYNC_MIXING_DATA_DELETE_ALL:
                    //-- Response from _syncMixingDataDeleteAll()
                    _helper.logDebug("Received TYPE_RSP: CMD_SYNC_MIXING_DATA_DELETE_ALL");
                    if(_internalM4State == InternalM4State::MIX_CHANNEL_DELETE) {
                        _internalM4State = InternalM4State::MIX_CHANNEL_ADD;
                        _currentChannelAdd = 0;
                        _syncMixingDataAdd();
                        _timer.start(COMMAND_WAIT_INTERVAL);
                    }
                    break;
                case Yuneec::CMD_SYNC_MIXING_DATA_ADD: {
                        //-- Response from _syncMixingDataAdd()
                        std::stringstream ss;
                        ss  << "Received TYPE_RSP: CMD_SYNC_MIXING_DATA_ADD" << _currentChannelAdd;
                        _helper.logDebug(ss.str());
                        if(_internalM4State == InternalM4State::MIX_CHANNEL_ADD) {
                            _currentChannelAdd++;
                            if(_currentChannelAdd < NUM_CHANNELS) {
                                _syncMixingDataAdd();
                                _timer.start(COMMAND_WAIT_INTERVAL);
                            }else {
                                _timer.stop();
                                _initChannelState = InitChannelMappingState::FINISH;
                                _initSequence();
                            }
                        }
                    }
                    break;
                case Yuneec::CMD_SEND_RX_RESINFO:
                    //-- Response from _sendRxResInfo()
                    if(_internalM4State == InternalM4State::SEND_RX_INFO && _sendRxInfoEnd) {
                        _helper.logDebug("Received TYPE_RSP: CMD_SEND_RX_RESINFO");
                        _initSequence();
                    }
                    break;
                case Yuneec::CMD_ENTER_RUN: {
                        //-- Response from _enterRun()
                        _helper.logDebug("Received TYPE_RSP: CMD_ENTER_RUN");
                        std::stringstream ss;
                        ss << "State: " << int(_internalM4State);
                        _helper.logDebug(ss.str());
                        if(_internalM4State == InternalM4State::ENTER_RUN) {
                            _internalM4State = InternalM4State::RUNNING;
                            _timer.stop();
                            _helper.logInfo("M4 ready, in run state.");
                        }
                    }
                    break;
                case Yuneec::CMD_ENTER_SIMULATOR: {
                        //-- Response from _enterSimulation()
                        _helper.logDebug("Received TYPE_RSP: CMD_ENTER_SIMULATOR");
                        std::stringstream ss;
                        ss << "State: " << int(_internalM4State);
                        _helper.logDebug(ss.str());
                        if(_internalM4State == InternalM4State::ENTER_SIMULATION) {
                            _internalM4State = InternalM4State::RUNNING_SIMULATION;
                            _timer.stop();
                            _helper.logInfo("M4 ready, in simulation state.");
                        }
                    }
                    break;
                case Yuneec::CMD_SET_BINDKEY_FUNCTION:
                    _helper.logDebug("Received TYPE_RSP: CMD_SET_BINDKEY_FUNCTION");
                    break;
                case Yuneec::CMD_ENTER_FACTORY_CAL: {
                        //-- Response from _enterSimulation()
                        _helper.logDebug("Received TYPE_RSP: CMD_ENTER_FACTORY_CAL");
                        std::stringstream ss;
                        ss << "State: " << int(_internalM4State);
                        _helper.logDebug(ss.str());
                        if(_internalM4State == InternalM4State::ENTER_FACTORY_CAL) {
                            _internalM4State = InternalM4State::RUNNING_FACTORY_CAL;
                            _timer.stop();
                            _helper.logInfo("M4 ready, in factory calibration state.");
                        }
                    }
                    break;
                case Yuneec::CMD_EXIT_FACTORY_CAL:
                    _helper.logDebug("Received TYPE_RSP: CMD_EXIT_FACTORY_CAL");
                    if(_internalM4State == InternalM4State::EXIT_FACTORY_CAL) {
                        //-- Now we start initsequence
                        _timer.stop();
                        _initSequence();
                    }
                    break;
                case Yuneec::CMD_GET_M4_VERSION:
                    {
                        std::stringstream ss;
                        ss << "Received TYPE_RSP: " << toHex(data);
                        _helper.logInfo(ss.str());

                        if (data.size() >= 6) {
                            ss.clear();
                            ss << "Received TYPE_RSP as string: "
                               << data.at(data.size()-6)
                               << data.at(data.size()-5)
                               << data.at(data.size()-4)
                               << data.at(data.size()-3)
                               << data.at(data.size()-2);
                            _helper.logInfo(ss.str());

                            std::stringstream str_major;
                            str_major << data.at(data.size()-6) << data.at(data.size()-5);

                            std::stringstream str_minor;
                            str_minor << data.at(data.size()-3) << data.at(data.size()-2);

                            int major = 0;
                            int minor = 0;

                            str_major >> major;
                            str_minor >> minor;

                            ss.clear();
                            ss << "Received M4 Version: " << major << "." << minor;
                            _helper.logInfo(ss.str());
                            _versionTimer.stop();
                            if (_getVersionState == GetVersionState::GETTING_VERSION) {
                                _getVersionState = GetVersionState::NONE;
                            }
                            if (_versionCallback != nullptr) {
                                _versionCallback(major, minor, 0);
                            } else {
                                _helper.logWarn("Version callback not set.");
                            }
                        } else {
                            _helper.logInfo("Received data has wrong format. "
                                "Cannot extract version information.");
                        }
                    }
                    break;
                default: {
                        std::stringstream ss;
                        ss << "Received TYPE_RSP: ???" << packet.commandID() << toHex(data);
                        _helper.logInfo(ss.str());
                    }
                    break;
            }
            break;
        case Yuneec::TYPE_PASS_THROUGH:
            //Received pass-through data.
            _helper.logInfo("Received TYPE_PASS_THROUGH (?)");
            _handlePassThroughPacket(packet);
            break;
        default: {
                std::stringstream ss;
                ss << "Received: Unknown Packet" << type << toHex(data);
                _helper.logWarn(ss.str());
            }
            break;
    }
}

void
M4Lib::_handleQueryBindResponse(std::vector<uint8_t> data)
{
    int nodeID = (data[10] & 0xff) | (data[11] << 8 & 0xff00);
    {
        std::stringstream ss;
        ss << "Received TYPE_RSP: CMD_QUERY_BIND_STATE" << nodeID << " -- " << _getRxBindInfoFeedbackName();
        _helper.logDebug(ss.str());
    }
    if(_internalM4State == InternalM4State::QUERY_BIND) {
        if(nodeID == _rxBindInfoFeedback.nodeId) {
            _timer.stop();
            std::stringstream ss;
            ss << "Switched to BOUND state with:" << _getRxBindInfoFeedbackName();
            _helper.logDebug(ss.str());
            _internalM4State = InternalM4State::EXIT_BIND;
            _exitBind();
            if (_saveSettingsCallback) {
                _saveSettingsCallback(_rxBindInfoFeedback);
            }
            _timer.start(COMMAND_WAIT_INTERVAL);
        } else {
            std::stringstream ss;
            ss << "Response CMD_QUERY_BIND_STATE from unkown origin:" << nodeID;
            _helper.logWarn(ss.str());
        }
    }
}

bool
M4Lib::_handleNonTypePacket(m4Packet& packet)
{
    int commandId = packet.commandID();
    switch(commandId) {
        case Yuneec::COMMAND_M4_SEND_GPS_DATA_TO_PA:
            _handleControllerFeedback(packet);
            return true;
    }
    return false;
}

void
M4Lib::_handleBindResponse()
{
    _helper.logDebug("Received TYPE_BIND: BIND Response");
    if(_internalM4State == InternalM4State::BIND) {
        _timer.stop();
        _internalM4State = InternalM4State::QUERY_BIND;
        _queryBindState();
        _timer.start(COMMAND_WAIT_INTERVAL);
    }
}

void
M4Lib::_handleRxBindInfo(m4Packet& packet)
{
    //-- TODO: If for some reason this is done where two or more Typhoons are in
    //   binding mode, we will be receiving multiple responses. No check for this
    //   situation is done below.
    /*
     * Based on original Java code as below:
     *
        private void handleRxBindInfo(byte[] data) {
            RxBindInfo rxBindInfoFeedback = new RxBindInfo();
            rxBindInfoFeedback.mode = (data[6] & 0xff) | (data[7] << 8 & 0xff00);
            rxBindInfoFeedback.panId = (data[8] & 0xff) | (data[9] << 8 & 0xff00);
            rxBindInfoFeedback.nodeId = (data[10] & 0xff) | (data[11] << 8 & 0xff00);
            rxBindInfoFeedback.aNum = data[20];
            rxBindInfoFeedback.aBit = data[21];
            rxBindInfoFeedback.swNum = data[24];
            rxBindInfoFeedback.swBit = data[25];
            int p = data.length - 2;
            rxBindInfoFeedback.txAddr = (data[p] & 0xff) | (data[p + 1] << 8 & 0xff00);//data[12]~data[19]
            ControllerStateManager manager = ControllerStateManager.getInstance();
            if (manager != null) {
                manager.onRecvBindInfo(rxBindInfoFeedback);
            }
        }
     *
     */
    _helper.logDebug("Received: TYPE_BIND with rxBindInfoFeedback");
    if(_internalM4State == InternalM4State::START_BIND) {

        //std::stringstream ss;
        //ss << dumpDataPacket(packet.data);
        //_helper.logDebug(ss.str());
        _timer.stop();
        _rxBindInfoFeedback.mode     = (uint8_t(packet.data[6])  & 0xff) | (uint8_t(packet.data[7])  << 8 & 0xff00);
        _rxBindInfoFeedback.panId    = (uint8_t(packet.data[8])  & 0xff) | (uint8_t(packet.data[9])  << 8 & 0xff00);
        _rxBindInfoFeedback.nodeId   = (uint8_t(packet.data[10]) & 0xff) | (uint8_t(packet.data[11]) << 8 & 0xff00);
        _rxBindInfoFeedback.aNum     = uint8_t(packet.data[20]);
        _rxBindInfoFeedback.aBit     = uint8_t(packet.data[21]);
        _rxBindInfoFeedback.trNum    = uint8_t(packet.data[22]);
        _rxBindInfoFeedback.trBit    = uint8_t(packet.data[23]);
        _rxBindInfoFeedback.swNum    = uint8_t(packet.data[24]);
        _rxBindInfoFeedback.swBit    = uint8_t(packet.data[25]);
        _rxBindInfoFeedback.monitNum = uint8_t(packet.data[26]);
        _rxBindInfoFeedback.monitBit = uint8_t(packet.data[27]);
        _rxBindInfoFeedback.extraNum = uint8_t(packet.data[28]);
        _rxBindInfoFeedback.extraBit = uint8_t(packet.data[29]);
        unsigned int ilen = 30;
        _rxBindInfoFeedback.achName.clear();
        for(int i = 0; i < _rxBindInfoFeedback.aNum ; i++) {
            _rxBindInfoFeedback.achName.push_back(uint8_t(packet.data[ilen++]));
        }
        _rxBindInfoFeedback.trName.clear();
        for(int i = 0; i < _rxBindInfoFeedback.trNum ; i++) {
            _rxBindInfoFeedback.trName.push_back(uint8_t(packet.data[ilen++]));
        }
        _rxBindInfoFeedback.swName.clear();
        for(int i = 0; i < _rxBindInfoFeedback.swNum ; i++) {
            _rxBindInfoFeedback.swName.push_back(uint8_t(packet.data[ilen++]));
        }
        _rxBindInfoFeedback.monitName.clear();
        for(int i = 0; i < _rxBindInfoFeedback.monitNum ; i++) {
            _rxBindInfoFeedback.monitName.push_back(uint8_t(packet.data[ilen++]));
        }
        _rxBindInfoFeedback.extraName.clear();
        for(int i = 0; i < _rxBindInfoFeedback.extraNum ; i++) {
            _rxBindInfoFeedback.extraName.push_back(uint8_t(packet.data[ilen++]));
        }
        unsigned int p = packet.data.size() - 2;
        _rxBindInfoFeedback.txAddr = (uint8_t(packet.data[p]) & 0xff) | (uint8_t(packet.data[p + 1]) << 8 & 0xff00);
        std::stringstream ss;
        ss << "RxBindInfo:" << _getRxBindInfoFeedbackName() << _rxBindInfoFeedback.nodeId;
        _helper.logDebug(ss.str());
        _internalM4State = InternalM4State::BIND;
        _bind(_rxBindInfoFeedback.nodeId);
        _timer.start(COMMAND_WAIT_INTERVAL);
    } else {
        _helper.logDebug("RxBindInfo discarded (out of sequence)");
    }
}

void
M4Lib::_handleChannel(m4Packet& packet)
{
    switch(packet.commandID()) {
        case Yuneec::CMD_RX_FEEDBACK_DATA:
            _helper.logDebug("Received TYPE_CHN: CMD_RX_FEEDBACK_DATA");
            /* From original Java code
             *
             * We're not going to ever receive this unless the Typhoon is running
             * the factory firmware.
             *
            if (droneFeedbackListener == null) {
                return;
            }
            handleDroneFeedback(packet);
            */
            break;
        case Yuneec::CMD_TX_CHANNEL_DATA_MIXED:
//            _helper.logDebug("CMD_TX_CHANNEL_DATA_MIXED");
            _handleMixedChannelData(packet);
            break;
        case Yuneec::CMD_TX_CHANNEL_DATA_RAW:
//            _helper.logDebug("CMD_TX_CHANNEL_DATA_RAW");
            _handleRawChannelData(packet);
            break;
        case 0x82:
            //-- COMMAND_M4_SEND_COMPRESS_TRIM_TO_PAD
            break;
        default:
            std::stringstream ss;
            ss << "Received Unknown TYPE_CHN:" << toHex(packet.data);
            _helper.logInfo(ss.str());
            break;
    }
}

bool
M4Lib::_handleCommand(m4Packet& packet)
{
    switch(packet.commandID()) {
        case Yuneec::CMD_TX_STATE_MACHINE: {
                std::vector<uint8_t> commandValues = packet.commandValues();
                M4State state = M4State(commandValues[0] & 0x1f);
                if(state != _m4State) {
                    M4State old_state = _m4State;
                    _m4State = state;
                    if (_m4StateChangedCallback) {
                        _m4StateChangedCallback();
                    }
                    std::stringstream ss;
                    ss << "New State:" << m4StateStr() << "(" << int(_m4State) << ")";
                    _helper.logDebug(ss.str());
                    //-- If we were connected and just finished calibration, bind again
                    if(_vehicleConnected && old_state == M4State::FACTORY_CAL) {
                        _initAndCheckBinding();
                    }

                    if (_m4State == M4State::AWAIT && _m4IntentState == M4State::AWAIT) {
                        _unbind();
                    }

                    if (old_state == M4State::NONE) {
                        _exitToAwait();
                    }
                }
            }
            break;
        case Yuneec::CMD_TX_SWITCH_CHANGED:
            _switchChanged(packet);
            return true;
        case Yuneec::CMD_CALIBRATION_STATE_CHANGE:
            _calibrationStateChanged(packet);
            return true;
        default: {
                std::stringstream ss;
                ss << "Received Unknown TYPE_CMD:" << packet.commandID() << toHex(packet.data);
                _helper.logDebug(ss.str());
                break;
            }
    }
    return false;
}

void
M4Lib::_switchChanged(m4Packet& packet)
{
    std::vector<uint8_t> commandValues = packet.commandValues();
    SwitchChanged switchChanged;
    switchChanged.hwId      = int(commandValues[0]);
    switchChanged.newState  = int(commandValues[2]); // This was previously mixed up with oldState
    //switchChanged.oldState  = (int)commandValues[1]; // Unused.

    switch (switchChanged.hwId) {
        case Yuneec::SWITCH_OBS:
            if (_switchStateChangedCallback) {
                //-- On is position 3 (index 2)
                if (switchChanged.newState == 2) {
                    _switchStateChangedCallback(SwitchId::OBSTACLE_AVOIDANCE, SwitchState::ON);
                    _helper.logDebug("Obstacle avoidance switch on");
                } else if (switchChanged.newState == 1) {
                    _switchStateChangedCallback(SwitchId::OBSTACLE_AVOIDANCE, SwitchState::CENTER);
                    _helper.logDebug("Obstacle avoidance switch center");
                } else {
                    _switchStateChangedCallback(SwitchId::OBSTACLE_AVOIDANCE, SwitchState::OFF);
                    _helper.logDebug("Obstacle avoidance switch off");
                }
            }
            break;
        case Yuneec::SWITCH_MODE:
            if (_switchStateChangedCallback) {
                //-- On is position 3 (index 2)
                if (switchChanged.newState == 2) {
                    _switchStateChangedCallback(SwitchId::FLIGHT_MODE, SwitchState::ON);
                    _helper.logDebug("Flight mode switch on");
                } else if (switchChanged.newState == 1) {
                    _switchStateChangedCallback(SwitchId::FLIGHT_MODE, SwitchState::CENTER);
                    _helper.logDebug("Flight mode switch center");
                } else {
                    _switchStateChangedCallback(SwitchId::FLIGHT_MODE, SwitchState::OFF);
                    _helper.logDebug("Flight mode switch off");
                }
            }
            break;
        case Yuneec::BUTTON_POWER:
            if (_buttonStateChangedCallback) {
                //-- Pressed is 0
                if (switchChanged.newState == 0) {
                    _buttonStateChangedCallback(ButtonId::POWER, ButtonState::PRESSED);
                    _helper.logDebug("Power button pressed");
                } else {
                    _buttonStateChangedCallback(ButtonId::POWER, ButtonState::NORMAL);
                    _helper.logDebug("Power button normal");
                }
            }
            break;
        case Yuneec::BUTTON_AUX:
            if (_buttonStateChangedCallback) {
                //-- Pressed is 0
                if (switchChanged.newState == 0) {
                    _buttonStateChangedCallback(ButtonId::AUX, ButtonState::PRESSED);
                    _helper.logDebug("Aux button pressed");
                } else {
                    _buttonStateChangedCallback(ButtonId::AUX, ButtonState::NORMAL);
                    _helper.logDebug("Aux button normal");
                }
            }
            break;
       case Yuneec::BUTTON_CAMERA_SHUTTER:
            if (_buttonStateChangedCallback) {
                if (switchChanged.newState == 0) {
                    _buttonStateChangedCallback(ButtonId::CAMERA_SHUTTER, ButtonState::PRESSED);
                    _helper.logDebug("Camera button pressed");
                } else {
                    _buttonStateChangedCallback(ButtonId::CAMERA_SHUTTER, ButtonState::NORMAL);
                    _helper.logDebug("Camera button normal");
                }
            }
           break;
       case Yuneec::BUTTON_VIDEO_SHUTTER:
            if (_buttonStateChangedCallback) {
                if (switchChanged.newState == 0) {
                    _buttonStateChangedCallback(ButtonId::VIDEO_SHUTTER, ButtonState::PRESSED);
                    _helper.logDebug("Video button pressed");
                } else {
                    _buttonStateChangedCallback(ButtonId::VIDEO_SHUTTER, ButtonState::NORMAL);
                    _helper.logDebug("Video button normal");
                }
            }
            break;
        default: {
                std::stringstream ss;
                ss << "Unhandled switch/button: " << switchChanged.hwId << " - " << switchChanged.newState;
                _helper.logWarn(ss.str());
            }
            break;
    }
}

/*
 * Calibration Progress
*/
void
M4Lib::_calibrationStateChanged(m4Packet &packet)
{
    bool state  = true;
    bool change = false;
    std::vector<uint8_t> commandValues = packet.commandValues();
    for (unsigned int i = CalibrationHwIndexJ1; i < CalibrationHwIndexMax; ++i) {
        if(_rawChannelsCalibration[i] != commandValues[i]) {
            _rawChannelsCalibration[i] = commandValues[i];
            change = true;
        }
        //if ((int)commandValues[i] != TyphoonHQuickInterface::CalibrationStateRag) {
        // TODO: we don't want to depend on TyphoonHQuickInterface::CalibrationStateRag, so we just
        //       copy the value over and hardcode it here.
        if (int(commandValues[i]) != 4) {
            state = false;
        }
    }

    /*
    QString text = "Cal: ";
    for (int i = CalibrationHwIndexJ1; i < CalibrationHwIndexMax; ++i) {
        text += QString::number(commandValues[i]);
        text += " ";
    }
    qDebug() << text;
    */

    if(_rcCalibrationComplete != state) {
        _rcCalibrationComplete = state;
        if (_calibrationCompleteChangedCallback) {
            _calibrationCompleteChangedCallback();
        }
    }
    if(change) {
        if (_calibrationStateChangedCallback) {
            _calibrationStateChangedCallback();
        }
    }
}

void
M4Lib::_handleRawChannelData(m4Packet& packet)
{
    std::vector<uint8_t> values = packet.commandValues();
    int analogChannelCount = _rxBindInfoFeedback.aNum  ? _rxBindInfoFeedback.aNum  : 10;
    int val1, val2;
    _rawChannels.clear();
    for(int i = 0; i < analogChannelCount; i++) {
        uint16_t value = 0;
        unsigned int index = static_cast<unsigned int>(std::floor(i * 1.5));
        val1 = values[index] & 0xff;
        val2 = values[index + 1] & 0xff;
        if(i % 2 == 0) {
            value = static_cast<uint16_t>(val1 << 4 | val2 >> 4);
        } else {
            value = static_cast<uint16_t>((val1 & 0x0f) << 8 | val2);
        }
        _rawChannels.push_back(value);
    }
    if (_rawChannelsChangedCallback) {
        _rawChannelsChangedCallback();
    }
#if 0
    std::stringstream ss;
    ss << "Raw channels ( " << int(analogChannelCount) << ")";
    _helper.logInfo(ss.str());
    ss.clear();
    for(int i = 0; i < _rawChannels.size(); i++) {
        ss << int(_rawChannels[i]);
        if (i + 1< _rawChannels.size()) {
            ss << ", ";
        }
    }
    _helper.logDebug(ss.str());
#endif
}

void
M4Lib::_handleMixedChannelData(m4Packet& packet)
{
    std::vector<uint8_t> values = packet.commandValues();
    _mixedChannels.clear();
    int analogChannelCount = 10;
    int switchChannelCount = 2;
    unsigned int value, val1, val2;
    for(int i = 0; i < analogChannelCount + switchChannelCount; i++) {
        if(i < analogChannelCount) {
            unsigned int index = static_cast<unsigned int>(floor(i * 1.5));
            val1 = values[index] & 0xff;
            val2 = values[index + 1] & 0xff;
            if(i % 2 == 0) {
                value = val1 << 4 | val2 >> 4;
            } else {
                value = (val1 & 0x0f) << 8 | val2;
            }
        } else {
            unsigned int index = static_cast<unsigned int>(ceil((analogChannelCount - 1) * 1.5) + ceil((i - analogChannelCount + 1) * 0.25));
            val1 = values[index] & 0xff;
            switch((i - analogChannelCount + 1) % 4) {
                case 1:
                    value = val1 >> 6 & 0x03;
                    break;
                case 2:
                    value = val1 >> 4 & 0x03;
                    break;
                case 3:
                    value = val1 >> 2 & 0x03;
                    break;
                case 0:
                    value = val1 >> 0 & 0x03;
                    break;
                default:
                    value = 0;
                    break;
            }
        }
        _mixedChannels.push_back(static_cast<uint16_t>(value));
    }
    if(_mixedChannelsChangedCallback) {
        _mixedChannelsChangedCallback();
    }
#if 0
    std::stringstream ss;
    ss << "Mixed channels (" << int(_mixedChannels.size()) << ")";
    for(unsigned int i = 0; i < _mixedChannels.size(); i++) {
        ss << int(_mixedChannels[i]) << ", ";
    }
    _helper.logDebug(ss.str());
#endif
}

void
M4Lib::_handleControllerFeedback(m4Packet& packet)
{
    std::vector<uint8_t> commandValues = packet.commandValues();
    int ilat = _byteArrayToInt(commandValues, 0);
    int ilon = _byteArrayToInt(commandValues, 4);
    int ialt = _byteArrayToInt(commandValues, 8);
    _controllerLocation.latitude     = double(ilat) / 1e7;
    _controllerLocation.longitude    = double(ilon) / 1e7;
    _controllerLocation.altitude     = double(ialt) / 1e7;
     // The uBlox NAV-PVT value has precision 0.01 and this value is /10 in the driver.
     // We /10 again to get the unitless actual number.
    _controllerLocation.pdop         = float(_byteArrayToShort(commandValues, 12))/10.0f;
    // The uBlox NAV-PVT value is in mm/s which is /100 in the driver.
    // We /10 to get m/s.
    _controllerLocation.speed        = float(_byteArrayToShort(commandValues, 14))/10.0f;
    // The uBloxk NAV-PVT value is in 1e-5 degrees which is converted to -180..180 degrees in driver.
    _controllerLocation.heading      = _byteArrayToShort(commandValues, 16);
    _controllerLocation.satelliteCount = commandValues[18] & 0x1f;
    if (_controllerLocationChangedCallback) {
        _controllerLocationChangedCallback();
    }
}


void
M4Lib::_handlePassThroughPacket(m4Packet& packet)
{
    //Handle pass thrugh messages
    std::vector<uint8_t> passThroughValues = packet.passthroughValues();
}

std::string
M4Lib::m4StateStr()
{
    switch(_m4State) {
        case M4State::NONE:
            return std::string("Waiting for vehicle to connect...");
        case M4State::AWAIT:
            return std::string("Waiting...");
        case M4State::BIND:
            return std::string("Binding...");
        case M4State::CALIBRATION:
            return std::string("Calibration...");
        case M4State::SETUP:
            return std::string("Setup...");
        case M4State::RUN:
            return std::string("Running...");
        case M4State::SIM:
            return std::string("Simulation...");
        case M4State::FACTORY_CAL:
            return std::string("Factory Calibration...");
    }
    return std::string();
}

int
M4Lib::_byteArrayToInt(std::vector<uint8_t> data, unsigned int offset, bool isBigEndian)
{
    int iRetVal = -1;
    if(data.size() < offset + 4) {
        return iRetVal;
    }
    int iLowest;
    int iLow;
    int iMid;
    int iHigh;
    if(isBigEndian) {
        iLowest = data[offset + 3];
        iLow    = data[offset + 2];
        iMid    = data[offset + 1];
        iHigh   = data[offset + 0];
    } else {
        iLowest = data[offset + 0];
        iLow    = data[offset + 1];
        iMid    = data[offset + 2];
        iHigh   = data[offset + 3];
    }
    iRetVal = (iHigh << 24) | ((iMid & 0xFF) << 16) | ((iLow & 0xFF) << 8) | (0xFF & iLowest);
    return iRetVal;
}

short
M4Lib::_byteArrayToShort(std::vector<uint8_t> data, unsigned int offset, bool isBigEndian)
{
    short iRetVal = -1;
    if(data.size() < offset + 2) {
        return iRetVal;
    }
    int iLow;
    int iHigh;
    if(isBigEndian) {
        iLow    = data[offset + 1];
        iHigh   = data[offset + 0];
    } else {
        iLow    = data[offset + 0];
        iHigh   = data[offset + 1];
    }
    iRetVal = static_cast<short>((iHigh << 8) | (0xFF & iLow));
    return iRetVal;
}

int
M4Lib::calChannel(int index)
{
    if(index < CalibrationHwIndexMax) {
        return _rawChannelsCalibration[index];
    }
    return 0;
}

std::string M4Lib::_getRxBindInfoFeedbackName()
{
    std::stringstream nodeSs;
    nodeSs << _rxBindInfoFeedback.nodeId;

    switch (static_cast<RxBindInfo::Type>(_rxBindInfoFeedback.mode)) {
        case RxBindInfo::Type::SR12S:
            return std::string("SR12S_") + nodeSs.str();
        case RxBindInfo::Type::SR12E:
            return std::string("SR12E_") + nodeSs.str();
        case RxBindInfo::Type::SR24S:
            return std::string("SR24S_") + nodeSs.str() + std::string(" v1.03");
        case RxBindInfo::Type::RX24:
            return std::string("RX24_") + nodeSs.str();
        case RxBindInfo::Type::SR19P:
            return std::string("SR19P_") + nodeSs.str();
        default:
            if (_rxBindInfoFeedback.mode >= 105) {
                std::stringstream modeSs;
                modeSs << float(_rxBindInfoFeedback.mode) / 100.0f;
                return std::string("SR24S_") + nodeSs.str() + std::string("v") + modeSs.str();
            } else {
                return nodeSs.str();
            }
    }
}

#endif // defined(__androidx86__)
