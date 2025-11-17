#pragma once
#include <Arduino.h>

enum {
  VESC_MODE_CURRENT = 0,
  VESC_MODE_BRAKE   = 1,
  VESC_MODE_DUTY    = 2,
  VESC_MODE_RPM     = 3
};

class VescBleRxTx {
public:
  struct VescTelemetry {
    float voltage         = NAN;
    float erpm            = NAN;
    float duty            = NAN;
    float avgInputCurrent = NAN;
    float avgMotorCurrent = NAN;
    float tempMosfet      = NAN;
    float tempMotor       = NAN;
    float current_in      = NAN;
    float current_motor   = NAN;
    float watt_hours      = NAN;
    float watt_hours_charged = NAN;
    float amp_hours       = NAN;
    float amp_hours_charged = NAN;
    float speed_kmh       = NAN;
    float position        = NAN;
    int32_t tachometer    = 0;
    int32_t tachometer_abs = 0;
    float temp_mos_1      = NAN;
    float temp_mos_2      = NAN;
    float temp_mos_3      = NAN;
    float temp_pcb        = NAN;
    float battery_level   = NAN;
    float battery_wh      = NAN;
    uint8_t fault_code    = 0;
    uint8_t controller_id = 0;
    float pid_pos         = NAN;
    uint32_t millisStamp  = 0;
  };

  using TelemetryFn = void(*)(const VescTelemetry&);
  using VoidFn      = void(*)();
  using RawNotifyFn = void(*)(const uint8_t*, size_t);

  VescBleRxTx();

  void begin();
  void loop();
  bool isConnected() const { return _connected; }
  void disconnect();

  bool connectByName(const char* name);
  bool connectByMac (const char* mac);

  void setNusUuids(const char* svc, const char* rx, const char* tx);

  void setControlMode(int mode);
  int  getControlMode() const { return _mode; }
  void setCommand(float value);

  void setSendIntervalMs(uint16_t ms) { _sendIntervalMs = ms; }
  void setIdleZeroMs(uint16_t ms)     { _idleZeroMs = ms; }
  void setIirAlpha(float a)           { _iirAlpha = constrain(a, 0.0f, 1.0f); }
  void setSlewAperSec(float a_per_s)  { _slewAperSec = max(0.0f, a_per_s); }
  void setDeadbandPct(float pct01)    { _deadbandPct = constrain(pct01, 0.0f, 0.49f); }

  void onTelemetry(TelemetryFn fn) { _telCb = fn; }
  void onConnect  (VoidFn fn)      { _onConn = fn; }
  void onDisconnect(VoidFn fn)     { _onDisc = fn; }
  void onRawNotify(RawNotifyFn fn) { _rawCb = fn; }

  void setDebug(bool en) { _debug = en; }

private:
  void _ensureBleInit();
  bool _connectResolvedAddress(const char* macStr);
  bool _postConnectSetup();
  void _handleNotify(uint8_t* data, size_t len);
  void _handleDisconnect();

  int32_t _encodeToBE1000(float v) const;
  float   _applyDeadband(float v) const;
  float   _applyIir(float in);
  float   _applySlew(float in, float dt_s);
  void    _parseTelemetryLine(const char* line, size_t len);

  String _uuidSvc = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
  String _uuidRx  = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
  String _uuidTx  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

  bool _connected = false;
  bool _connecting = false;
  bool _debug = false;

  String _targetName;
  String _targetMac;

  int    _mode = VESC_MODE_CURRENT;
  float  _cmdUser = 0.0f;
  float  _cmdFilt = 0.0f;
  float  _iirAlpha = 0.20f;
  float  _slewAperSec = 4.0f;
  float  _deadbandPct = 0.06f;

  uint16_t _sendIntervalMs = 50;
  uint16_t _idleZeroMs     = 150;
  uint32_t _lastSendMs     = 0;
  uint32_t _lastCmdSetMs   = 0;

  TelemetryFn _telCb = nullptr;
  RawNotifyFn _rawCb = nullptr;
  VoidFn _onConn = nullptr;
  VoidFn _onDisc = nullptr;

  static const size_t _NL = 256;
  char  _lineBuf[_NL];
  size_t _lineLen = 0;

  class _ClientCB;
  static VescBleRxTx* _active;
  void* _client = nullptr;
  void* _rxChar = nullptr;
  void* _txChar = nullptr;
};
