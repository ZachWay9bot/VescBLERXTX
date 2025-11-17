
#include "vescblerxtx.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

VescBleRxTx* VescBleRxTx::_active = nullptr;
static bool s_bleInit = false;

template<typename T> static inline T clampv(T x, T lo, T hi){ return x<lo?lo:(x>hi?hi:x); }

class VescBleRxTx::_ClientCB : public BLEClientCallbacks {
  void onConnect(BLEClient* c) override {}
  void onDisconnect(BLEClient* c) override {
    if (VescBleRxTx::_active) VescBleRxTx::_active->_handleDisconnect();
  }
};

VescBleRxTx::VescBleRxTx(){}

void VescBleRxTx::_ensureBleInit(){
  if (!s_bleInit){
    BLEDevice::init("");
    s_bleInit = true;
  }
}

void VescBleRxTx::begin(){
  _ensureBleInit();
}

void VescBleRxTx::disconnect(){
  if (!_connected && !_connecting) return;
  BLEClient* client = reinterpret_cast<BLEClient*>(_client);
  if (client){ client->disconnect(); }
  _handleDisconnect();
}

bool VescBleRxTx::_connectResolvedAddress(const char* macStr){
  _ensureBleInit();
  BLEAddress addr(macStr);
  BLEClient* client = BLEDevice::createClient();
  client->setClientCallbacks(new _ClientCB());
  if (!client->connect(addr)) return false;
  _client = client;
  return _postConnectSetup();
}

bool VescBleRxTx::_postConnectSetup(){
  BLEClient* client = reinterpret_cast<BLEClient*>(_client);
  if (!client) return false;
  BLERemoteService* svc = client->getService(BLEUUID(_uuidSvc.c_str()));
  if (!svc) return false;
  BLERemoteCharacteristic* rx = svc->getCharacteristic(BLEUUID(_uuidRx.c_str()));
  BLERemoteCharacteristic* tx = svc->getCharacteristic(BLEUUID(_uuidTx.c_str()));
  if (!rx || !tx) return false;
  _rxChar = rx; _txChar = tx;
  if (tx->canNotify()){
    tx->registerForNotify([](BLERemoteCharacteristic* chr, uint8_t* data, size_t len, bool isNotify){
      if (VescBleRxTx::_active) VescBleRxTx::_active->_handleNotify(data, len);
    });
  }
  _connected = true;
  _connecting = false;
  _active = this;
  if (_onConn) _onConn();
  return true;
}

bool VescBleRxTx::connectByMac(const char* mac){
  _targetMac = mac ? mac : "";
  _targetName = "";
  _connecting = true;
  bool ok = _connectResolvedAddress(_targetMac.c_str());
  _connecting = false;
  if (!ok) _handleDisconnect();
  return ok;
}

bool VescBleRxTx::connectByName(const char* name){
  _ensureBleInit();
  _targetName = name ? name : "";
  _targetMac  = "";
  BLEScan* scan = BLEDevice::getScan();
  scan->setActiveScan(true);
  BLEScanResults results = scan->start(6, false);
  String macFound;
  for (int i = 0; i < results.getCount(); ++i){
    BLEAdvertisedDevice d = results.getDevice(i);
    if (d.haveName() && _targetName == d.getName().c_str()){
      macFound = d.getAddress().toString().c_str();
      break;
    }
  }
  scan->stop();
  if (macFound.length() == 0) return false;
  return connectByMac(macFound.c_str());
}

void VescBleRxTx::setNusUuids(const char* svc, const char* rx, const char* tx){
  if (svc) _uuidSvc = svc;
  if (rx)  _uuidRx  = rx;
  if (tx)  _uuidTx  = tx;
}

void VescBleRxTx::setControlMode(int mode){ _mode = clampv(mode, 0, 3); }
void VescBleRxTx::setCommand(float value){ _cmdUser = value; _lastCmdSetMs = millis(); }

float VescBleRxTx::_applyDeadband(float v) const {
  const float db = _deadbandPct;
  if (fabsf(v) < db) return 0.0f;
  return v;
}
float VescBleRxTx::_applyIir(float in){
  _cmdFilt = _iirAlpha * in + (1.0f - _iirAlpha) * _cmdFilt;
  return _cmdFilt;
}
float VescBleRxTx::_applySlew(float in, float dt_s){
  const float maxDelta = _slewAperSec * dt_s;
  float delta = in - _cmdFilt;
  if (delta >  maxDelta) delta =  maxDelta;
  if (delta < -maxDelta) delta = -maxDelta;
  _cmdFilt += delta;
  return _cmdFilt;
}
static inline int32_t float_to_be1000(float v){
  long iv = lroundf(v * 1000.0f);
  return (int32_t)iv;
}
int32_t VescBleRxTx::_encodeToBE1000(float v) const { return float_to_be1000(v); }

void VescBleRxTx::loop(){
  const uint32_t now = millis();
  float cmd = _cmdUser;
  switch (_mode){
    case VESC_MODE_BRAKE: cmd = cmd < 0 ? 0 : cmd; break;
    case VESC_MODE_DUTY:  cmd = clampv(cmd, -1.0f, 1.0f); break;
    default: break;
  }
  cmd = _applyDeadband(cmd);
  _applyIir(cmd);
  float dt = (now - _lastSendMs) * 0.001f; if (dt < 0) dt = 0;
  _applySlew(_cmdFilt, dt);

  if (_connected && (now - _lastSendMs >= _sendIntervalMs)){
    float out = _cmdFilt;
    if ((now - _lastCmdSetMs) > _idleZeroMs){
      out = 0.0f; _cmdFilt = 0.0f;
    }
    BLERemoteCharacteristic* rx = reinterpret_cast<BLERemoteCharacteristic*>(_rxChar);
    if (rx){
      int32_t be = _encodeToBE1000(out);
      uint8_t b[4] = {
        uint8_t((be >> 24) & 0xFF),
        uint8_t((be >> 16) & 0xFF),
        uint8_t((be >>  8) & 0xFF),
        uint8_t( be        & 0xFF)
      };
      rx->writeValue(b, 4, false); // WWR
    }
    _lastSendMs = now;
  }
}

void VescBleRxTx::_handleNotify(uint8_t* data, size_t len){
  if (_rawCb) _rawCb(data, len);
  for (size_t i=0;i<len;i++){
    char c = (char)data[i];
    if (c == '\r') continue;
    if (c == '\n'){
      if (_lineLen > 0){ _parseTelemetryLine(_lineBuf, _lineLen); _lineLen = 0; }
    } else {
      if (_lineLen < _NL-1){ _lineBuf[_lineLen++] = c; _lineBuf[_lineLen] = 0; }
    }
  }
}

static bool parse_keyval(const char* s, String& key, float& val){
  const char* eq = strchr(s, '=');
  if (!eq) return false;
  key = String();
  const char* p = s;
  while (*p && *p <= ' ') p++;
  const char* q = eq-1;
  while (q>p && *q <= ' ') q--;
  key.reserve(q-p+1);
  for (const char* k=p; k<=q; ++k) key += *k;
  val = atof(eq+1);
  return true;
}

void VescBleRxTx::_parseTelemetryLine(const char* line, size_t len){
  if (!_telCb) return;
  VescTelemetry t{}; t.millisStamp = millis();
  String token;
  for (size_t i=0;i<=len;i++){
    char c = (i<len) ? line[i] : ',';
    if (c==',' || c==';'){
      if (token.length()){
        String key; float v=NAN;
        if (parse_keyval(token.c_str(), key, v)){
          if      (key=="volt"||key=="voltage")         t.voltage = v;
          else if (key=="erpm"||key=="rpm")             t.erpm = v;
          else if (key=="duty")                         t.duty = v;
          else if (key=="ain" ||key=="avginputcurrent") t.avgInputCurrent = v;
          else if (key=="amot"||key=="avgmotorcurrent") t.avgMotorCurrent = v;
          else if (key=="tmos"||key=="tempmosfet")      t.tempMosfet = v;
          else if (key=="tmot"||key=="tempmotor")       t.tempMotor = v;
        }
        token = String();
      }
    } else { token += c; }
  }
  _telCb(t);
}

void VescBleRxTx::_handleDisconnect(){
  _connected = false; _connecting = false;
  _client = nullptr; _rxChar = nullptr; _txChar = nullptr;
  if (_onDisc) _onDisc();
}
