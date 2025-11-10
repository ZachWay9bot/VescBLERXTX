/*
  VescThrottle_Current_BBCapTouch_Debug.ino
  - sendet COMM_SET_CURRENT (A*1000 int32 BE) über vescblerxtx
  - hexdump der gesendeten Frames
  - ARM-Toggle oben links
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>

#include <bb_captouch.h>
#include <vescblerxtx.h>

#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21
#define TOUCH_RST 25

#define TP_W 240
#define TP_H 320

static const char* BLE_TARGET_NAME = "VESC BLE UART";
static BLEUUID UART_SERVICE_UUID  ("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UART_CHAR_TX_UUID  ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

TFT_eSPI tft;
BBCapTouch bbct;

static const int W=240,H=320,TRACK_X=100,TRACK_W=40,TRACK_Y1=30,TRACK_Y2=H-30;
static const float ALPHA=0.25f;
static const uint32_t SEND_MS=80;
static const float I_MAX=2.0f;  // klein anfangen
static const float DEAD=0.0f;

float target=0, smooth=0;
uint32_t lastSend=0;
bool armed=false;

static BLEAdvertisedDevice* g_found=nullptr; static BLEClient* g_client=nullptr; static BLERemoteCharacteristic* g_tx=nullptr;

static bool vesc_tx_writer(const uint8_t* data, size_t len) {
  if (!g_tx) return false;
  g_tx->writeValue((uint8_t*)data, len, false);
  Serial.print("TX "); for(size_t i=0;i<len;i++){ char b[4]; sprintf(b,"%02X", data[i]); Serial.print(b); Serial.print(' ');} Serial.println();
  return true;
}

class MyAdvCB: public BLEAdvertisedDeviceCallbacks { void onResult(BLEAdvertisedDevice d) override {
  bool m=false; if(d.haveName()){auto n=d.getName(); if(n.find("VESC")!=std::string::npos||n.find("UART")!=std::string::npos)m=true; if(!strcmp(n.c_str(),BLE_TARGET_NAME))m=true;}
  if(!m && d.haveServiceUUID() && d.isAdvertisingService(UART_SERVICE_UUID)) m=true; if(m){ g_found=new BLEAdvertisedDevice(d); BLEDevice::getScan()->stop(); }}};

static void status(const char* s,uint16_t c){ tft.fillRect(6,6,W-12,14,TFT_BLACK); tft.setTextDatum(TL_DATUM); tft.setTextColor(c); tft.drawString(s,6,6); }
static void drawUI(){ tft.fillScreen(TFT_BLACK); tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE); tft.drawString("CURRENT TEST (A)",W/2,14);
  tft.drawRoundRect(TRACK_X,TRACK_Y1,TRACK_W,TRACK_Y2-TRACK_Y1,6,TFT_DARKGREY); status("BLE: scan…",TFT_YELLOW);
  tft.setTextDatum(TL_DATUM); tft.setTextColor(armed?TFT_GREEN:TFT_RED); tft.drawString(armed? "ARMED":"DISARMED", 6, 24); }

static float y2val(int y){ if(y<TRACK_Y1) y=TRACK_Y1; if(y>TRACK_Y2) y=TRACK_Y2; float t=(float)(TRACK_Y2-y)/(float)(TRACK_Y2-TRACK_Y1); if (fabsf(t-0.5f)<DEAD) t=0.5f; return t; }
static bool touch(int& x,int& y){ TOUCHINFO ti; if(!bbct.getSamples(&ti)) return false; if(ti.count<=0) return false; x=ti.x[0]; y=ti.y[0]; return true; }

static bool bindTX(){ if(vescblerxtx::is_ready()) return true; BLEScan* sc=BLEDevice::getScan(); sc->setAdvertisedDeviceCallbacks(new MyAdvCB(), true); sc->setActiveScan(true); sc->setInterval(45); sc->setWindow(30); sc->start(5,false);
  if(!g_found) return false; if(!g_client) g_client=BLEDevice::createClient(); if(!g_client->isConnected()){ if(!g_client->connect(g_found)){ status("BLE: connect fail",TFT_RED); return false; } }
  BLERemoteService* svc=g_client->getService(UART_SERVICE_UUID); if(!svc){ status("BLE: no NUS",TFT_RED); return false; } BLERemoteCharacteristic* tx=svc->getCharacteristic(UART_CHAR_TX_UUID); if(!tx||!tx->canWrite()){ status("BLE: no TX",TFT_RED); return false; }
  g_tx=tx; vescblerxtx::set_writer(&vesc_tx_writer); status("TX ready",TFT_GREEN); return true; }

void setup(){ Serial.begin(115200); tft.init(); tft.setRotation(0); drawUI(); Wire.begin(TOUCH_SDA,TOUCH_SCL); Wire.setClock(400000); bbct.init(TOUCH_SDA,TOUCH_SCL,TOUCH_RST,TOUCH_INT);
  BLEDevice::init("ESP32-Throttle"); BLEDevice::setPower(ESP_PWR_LVL_P9); }

void loop(){
  bindTX();
  int x,y; bool t=touch(x,y);
  if(t){
    if (x<70 && y<60){ armed=!armed; drawUI(); delay(200); }
    if (x>=TRACK_X-10 && x<=TRACK_X+TRACK_W+10 && y>=TRACK_Y1 && y<=TRACK_Y2){
      float val = y2val(y);
      target = val * I_MAX; // A
      tft.fillRect(6,40,120,20,TFT_BLACK); tft.setTextColor(TFT_CYAN); tft.setTextDatum(TL_DATUM);
      char s[32]; sprintf(s,"I=%.2f A", target); tft.drawString(s,6,40);
    }
  } else {
    target = 0.0f;
  }
  smooth += 0.25f*(target - smooth);
  if (millis()-lastSend > SEND_MS){
    lastSend = millis();
    float outA = armed ? smooth : 0.0f;
    vescblerxtx::set_current(outA);
  }
}
