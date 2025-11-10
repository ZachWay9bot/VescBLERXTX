/*
  VescThrottle_GT911_TAMC_StandaloneTx.ino
  - KEINE Änderungen an VescBLE.cpp
  - Classic BLE scan/connect, bindet vescblerxtx via set_writer()
  - TAMC_GT911 + TFT_eSPI 240x320 Portrait
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

#include <TAMC_GT911.h>
#include <vescblerxtx.h>

// Touch Pins (dein Standard-Setup)
#define GT911_SDA 33
#define GT911_SCL 32
#define GT911_INT 21
#define GT911_RST 25

#define TP_W 240
#define TP_H 320

// BLE (VESC NUS)
static const char* BLE_TARGET_NAME = "VESC BLE UART";
static BLEUUID UART_SERVICE_UUID  ("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID UART_CHAR_TX_UUID  ("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // write

TFT_eSPI tft;
TAMC_GT911 touch(GT911_INT, GT911_RST, GT911_SDA, GT911_SCL, TP_W, TP_H);

static const int W=240, H=320;
static const int TRACK_X=100, TRACK_W=40;
static const int TRACK_Y1=30, TRACK_Y2=H-30;
static const int KNOB_W=44, KNOB_H=18;

static const float    MAX_CURRENT_A=20.0f;   // <- hier Stromlimit
static const float    ALPHA=0.25f;
static const uint32_t SEND_MS=40;
static const uint32_t IDLE_ZERO_MS=150;

float targetA=0.0f, smoothA=0.0f;
uint32_t lastSend=0, lastTouchMs=0;
int lastKnobY=-9999;

// BLE state
static BLEAdvertisedDevice* g_found = nullptr;
static BLEClient* g_client = nullptr;
static BLERemoteCharacteristic* g_tx = nullptr;

// Writer-Callback: framed Bytes -> TX schreiben (Classic BLE)
static bool vesc_tx_writer(const uint8_t* data, size_t len) {
  if (!g_tx) return false;
  g_tx->writeValue((uint8_t*)data, len, false);
  return true;
}

// Scan Callback
class MyAdvCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    bool match=false;
    if (d.haveName()){
      std::string n=d.getName();
      if (n.find("VESC")!=std::string::npos || n.find("UART")!=std::string::npos) match=true;
      if (!strcmp(n.c_str(), BLE_TARGET_NAME)) match=true;
    }
    if (!match && d.haveServiceUUID() && d.isAdvertisingService(UART_SERVICE_UUID)) match=true;
    if (match){
      g_found = new BLEAdvertisedDevice(d);
      BLEDevice::getScan()->stop();
    }
  }
};

static void ui_status(const char* s, uint16_t col){
  tft.fillRect(6,6, W-12, 14, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(col);
  tft.drawString(s, 6, 6);
}
static void ui_draw(){
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("VESC Throttle (A)", W/2, 14);
  tft.drawRoundRect(TRACK_X, TRACK_Y1, TRACK_W, TRACK_Y2-TRACK_Y1, 6, TFT_DARKGREY);
  for(int i=0;i<=10;++i){
    int y=map(i,0,10,TRACK_Y2,TRACK_Y1);
    tft.drawLine(TRACK_X+TRACK_W+4,y,TRACK_X+TRACK_W+12,y,TFT_DARKGREY);
  }
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("0", TRACK_X-10, TRACK_Y2);
  char b[12]; snprintf(b,sizeof(b),"%.0f",MAX_CURRENT_A);
  tft.drawString(b, TRACK_X-10, TRACK_Y1);
  ui_status("BLE: scanning…", TFT_YELLOW);
}
static void ui_ready(bool ready){
  ui_status(ready ? "TX ready" : "NO TX/BIND", ready ? TFT_GREEN : TFT_RED);
}
static void ui_knob(int y){
  static int lastY = -9999;
  if (lastY!=-9999){
    tft.fillRect(TRACK_X-2, lastY-KNOB_H/2-1, TRACK_W+4, KNOB_H+2, TFT_BLACK);
    tft.drawRoundRect(TRACK_X, TRACK_Y1, TRACK_W, TRACK_Y2-TRACK_Y1, 6, TFT_DARKGREY);
  }
  if (y<TRACK_Y1) y=TRACK_Y1;
  if (y>TRACK_Y2) y=TRACK_Y2;
  tft.fillRoundRect(TRACK_X-2, y-KNOB_H/2, TRACK_W+4, KNOB_H, 6, TFT_BLUE);
  tft.drawRoundRect(TRACK_X-2, y-KNOB_H/2, TRACK_W+4, KNOB_H, 6, TFT_WHITE);

  tft.fillRect(W-110, H-44, 108, 40, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_CYAN);
  tft.setTextSize(2);
  char s[24]; snprintf(s, sizeof(s), "%4.1f A", smoothA);
  tft.drawString(s, W-56, H-24);

  lastY=y;
}
static float yToCurrentA(int y){
  float t=(float)(TRACK_Y2 - y)/(float)(TRACK_Y2 - TRACK_Y1);
  if (t<0) t=0; if (t>1) t=1;
  return t*MAX_CURRENT_A;
}
static bool touch_point(int &tx, int &ty){
  touch.read();
  if (!touch.isTouched) return false;
  tx = touch.points[0].x;
  ty = touch.points[0].y;
  return true;
}

static bool bind_tx(){
  if (vescblerxtx::is_ready()) return true;
  // Start scan (5s)
  BLEScan* sc=BLEDevice::getScan();
  sc->setAdvertisedDeviceCallbacks(new MyAdvCB(), true);
  sc->setActiveScan(true); sc->setInterval(45); sc->setWindow(30);
  sc->start(5, false);

  if (!g_found) return false;

  BLEClient* client = BLEDevice::createClient();
  if (!client->connect(g_found)) { ui_status("BLE: connect fail", TFT_RED); return false; }

  BLERemoteService* svc = client->getService(UART_SERVICE_UUID);
  if (!svc){ ui_status("BLE: no NUS svc", TFT_RED); return false; }
  BLERemoteCharacteristic* tx = svc->getCharacteristic(UART_CHAR_TX_UUID);
  if (!tx || !tx->canWrite()){ ui_status("BLE: no TX writable", TFT_RED); return false; }

  g_client = client;
  g_tx = tx;
  vescblerxtx::set_writer(&vesc_tx_writer);
  ui_ready(true);
  return true;
}

void setup(){
  Serial.begin(115200);

  tft.init(); tft.setRotation(0);
  ui_draw();

  Wire.begin(GT911_SDA, GT911_SCL);
  touch.begin();

  BLEDevice::init("ESP32-Throttle");
  BLEDevice::setPower(ESP_PWR_LVL_P9);

  bind_tx();
  smoothA=0; ui_knob(TRACK_Y2);
}

void loop(){
  bind_tx();

  int tx, ty;
  bool touched = touch_point(tx, ty);
  if (touched){
    if (tx >= (TRACK_X-10) && tx <= (TRACK_X+TRACK_W+10) && ty >= TRACK_Y1 && ty <= TRACK_Y2){
      targetA = yToCurrentA(ty);
      lastTouchMs=millis();
      ui_knob(ty);
    }
  }
  if (!touched && (millis()-lastTouchMs)>IDLE_ZERO_MS) targetA=0.0f;

  // Glätten + Senden
  smoothA = smoothA + ALPHA*(targetA - smoothA);
  static uint32_t lastSend=0;
  if (millis()-lastSend >= SEND_MS){
    lastSend=millis();
    if (vescblerxtx::is_ready()){
      vescblerxtx::set_current(smoothA);
      ui_ready(true);
    } else ui_ready(false);
  }
}
