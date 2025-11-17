\
/*
   GT911_Touch_Slider Example (configurable touch mapping)

   Wenn X/Y gespiegelt oder invertiert sind:
     -> einfach die Defines TOUCH_SWAP_XY / TOUCH_INVERT_X / TOUCH_INVERT_Y
        unten anpassen (0/1) und neu flashen.
*/

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <bb_captouch.h>
#include <vescblerxtx.h>

#define TFT_ROTATION 0

TFT_eSPI tft = TFT_eSPI();

#if (TFT_ROTATION == 0 || TFT_ROTATION == 2)
  static const int SCREEN_W = 240;
  static const int SCREEN_H = 320;
#else
  static const int SCREEN_W = 320;
  static const int SCREEN_H = 240;
#endif

// --- TOUCH MAPPING TWEAKS ---
// 1 = aktiv, 0 = aus
#define TOUCH_SWAP_XY   1   // 1: vertauscht X/Y
#define TOUCH_INVERT_X  0   // 1: spiegelt an X-Achse
#define TOUCH_INVERT_Y  0   // 1: spiegelt an Y-Achse

// GT911 Pins
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_RST 25
#define TOUCH_INT 21

BBCapTouch touch;

// BLE-Ziel
static const char *TARGET_NAME = "VESC_BLE";
static const char *TARGET_MAC  = "";

VescBleRxTx vesc;

static VescBleRxTx::VescTelemetry g_lastTel;
static volatile bool g_telPending = false;
static bool g_bleConnected = false;

// UI-State
static bool  g_armed      = false;
static int   g_mode       = VESC_MODE_CURRENT;
static float g_throttle01 = 0.0f;
static float g_brake01    = 0.0f;

static const float MAX_CURRENT_A = 2.0f;

static bool g_touchDownPrev = false;

// Slider-Geometrie
static const int SLIDER_TOP    = 80;
static const int SLIDER_HEIGHT = 210;
static const int SLIDER_WIDTH  = 60;
static const int SLIDER_MARGIN = 20;

static const int SLIDER_X_THROTTLE = SLIDER_MARGIN;
static const int SLIDER_X_BRAKE    = SCREEN_W - SLIDER_MARGIN - SLIDER_WIDTH;

// Button-Geometrie
static const int ARM_X  = 5;
static const int ARM_Y  = 5;
static const int ARM_W  = 70;
static const int ARM_H  = 30;

static const int MODE_W = 100;
static const int MODE_H = 30;
static const int MODE_X = SCREEN_W - MODE_W - 5;
static const int MODE_Y = 5;

// Reconnect
static uint32_t g_lastReconnectMs = 0;
static const uint32_t RECONNECT_MS = 5000;

// --- Hilfsfunktionen ---
static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// rohes Touch -> Display-Koordinaten
static void mapTouch(int rx, int ry, int &x, int &y)
{
  int tx = rx;
  int ty = ry;

  if (TOUCH_SWAP_XY) {
    int tmp = tx;
    tx = ty;
    ty = tmp;
  }
  if (TOUCH_INVERT_X) {
    tx = SCREEN_W - tx;
  }
  if (TOUCH_INVERT_Y) {
    ty = SCREEN_H - ty;
  }

  x = tx;
  y = ty;
}

// --- UI-Draw ---
static void drawStaticUI()
{
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawLine(0, 40, SCREEN_W, 40, TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);

  tft.setCursor(SLIDER_X_THROTTLE, SLIDER_TOP - 20);
  tft.print("THROTTLE");
  tft.setCursor(SLIDER_X_BRAKE, SLIDER_TOP - 20);
  tft.print("BRAKE");

  tft.drawRect(SLIDER_X_THROTTLE, SLIDER_TOP, SLIDER_WIDTH, SLIDER_HEIGHT, TFT_DARKGREY);
  tft.drawRect(SLIDER_X_BRAKE,    SLIDER_TOP, SLIDER_WIDTH, SLIDER_HEIGHT, TFT_DARKGREY);

  tft.drawRect(ARM_X, ARM_Y, ARM_W, ARM_H, TFT_DARKGREY);
  tft.drawRect(MODE_X, MODE_Y, MODE_W, MODE_H, TFT_DARKGREY);

  tft.setCursor(ARM_X + 10, ARM_Y + 10);
  tft.print("ARM");
  tft.setCursor(MODE_X + 10, MODE_Y + 10);
  tft.print("MODE");
}

static void drawArmButton()
{
  uint16_t fill = g_armed ? TFT_GREEN : TFT_DARKGREY;
  tft.fillRect(ARM_X + 1, ARM_Y + 1, ARM_W - 2, ARM_H - 2, fill);
  tft.drawRect(ARM_X, ARM_Y, ARM_W, ARM_H, TFT_WHITE);

  tft.setTextColor(TFT_BLACK, fill);
  tft.setTextSize(1);
  tft.setCursor(ARM_X + 8, ARM_Y + 10);
  tft.print(g_armed ? "ARMED" : "DISARM");
}

static const char* modeToText(int m)
{
  switch (m) {
    case VESC_MODE_CURRENT: return "CURRENT";
    case VESC_MODE_BRAKE:   return "BRAKE";
    case VESC_MODE_DUTY:    return "DUTY";
    case VESC_MODE_RPM:     return "RPM";
    default:                return "?";
  }
}

static void drawModeButton()
{
  uint16_t fill = TFT_NAVY;
  tft.fillRect(MODE_X + 1, MODE_Y + 1, MODE_W - 2, MODE_H - 2, fill);
  tft.drawRect(MODE_X, MODE_Y, MODE_W, MODE_H, TFT_WHITE);

  tft.setTextColor(TFT_YELLOW, fill);
  tft.setTextSize(1);
  tft.setCursor(MODE_X + 4, MODE_Y + 10);
  tft.print(modeToText(g_mode));
}

static void drawSlider(int x, float value01, bool isBrake)
{
  value01 = clamp01(value01);
  uint16_t fillColor = isBrake ? TFT_RED : TFT_BLUE;

  tft.fillRect(x + 1, SLIDER_TOP + 1, SLIDER_WIDTH - 2, SLIDER_HEIGHT - 2, TFT_BLACK);

  int filled = (int)roundf(value01 * (SLIDER_HEIGHT - 4));
  if (filled > 0) {
    int yFill = SLIDER_TOP + SLIDER_HEIGHT - 2 - filled;
    tft.fillRect(x + 2, yFill, SLIDER_WIDTH - 4, filled, fillColor);
  }

  tft.drawRect(x, SLIDER_TOP, SLIDER_WIDTH, SLIDER_HEIGHT, TFT_DARKGREY);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  char buf[16];
  int pct = (int)roundf(value01 * 100.0f);
  snprintf(buf, sizeof(buf), "%3d%%", pct);

  int textY = SLIDER_TOP + SLIDER_HEIGHT + 4;
  tft.fillRect(x, textY, SLIDER_WIDTH, 12, TFT_BLACK);
  tft.setCursor(x + 8, textY);
  tft.print(buf);
}

static void drawBleStatus(const char *status)
{
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.fillRect(0, 40, SCREEN_W, 12, TFT_BLACK);
  tft.setCursor(4, 42);
  tft.print("BLE: ");
  tft.print(status);
}

static void drawTelemetry(const VescBleRxTx::VescTelemetry &t)
{
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(1);

  tft.fillRect(0, 52, SCREEN_W, 12, TFT_BLACK);
  tft.setCursor(4, 54);
  tft.print("SPD: ");
  if (!isnan(t.speed_kmh)) {
    tft.print(t.speed_kmh, 1);
    tft.print(" km/h");
  } else if (!isnan(t.erpm)) {
    tft.print("erpm=");
    tft.print(t.erpm, 0);
  } else {
    tft.print("--");
  }

  tft.setCursor(SCREEN_W/2, 54);
  tft.print("V: ");
  if (!isnan(t.voltage)) {
    tft.print(t.voltage, 1);
  } else {
    tft.print("--");
  }

  tft.fillRect(0, 64, SCREEN_W, 12, TFT_BLACK);
  tft.setCursor(4, 66);
  tft.print("BAT: ");
  if (!isnan(t.battery_level)) {
    tft.print(t.battery_level, 0);
    tft.print("%");
  } else {
    tft.print("--");
  }

  tft.setCursor(SCREEN_W/2, 66);
  tft.print("I: ");
  if (!isnan(t.avgInputCurrent)) {
    tft.print(t.avgInputCurrent, 1);
    tft.print("A");
  } else {
    tft.print("--");
  }
}

// Callbacks
static void onVescTelemetry(const VescBleRxTx::VescTelemetry &t)
{
  g_lastTel = t;
  g_telPending = true;
}

static void onVescConnect()
{
  g_bleConnected = true;
  drawBleStatus("CONNECTED");
}

static void onVescDisconnect()
{
  g_bleConnected = false;
  drawBleStatus("DISCONNECTED");
}

static bool connectVesc()
{
  drawBleStatus("CONNECTING...");
  bool ok = false;
  if (TARGET_MAC && strlen(TARGET_MAC) > 0) {
    ok = vesc.connectByMac(TARGET_MAC);
  } else if (TARGET_NAME && strlen(TARGET_NAME) > 0) {
    ok = vesc.connectByName(TARGET_NAME);
  }
  if (!ok) {
    drawBleStatus("CONNECT FAILED");
  }
  return ok;
}

// Touch handling
static void handleTouch()
{
  TOUCHINFO ti;
  if (!touch.getSamples(&ti)) {
    g_touchDownPrev = false;
    return;
  }

  bool touchDown = (ti.count > 0);
  if (!touchDown) {
    g_touchDownPrev = false;
    return;
  }

  int rawX = ti.x[0];
  int rawY = ti.y[0];
  int x, y;
  mapTouch(rawX, rawY, x, y);

  bool isNewPress = touchDown && !g_touchDownPrev;

  if (isNewPress) {
    if (x >= ARM_X && x <= ARM_X + ARM_W &&
        y >= ARM_Y && y <= ARM_Y + ARM_H) {
      g_armed = !g_armed;
      drawArmButton();
      g_touchDownPrev = touchDown;
      return;
    }

    if (x >= MODE_X && x <= MODE_X + MODE_W &&
        y >= MODE_Y && y <= MODE_Y + MODE_H) {
      g_mode++;
      if (g_mode > VESC_MODE_RPM) g_mode = VESC_MODE_CURRENT;
      vesc.setControlMode(g_mode);
      drawModeButton();
      g_touchDownPrev = touchDown;
      return;
    }
  }

  if (y >= SLIDER_TOP && y <= SLIDER_TOP + SLIDER_HEIGHT) {
    float v = (float)(SLIDER_TOP + SLIDER_HEIGHT - y) / (float)SLIDER_HEIGHT;
    v = clamp01(v);

    if (x >= SLIDER_X_THROTTLE && x <= SLIDER_X_THROTTLE + SLIDER_WIDTH) {
      g_throttle01 = v;
      drawSlider(SLIDER_X_THROTTLE, g_throttle01, false);
    } else if (x >= SLIDER_X_BRAKE && x <= SLIDER_X_BRAKE + SLIDER_WIDTH) {
      g_brake01 = v;
      drawSlider(SLIDER_X_BRAKE, g_brake01, true);
    }
  }

  g_touchDownPrev = touchDown;
}

// Command-Berechnung
static void updateCommand()
{
  if (!g_armed) {
    vesc.setCommand(0.0f);
    return;
  }

  float cmd = 0.0f;

  switch (g_mode) {
    case VESC_MODE_CURRENT: {
      float up = g_throttle01 * MAX_CURRENT_A;
      float dn = g_brake01    * MAX_CURRENT_A;
      cmd = up - dn;
    } break;

    case VESC_MODE_DUTY: {
      float up = g_throttle01;
      float dn = g_brake01;
      cmd = clamp01(up - dn);
    } break;

    case VESC_MODE_BRAKE: {
      cmd = g_brake01 * MAX_CURRENT_A;
    } break;

    case VESC_MODE_RPM: {
      float up = g_throttle01;
      cmd = up * 15000.0f;
    } break;
  }

  vesc.setCommand(cmd);
}

void setup()
{
  Serial.begin(115200);
  delay(200);

  tft.init();
  tft.setRotation(TFT_ROTATION);
  tft.fillScreen(TFT_BLACK);

  drawStaticUI();
  drawArmButton();
  drawModeButton();
  drawSlider(SLIDER_X_THROTTLE, g_throttle01, false);
  drawSlider(SLIDER_X_BRAKE,    g_brake01, true);
  drawBleStatus("BOOTING");

  touch.init(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);

  vesc.begin();
  vesc.setControlMode(g_mode);
  vesc.setSendIntervalMs(50);
  vesc.setIdleZeroMs(150);
  vesc.setIirAlpha(0.20f);
  vesc.setSlewAperSec(4.0f);
  vesc.setDeadbandPct(0.06f);

  vesc.onTelemetry(onVescTelemetry);
  vesc.onConnect(onVescConnect);
  vesc.onDisconnect(onVescDisconnect);

  connectVesc();
  g_lastReconnectMs = millis();
}

void loop()
{
  uint32_t now = millis();

  vesc.loop();
  handleTouch();
  updateCommand();

  if (g_telPending) {
    noInterrupts();
    VescBleRxTx::VescTelemetry t = g_lastTel;
    g_telPending = false;
    interrupts();
    drawTelemetry(t);
  }

  if (!vesc.isConnected() && (now - g_lastReconnectMs) > RECONNECT_MS) {
    g_lastReconnectMs = now;
    connectVesc();
  }

  delay(10);
}
