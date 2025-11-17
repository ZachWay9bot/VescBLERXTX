#include <Arduino.h>
#include <vescblerxtx.h>

VescBleRxTx vesc;

void onTelemetry(const VescBleRxTx::VescTelemetry& t) {
  Serial.print("[TEL] U=");
  Serial.print(t.voltage);
  Serial.print(" V  I_in=");
  Serial.print(t.current_in);
  Serial.print(" A  km/h=");
  Serial.print(t.speed_kmh);
  Serial.print("  fault=");
  Serial.println(t.fault_code);
}

void onConnect() { Serial.println("VESC BLE connected."); }
void onDisconnect() { Serial.println("VESC BLE disconnected."); }

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nvescblerxtx SimpleDemo");

  vesc.begin();
  vesc.onTelemetry(onTelemetry);
  vesc.onConnect(onConnect);
  vesc.onDisconnect(onDisconnect);

  vesc.setControlMode(VESC_MODE_CURRENT);
  vesc.setSendIntervalMs(50);
  vesc.setIdleZeroMs(150);
  vesc.setIirAlpha(0.20f);
  vesc.setSlewAperSec(4.0f);
  vesc.setDeadbandPct(0.06f);

  bool ok = vesc.connectByName("VESC_BLE");
  if (!ok) {
    Serial.println("connectByName() failed, check name / power / distance.");
  }
}

void loop() {
  vesc.loop();

  static uint32_t lastCmdMs = 0;
  uint32_t now = millis();
  if (now - lastCmdMs > 100) {
    lastCmdMs = now;
    if (vesc.isConnected()) {
      vesc.setCommand(0.25f);
    }
  }
}
