vescblerxtx

Kleine, knackige ESP32-Arduino-Lib für VESC-Steuerung per BLE (Nordic-UART/NUS) – optional mit Telemetrie. Fokus: niedrige Latenz, simple API, sauberer Sollwert-Pfad (Current, Brake, Duty, RPM) mit eingebauter Safety.

Zielplattform: ESP32 / ESP32-C3 / ESP32-C6 (Arduino Core ≥ 2.0)
Transport: BLE (NUS-Service als Standard)
Payload: int32 Big-Endian, Skala ×1000 (v1.0.4)
Modi: CURRENT, BRAKE, DUTY, RPM (umschaltbar)
Safety: Deadband, Slew-Limiter, IIR-Glättung, Idle-Timeout → sendet 0

features

Connect per MAC oder Name (NUS-UUIDs vordefiniert, bei Bedarf überladbar)

Periodisches Senden im festen Raster (Default 50 ms)

Eingangsfilter: IIR, Slew-Limit, Deadband

Failsafe: nach IDLE_ZERO_MS ohne Updates → Befehl = 0

Optional: Telemetrie-Callback (Volt, ERPM, Duty, Ströme, Temperaturen), falls die Gegenseite sendet

installation

Ordner vescblerxtx/ nach ~/Documents/Arduino/libraries/ kopieren
(enthält src/vescblerxtx.h, src/vescblerxtx.cpp, library.properties …)

Arduino IDE neu starten.

Abhängigkeit: ESP32 BLE Arduino oder NimBLE-Arduino (je nach Implementierung der Lib).
Hinweis: Bei NimBLE keine zweite BLE-Lib parallel aktiv halten.

quickstart
#include <Arduino.h>
#include <vescblerxtx.h>

VescBleRxTx vesc;

void setup() {
  Serial.begin(115200);

  // optional: Telemetrie anhängen
  vesc.onTelemetry([](const VescTelemetry& t){
    // Serial.printf("V:%.2f  ERPM:%.0f  Duty:%.3f\n", t.voltage, t.erpm, t.duty);
  });

  vesc.begin();                        // NUS-UUIDs sind Default
  vesc.connectByName("VESC_BLE");      // oder: vesc.connectByMac("AA:BB:CC:DD:EE:FF");

  vesc.setControlMode(VESC_MODE_CURRENT); // CURRENT / DUTY / RPM
  vesc.setSendIntervalMs(50);             // periodischer Output
  vesc.setIdleZeroMs(150);                // Failsafe-Fenster
  vesc.setIirAlpha(0.20f);                // 0..1 (höher = weniger Glättung)
  vesc.setSlewAperSec(4.0f);              // A/s bei CURRENT
  vesc.setDeadbandPct(0.06f);             // neutraler Bereich um 0
}

void loop() {
  vesc.loop(); // BLE-Housekeeping + Timing für periodisches Senden

  // Beispiel: 1.2 A Sollstrom (abhängig vom Modus)
  if (vesc.isConnected()) {
    vesc.setCommand(1.2f);
  }
  delay(10);
}

kompatibilität

Getestet: ESP32, ESP32-C3, ESP32-C6 Devkits / „Super Mini“

UART parallel nutzbar (BLE ist unabhängig)

NUS-Service: 6E4000xx-… (siehe PROTOCOL.md), bei Bedarf via begin(...) übersteuerbar

sicherheit & haftung

Sinnvolle Limits setzen: Max-Strom, Max-Brake, Duty-Cap, RPM-Cap.

Im VESC zusätzlich Undervoltage/Speed-Caps aktivieren.

Erst ohne Last testen. Nutzung auf eigenes Risiko.
