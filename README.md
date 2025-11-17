# vescblerxtx

**vescblerxtx** is a small Arduino C++ library for controlling a VESC (or compatible ESC) over BLE using a simple **int32 big-endian ×1000 command stream** and receiving **ASCII telemetry** (`key=value` lines).

It is designed as the BLE “transport core” for things like:

- Touch throttles
- DIY dashboards
- Scooter bridge projects
- Remote controls and test rigs

The library does **not** parse the raw VESC binary protocol directly.  
Instead, it expects a BLE target that:

1. Exposes a Nordic UART Service (NUS) or compatible RX/TX characteristics.
2. Interprets incoming 4-byte big-endian `int32` values as a control command.
3. Sends human-readable telemetry lines like:

   ```text
   volt=50.3,erpm=12345,duty=0.13,ain=1.2,amot=3.4,kmh=25.0,batt=82,...
The library then:

Handles BLE connect / reconnect / notify.

Applies deadband, IIR filtering, slew rate limiting, idle-zero safety.

Parses telemetry into a rich VescTelemetry struct.

Features
 
 BLE client for Nordic UART Service (NUS)
 
 Connect by device name or MAC address

 Command modes: CURRENT, BRAKE, DUTY, RPM

 Simple command encoding: float → int32 BE × 1000


Built-in filters:

Deadband

IIR low-pass

Slew rate limiter

Idle zero timeout

Telemetry callback with a packed struct:

Voltage, ERPM, duty

Currents, power/energy (Ah/Wh)

Temperatures (MOSFET, motor, extended)

Speed, tacho, position

Battery %, Wh, fault code, controller ID, etc.

Raw notify callback for debugging/logging


use it at your own risk!
