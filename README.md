# TUF-2000 ultrasonic liquid flow meter, ESP32 library

An ultrasonic clamp-on flow meter — no pipe-cutting install, reads flow rate and totalized volume over Modbus RTU on RS-485. It also carries its own onboard relay (OCT), configurable locally on the meter to trip on a flow-rate or totalizer threshold — that relay is meant to drive the shut-off valve directly as a hardware failsafe (see below), independent of the ESP32/Wi-Fi being alive.

[Original flow meter documentation](https://images-na.ssl-images-amazon.com/images/I/91CvZHsNYBL.pdf).

TUF-2000 is produced in different form-factors. The form-factor defines the last letter after the `2000`. All these models have the same functionality, features, and bugs :) Hence this library fits all TUF-2000 models with all compatible ultrasonic sensors.

## Usage

Install the library (`lib_deps = igrowing/tuf2000`), include `<Tuf2000.h>` and read the meter. The simplest form is a blocking read, from [examples/basic](examples/basic/basic.ino):

```cpp
#include <Tuf2000.h>

tuf2000::Tuf2000FlowMeter meter;

void setup() {
    Serial.begin(115200);

    tuf2000::Tuf2000FlowMeter::Config cfg;
    cfg.rxPin = 17;   // ESP32 RX <- converter TXD
    cfg.txPin = 16;   // ESP32 TX -> converter RXD
    cfg.slaveId = 1;  // must equal the meter's M46

    if (!meter.begin(Serial2, cfg)) {
        Serial.println("meter.begin() failed");
    }
}

void loop() {
    tuf2000::Tuf2000FlowMeter::Reading r;
    if (!meter.read(3000, r)) {
        Serial.println("no reply within 3 s");
    } else if (r.valid) {
        Serial.printf("flow %.3f m3/h, total %.3f m3\n", r.flowRateM3h, r.totalizerM3);
    } else {
        Serial.printf("poll failed: %s\n", tuf2000::tuf2000FailureText(r.failure));
    }
    delay(5000);  // read() starts a new poll only once cfg.pollIntervalMs (default 5 s) has passed
}
```

Expected serial output, one line every 5 seconds (values depend on your meter):

```
flow 0.000 m3/h, total 2.680 m3
flow 0.412 m3/h, total 2.681 m3
```

With the RS-485 cable unplugged each poll is rejected:

```
poll failed: modbus_error_or_timeout
```

Readings resume on the next poll after the cable is reconnected.

`read()` blocks the calling task, so use it only in bench sketches. Real firmware should use `readAsync()`/`takeReading()`: [examples/async](examples/async/async.ino) shows that polling next to other work in `loop()`, with `responseReady()`, an optional log callback (`setLogger()`), a failure counter and stale-data detection.

### Behaviour worth knowing

- A poll is **all or nothing**: a Modbus error, a wrong-length reply, a NaN/infinity value or an implausible totalizer jump makes the whole `Reading` invalid (`valid == false`, with the reason in `failure`). Nothing is clamped or defaulted.
- 32-bit values (flow rate, totalizer, sound speed) arrive **low word first**.
- Modbus protocol addresses are the manual's register numbers minus one. Defaults in `Config`: flow rate REG1-2, sound speed REG7-8, signal quality/up/down strength REG92-94, positive totalizer (m3) REG115-116. Override them in `Config` for other firmware variants.
- `calibrationMultiplier` scales flow rate and totalizer if your meter's calibration is off. `maxTotalizerJumpM3` (default 10) is the largest forward step accepted between polls.
- `lastErrorCode()`/`lastFailure()` are sticky, so an intermittent fault stays visible after a good poll.

## Flow meter installing and configuration

This is the must to do part **BEFORE** connection the flow meter to MCU. The meter talks Modbus RTU over RS-485 bus as slave at 9600 8N1.

### Flow meter settings (set on the meter's keypad)

| Menu | Setting | Required value | Note |
|--|--|--|--|
| M11 | Pipe outer diameter | 44 | Measure yours |
| M12 | Pipe thickness | 9 | Measure yours |
| M14 | Pipe material | 9 (other)| For plastic green pipes |
| M15 | Sound speed in pipe material | 2650 | Not needed to change is non-"other" known pipe material is used |
| M16 | Pipe liner | No Liner | Or select needed liner |
| M18 | Pipe inner diameter |  | Just see that meter calculated your pipe correctly, having pipe OD and thickness (M11, M12) |
| M20 | Liquid type | 0 (water) |  |
| M23 | Transducer type | 19 (TS-2) | Choose your transducer type |
| M24 | Transducer mounting | V or W | Choose what suits better your pipe. For very small pipes W works better. For 1" and up OD the V is good enough |
| M25 | Distance between transducers |  | See for easy of adjustment |
| M63 | Communication protocol | **MODBUS RTU** | Factory default is MODBUS ASCII, which ignores RTU frames completely. Forgetting this was the cause of the first bring-up timeout. |
| M62 | RS-232/RS-485 setup | 9600 baud, parity none, 8 data bits, 1 stop bit | Factory default. Any other parity makes the meter drop every frame silently. |
| M46 | Network address (ID) | 1 | Must equal `Config::slaveId`. |
| M26 | Save settings (Solidify) | run after changing the above | Settings live in RAM until stored. Without this step they revert on the next power cycle (manual, page 4). |

### Placing and adjustment of transducers
1. Decide the water direction. Put the "Up" transducer on a side from where the water runs. Put the "Down" transducer on a side where the water runs.
2. Use provided with the meter silicon-like tighthening cream between the transducer and the pipe to make air-less tight placement.
3. Using menu M90 move the transducers slightly around the calculated distance from M25 to read on the display Q value between 60 and 90. More is better. After every slight position change let the meter 3-5 seconds to show you the new Q value. When you reached best possible Q, tighten the transducers with provided rings.

#### Diagnosing a silent bus
Menu M49 on the meter displays the raw bytes it receives on the serial port. If bytes show up every couple of seconds, wiring and converter are fine and the fault is in M63/M62/M46. If M49 stays empty, look at the physical link (A/B, termination, converter). On the converter, the R LED blinks for each request coming from the ESP32 and the T LED blinks for each reply going back to it.

#### Meter adjustment
Meter might count water flow even on still water. And it may count the flow wrongly.

1. To set 0 liquid flow, use menu M42. Activating it, wait 30 seconds: do not touch the meter and make 100% sure the water is NOT running.
2. To calibrate the water flow measurement use menu M45. The scale factor is expected/actual.

### RS-485 GPIOs

Any two free GPIOs work for the UART. Avoid boot-strapping pins and the pins used by `Serial`/flashing, and pass the pins you picked in `Config::rxPin`/`txPin`. The example uses `Serial2` on GPIO16 (TX) and GPIO17 (RX):

| Signal | Example pin |
|--|--|
| ESP32 TX → converter RXD | 16 |
| ESP32 RX ← converter TXD | 17 |
| DE/RE | — (unused/automatic; set `Config::dePin` if your transceiver needs it) |
| GND | GND |
| VIN | 3V3 |

### RS-485 converter: Waveshare TTL TO RS485 (C)

Picked over a plain MAX485-style breakout for two reasons:
- **Galvanic isolation.** If the cable run parallels mains wiring, this module isolates the RS-485 bus side (both power and signal, via an onboard isolated DC-DC plus a digital isolator) from the ESP32's own logic and ground, so an induced spike on the bus has no metallic path to the MCU even if it gets past the onboard TVS clamp first.
- **Automatic direction switching**, built into the chip — no DE/RE control needed, so the link only spends 2 GPIOs instead of 3.

Also has onboard 120Ω termination (enabled by soldering a jumper) and 200W surge / 6kV ESD protection on the bus side. TTL-side pinout: VCC / TXD / RXD / GND, wired to the ESP32's 3.3V / RX pin / TX pin / GND (crossed: the converter's RXD receives what the ESP32 transmits, and its TXD feeds the ESP32's RX pin); RS-485 side: A+ / B- / PE.

### Wiring the RS-485 cable

- Shielded twisted pair for A/B.
- Land the shield on the converter's **PE** terminal, at the ESP32 end only — don't also ground it at the flow meter end (that turns the shield into a ground-loop path), and don't bridge PE to the TTL-side GND (that defeats the isolation).
- No common GND wire: the meter's RS-485 port is itself isolated, and its GND terminal is only the transducer ground ("NOT for power and AO and RS485", manual page 8). Two isolated ports need only the A/B pair; this link works that way.

### Why the 120Ω terminator matters

RS-485 is a shared differential line; an un-terminated cable end reflects a fast signal edge back down the wire instead of absorbing it, corrupting bytes — worse at higher baud rates or longer runs. A 120Ω resistor at each of the two *physical ends* of the bus (matched to the cable's characteristic impedance) soaks up the edge instead of reflecting it. Enable it only at the two endpoints, never at an intermediate device on a multi-drop run — extra resistors along the way just overload the driver and attenuate the signal for everyone.

### Room for more RS-485 devices later

RS-485/Modbus is multi-drop: a shut-off valve controller or any other Modbus RTU device can join the same A/B pair later with its own slave ID — no extra ESP32 pins or a second converter needed, they all share this one bus.

### Shut-off valve strategy

Prefer the TUF-2000M's own relay, configured locally (flow-rate or totalizer threshold), as the failsafe that closes a master shut-off valve even when the ESP32 or Wi-Fi is down: don't trust the network for safety-critical shut-off. Poll flow readings over Modbus in firmware as well, for reporting/logging and for application-aware logic the meter alone can't know (e.g. unusually high volume mid-cycle for whichever zone is currently running).
