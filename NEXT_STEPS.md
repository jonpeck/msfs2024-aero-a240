# Next steps

Improvements to the A240, most valuable first. Step 0 comes before everything else,
because the newest changes have not been in the sim yet.

| # | Item | Value | Effort | Can be checked without the sim |
| --- | --- | --- | --- | --- |
| 0 | Verify the G3X panel and Rotax 915 iS | Required | Small | No |
| 1 | Moving controls | High | Small | Done; directions to confirm in the sim (step 0) |
| 2 | Live standby gauges | High | Small-medium | Needle pivots, in renders |
| 3 | Clickable switches | High | Medium | No |
| 4 | Text on labels and placards | Medium | Small-medium | Yes, in renders |
| 5 | Working G3X bezel and touch | Medium | Medium | No |
| 6 | Sound | Very high | Large | No (needs Wwise on Windows) |
| 7 | Modular electrical system | Medium | Medium | No |
| 8 | Exterior textures and registration | Medium | Medium | Yes, in renders |
| 9 | Opening doors and folding wings | Low-medium | Medium | Animation, in renders |
| 10 | Flight model tuning | High | Medium | No |
| 11 | Career mode and checklists | Low | Small-medium | No |

**Suggested order:**

1. Step 0.
2. Items 1-3 as one cockpit pass, with item 4 alongside.
3. Item 10, once the 915's power is settled.
4. Item 7, which gives the switches from item 3 something real to do.
5. Item 6.
6. Items 8, 9 and 11, as time allows.

"Checked without the sim" means it can be built and rendered with Blender alone, as the
panel was. Anything that depends on the sim's behaviour needs a Windows build and a
flight.

---

## 0. Verify the G3X panel and Rotax 915 iS

Follow [VERIFY_ON_WINDOWS.md](VERIFY_ON_WINDOWS.md). These values are estimates until
then:

- engine: `power_scalar`, `max_design_mp`, `fixed_pitch_beta` and `compression_ratio`
- the MAN IN marks in `panel.xml`
- pitch trim after the heavier engine moved the CG forward

## Cockpit

### 1. Moving controls

**Done, not yet seen in the sim.** `build_moving_controls()` in `make_a240_blend.py`
makes each control its own node on its real pivot, keyed 0-100:

- stick: roll and pitch about its base, both seats
- rudder pedals about their floor hinge, toe brakes tilting the pedal faces
- throttle knob sliding out 5 cm to idle ("idle PULL")
- flap lever over a three-notch quadrant, and the elevator trim lever beside it

`model/A240_Interior_Behavior.xml` drives the keys from the sim. The directions of the stick
and trim rest on SimVar signs the SDK docs don't state: confirm them with the "Moving
controls" checks in `VERIFY_ON_WINDOWS.md`.

Done when the cockpit controls follow the sim's controls, with the right direction and
travel.

### 2. Live standby gauges

Make each needle its own node, rotating about its gauge's centre:

| Gauge | Needles | Drive from | Notes |
| --- | --- | --- | --- |
| Airspeed | 1 | `AIRSPEED INDICATED` (kt) | Not linear: `asi_angle()` in `make_a240_blend.py` gives the dial |
| Vertical speed | 1 | `VERTICAL SPEED` | ±10 m/s, 0 at 9 o'clock |
| Altimeter | 2 or 3 | `INDICATED ALTITUDE` | Baro knob turns `KOHLSMAN SETTING` |
| Slip indicator | ball | `TURN COORDINATOR BALL` | Ball slides along its tube |

Done when the standby gauges agree with the G3X.

### 3. Clickable switches

Give each switch its own node with a toggle animation and a click area, and wire it to the
sim. Where the simple electrical system has no matching function, the switch is cosmetic
until item 7.

| Control | Does |
| --- | --- |
| Battery backup (guarded) | Battery master, or the backup battery after item 7 |
| Engine/Avionics key | Avionics master |
| START button | Starter |
| Lane A / Lane B (guarded) | Ignition: the two lanes stand in for left and right magnetos |
| Pump A / Pump B (guarded) | Electric fuel pump(s) |
| Landing light, position light, strobe | Lights |
| COM, TXP, GDU 460 | COM, transponder and G3X power (avionics circuits) |
| FLARM, USB, spare | Cosmetic |
| Test button | Test lights |
| Breakers | Circuit breakers (after item 7) |
| Throttle knob | Drag it in and out |
| Cabin heat knob | Pull |

Done when you can start and run the aircraft from cold and dark with the mouse only.

### 4. Text on labels and placards

The labels and placards are blank plates with grey lines standing in for text.

- Generate one texture atlas (for example with Pillow) holding every label from Aeropro's
  panel image: switch names, OFF, Lane A/B, Pump A/B, the throttle and heat labels,
  breaker ratings, the pre-flight check, the Rotax 915 iS limits and the passenger
  warning.
- UV-map the plates onto it.
- This is the project's first texture. Check that the exporter and the package build
  handle it: the texture folder, and whether a `texture.cfg` is needed.
- Optionally use the same atlas for the gauge dial faces (numbers, KNOTS, M/S, ALT).

Done when a render reads like the factory image.

### 5. Working G3X bezel and touch

The G3X screen runs, but the knobs and buttons are dead and mouse clicks on the screen may
not register. There are two routes:

- **A (documented, preferred):** use the sim's `Asobo_MPA_G3X_Touch` attachment, which
  brings its own bezel model, knobs, buttons, sounds and backlight.
  - Remove our bezel from `build_g3x()`.
  - Place the attachment at the G3X position: centre 229.5 mm left of the centreline and
    166.4 mm below the panel top, on the panel face at z = 0.50 m.
  - Configure it in the preset's `attached_objects.cfg`, with
    `vcockpit_parameter.0 = "VCockpit01_htmlgauge00_params,?Index=1"`.
- **B:** keep our bezel. Add an interactive screen, and the legacy `WT_G3XTouch_Template`
  for the knobs. The docs call that template "not recommended", and it cannot be used with
  compiled behaviours.

Docs: <https://microsoft.github.io/msfs-avionics-mirror/2024/docs/avionics/g3xtouch/overview>

## Aircraft

### 6. Sound

The aircraft is silent. MSFS 2024 does not let a native aircraft borrow another aircraft's
sounds, so it needs its own sound package, built with Audiokinetic Wwise and the SDK's
sample Wwise project.

- Minimum: engine loops (idle to full power, inside and outside), starter, wind and
  rolling.
- Then: switch clicks, flaps and the stall warning.

### 7. Modular electrical system

`systems.cfg` uses the simple MSFS 2020-style description (`[ELECTRICAL]` `Major = 1`).
Move to the modular system:

- a main bus
- the battery plus the panel's backup battery
- the alternator
- one circuit per breaker (the panel has 13: 2 A, 3 A, 5 A and 10 A)

That gives the 915 iS switches real effects:

- the engine stops with both lanes off
- it needs a fuel pump running
- the battery backup keeps the engine computer alive if the main battery drops

### 8. Exterior textures and registration

Every surface is a flat colour. In order:

1. Registration marks.
2. Panel lines and rivets as a normal map.
3. Optional liveries matching real A240s and Eurofoxes.

### 9. Opening doors and folding wings

- Gull-wing doors on gas struts, which can be removed in the real aircraft.
- The Eurofox's folding wings: a parked pose, at least.

### 10. Flight model tuning

Flaps, stalls, takeoff and landing are still untested. Aeropro's figures to tune against:

| Figure | Value |
| --- | --- |
| VS0 (flaps) | 66 km/h (35.6 kt) |
| VS1 (clean) | 80 km/h (43 kt) |
| VFE | 150 km/h (81 kt) |
| VNO | 175 km/h (94.5 kt) |
| VNE | 230 km/h (124 kt) |
| Take-off ground roll at MTOW | 139 m |
| Landing ground roll at MTOW | 130 m |
| Glide ratio | 9:1 |

Climb and cruise targets need updating for the 915 iS. The README's ~100 kt and
~1000 fpm are 912 figures.

### 11. Career mode and checklists

- `targeted_specializations` in the preset's `aircraft.cfg`, with the payload and
  passenger setup that goes with it.
- An in-sim checklist (`kb_checklists`) covering cold-and-dark start, run-up, before
  takeoff and shutdown, using the 915 iS procedure: lanes, pumps and START button.
