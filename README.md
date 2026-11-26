# Aero A240 for Microsoft Flight Simulator 2024

A simple community aircraft modelled on the Aero A240 (the tricycle-gear Aero
EuroFox): two seats, strut-braced folding high wing, full-span flaperons, a 141 hp
turbocharged Rotax 915 iS and Aeropro's current Garmin G3X Touch instrument panel.

It is a native MSFS 2024 ("modular") aircraft, built with the SDK's package tool. The 3D
model lives in `blender\A240.blend` and is exported with the SDK's Blender exporter.

![Model preview](previews/A240_sheet.png)

## Status

| Part | State |
| --- | --- |
| SDK package build | Builds with no problems reported by the SDK builder |
| 3D model (exterior + cockpit) | Detailed Blender model (~99k triangles). The instrument panel is traced from Aeropro's 2025 render of its Garmin panel (see [Instrument panel](#instrument-panel)). Seams checked for VR with `check_vr_gaps.py` (see [Check for VR gaps](#check-for-vr-gaps)). **Not yet built with the SDK or seen in the sim** - the earlier low-detail model was |
| Flight / engine / systems configs | Written from published A240 figures. Controls and hands-off pitch stability were **checked in the sim** with the old Rotax 912 ULS; the switch to the 915 iS (turbo, heavier nose) is **not yet flown** |
| Cockpit display | The sim's own Garmin G3X Touch (Working Title, v2) with a Rotax 915 iS engine strip. **Not yet seen in the sim.** The package's simpler EFIS (checked in the sim with live data, and viewable in a browser) is kept as a fallback - see `panel.cfg` |
| Sound | **None yet** - the aircraft is silent (see below) |
| In-simulator test | Engine run-up and basic handling at 80-100 kt tested; flaps, stalls, takeoff and landing **not yet** |

## Build

Needs the MSFS 2024 SDK and Blender 5.2 (see [SDK_TOOLS.md](SDK_TOOLS.md)). Close the
simulator first.

```
powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1
```

This exports the models from `blender\A240.blend`, then runs the SDK package tool, which
starts the simulator in build mode for about half a minute. The result is
`sdk-project\Packages\local-aircraft-a240`. Add `-SkipModel` to rebuild after a config
change only, `-Rebuild` to ignore the build cache, or `-Previews` to re-render the
preview images and preset thumbnails too. If Blender is not in
`C:\Program Files\Blender Foundation\Blender 5.2`, set `$env:BLENDER` to its `blender.exe`.

## Edit the model

Open `blender\A240.blend` in Blender 5.2 and edit it like any model. To see and change
the MSFS material settings in Blender, enable the SDK's exporter once (Edit > Preferences >
File Paths > Script Directories > add `C:\MSFS 2024 SDK\Tools\Blender`, then enable
"Microsoft Flight Simulator 2024: glTF Extension"). The build scripts load the exporter
from the SDK themselves, so they work without that step.

To export just the models (and optionally previews) without the SDK build:

```
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b blender\A240.blend --python-exit-code 1 --python tools\blender\export_a240.py -- --previews --cockpit
```

The export checks that the six control-surface animations, the 15 cockpit-control
animations, the `PROPELLER` / `PROPELLER_Blur` nodes and the `EFIS_Display` screen material
are all still there, and fails if not. `--cockpit` also renders cockpit and close-up views into `previews\`.

### Check for VR gaps

```
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b blender\A240.blend --python-exit-code 1 --python tools\blender\check_vr_gaps.py
```

In VR you lean into every corner of the cockpit, so cracks that a fixed camera never shows
become obvious. The sim draws both models in the cockpit and does not draw the back faces
of single-sided materials, so a crack shows whatever is behind it, often daylight. The
check traces rays the way the sim draws, from about 370 head positions around both seats,
and fails on any ray that sees through a seam. It also lists parts that float free of
everything else, and cabin parts that poke out through the skin. It runs with the moving
controls centred and at both ends of their travel, takes about 15 seconds, and is worth
running after any edit to the model.

The generator keeps the model gap-free like this. Keep to the same rules when editing:

- The cabin lining has a reveal at every window: a strip from the lining's edge out to the
  skin. The bulkheads are a little larger than the lining, and the lining runs into them.
- Cabin parts that meet the walls (floor, panel backing, glareshield, baggage shelf) end
  1.5 mm past the lining (`wall_x()`), measured on the mesh's polygon. Near the belly the
  polygon lies several mm inside the smooth curve.
- Fittings are set a millimetre or two into whatever they are mounted on, never just
  short of it.

Things the sim depends on - keep these names when editing:

- Collections `Exterior` (exported as `A240.gltf`) and `Interior` (`A240_interior.gltf`);
  the export presets point at them.
- `SKEL_Flap_L/R` (empties) with `SKEL_Aileron_L/R` under them, `SKEL_Elevator`,
  `SKEL_Rudder`, each with its `*_percent_key` action; keep their origins on the hinge line.
- `PROPELLER` (blades) and `PROPELLER_Blur` (disc), origins on the propeller axis. The
  spinner stays outside these, or it would vanish with the blades at high RPM.
- `EFIS_Screen` with material `EFIS_Display` (the G3X Touch is painted onto it, using its
  UVs). It is the GDU 460's 5:3 screen, 230.9 x 138.5 mm, to match the 1280 x 768 display.
- The cockpit controls, each with its origin on its pivot and its action named as in
  `model/A240_Interior_Behavior.xml`, which moves them with the sim:
  - `HANDLING_Stick_Roll_Pilot/Copilot` (empties) with `HANDLING_Stick_Pitch_*` under them
  - `HANDLING_Pedal_L/R_*` with `HANDLING_ToeBrake_L/R_*` under them
  - `THROTTLE_Knob` (slides), `HANDLING_Flap_Lever` and `HANDLING_Trim_Lever`

`tools\blender\make_a240_blend.py` is the script that built the .blend from scratch. Once
you have edited the .blend by hand, do not re-run it - it overwrites the file.

## Install

```
powershell -ExecutionPolicy Bypass -File tools\install.ps1
```

This links the built package into the sim's Community folder. Restart the sim and pick
**Aero A240** (manufacturer *Aero*). Add `-Remove` to uninstall. Because it is a
link, later builds show up after a sim restart with no reinstall.

## Look at it without the sim

```
py -m http.server 8240 --bind 127.0.0.1
```

- <http://localhost:8240/viewer/index.html> - orbit the model, move the control surfaces,
  sit in the cockpit
- <http://localhost:8240/viewer/efis.html> - the cockpit display running on demo data

## Layout

```
sdk-project/                              the SDK project
  A240_Project.xml                        open this in the sim's Project Editor
  PackageDefinitions/local-aircraft-a240.xml
  PackageSources/
    SimObjects/Airplanes/Local_A240/
      common/config/                      aircraft, flight_model, engines, systems,
                                          cameras, cockpit .cfg
      common/model/                       exported glTF + behaviour XML (animations)
      common/panel/panel.cfg              puts the G3X Touch on the "EFIS_Display" surface
      common/panel/panel.xml              G3X Touch setup: sensors, COM, transponder, engine strip
      common/effects/                     light effects (from the SDK sample)
      presets/local/A240/                 the selectable variant + thumbnails
    html_ui/.../LocalA240/EFIS/           the older, simpler cockpit display (fallback)
  Packages/                               build output (created by the build)
blender/
  A240.blend                              the 3D model (source)
tools/
  blender/export_a240.py                  .blend -> glTF, plus preview renders
  blender/make_a240_blend.py              built the .blend from scratch (don't re-run)
  blender/a240_lib.py                     shared helpers (coordinates, materials)
  build_sdk.ps1                           models + SDK package build
  legacy/build_model.py                   the old low-detail procedural model (reference)
  install.ps1                             links the build into Community
viewer/                                   browser previews
package/                                  the earlier hand-built package (see below)
```

`package/` is the first, hand-assembled version in the older single-folder layout. It is
no longer updated by the tools and can be deleted; it is kept only as a fallback.

## Instrument panel

The panel is Aeropro's current factory panel for the Rotax 915 iS, as shown on
[aeropro.sk/eurofox](https://www.aeropro.sk/eurofox/) (image `panel-garmin-1a`, April 2025).
That is the newest configuration and the one with the most glass. A local copy of that image and of
the older panels is kept in `reference/` (git-ignored, as the images are Aeropro's and AeroTrek's). The AeroTrek panel
(iPad, AvMap EFIS, steam gauges) on aerotrek.aero dates from 2018. Left to right:

- Battery backup (guarded), Lane A/B and Pump A/B (guarded), test button, fuel-reserve light
- START button and slip ball above a 10.6 in **Garmin G3X Touch** (GDU 460)
- Standby airspeed (kt), vertical speed (m/s) and altimeter across the top
- Garmin COM; switches for landing light, position light, strobe, COM, transponder, FLARM,
  GDU 460, a spare and USB; the Engine/Avionics key; a Trig transponder
- Pre-flight and Rotax 915 iS limits placards, 12 V and USB sockets, two columns of
  breakers and the map box on the right
- Below the panel: the vernier throttle ("idle PULL"), cabin heat and the yellow lever

The render was scaled from two parts of known size, the GDU 460 bezel (10.85 x 7.82 in)
and the 6.25 in Garmin COM, which agree at 1.83 px/mm. The panel is 0.91 m wide and
0.28 m tall. In `build_panel()` every item is placed with `pp(x, d)`: mm right of the
centreline (as the pilot sees it) and mm down from the top centre of the panel, read
straight off that image. Switches, labels and placards are shapes only: there is no text,
the needles do not move and nothing is clickable yet. The sticks, rudder pedals and toe
brakes, throttle knob, flap lever and trim lever move with the sim's controls.

## Coordinates

One origin is used everywhere: on the centreline, at the wing quarter-chord, at
propeller height. The model is in metres, the `.cfg` files in feet
(`z` forward, `x` right, `y` up). In Blender the nose points along -Y and the left wing
along +X (Blender is Z-up), which the exporter turns back into the sim's frame. If you
move something in the model (wheels, wing, tail, lights), update the matching numbers in
`flight_model.cfg` / `systems.cfg`.

Control-surface directions follow the SDK's own DA62 sample, so those should be right.
