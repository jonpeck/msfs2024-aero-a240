# Verifying on Windows

Checklist for the first build and flight after two changes:

- the cockpit rebuilt as Aeropro's G3X Touch panel
- the engine switched to the Rotax 915 iS
- the seams closed for VR: window reveals, bulkheads, floor, panel backing and glareshield
  to the walls, the wing-root fairing, and parts that floated or poked through

These changes were written and test-rendered on Linux, without the MSFS SDK and without
the sim. Nothing below has been checked yet. Run the steps in order, and note anything
that fails for the "What to send back" list at the end.

Close the simulator before steps 1-3. The sim only reads config files when it starts.

## 1. Regenerate the .blend

The generator script is the source of truth. The committed `blender\A240.blend` is
stale: it has neither this panel nor the exterior changes from the commit before it.

```
git pull
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b --factory-startup --python tools\blender\make_a240_blend.py
```

This overwrites `blender\A240.blend`. It loads the SDK's exporter from
`C:\MSFS 2024 SDK\Tools\Blender`. If the SDK is installed somewhere else, set
`$env:MSFS2024_SDK` first.

- [ ] It ends with `saved ...\A240.blend: 28 objects, ~99000 triangles`. The Linux test
      build gave 99,166 triangles.
- [ ] The VR gap check passes on the new .blend:

      ```
      "C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b blender\A240.blend --python-exit-code 1 --python tools\blender\check_vr_gaps.py
      ```

      It ends with `VR gap check passed`, as it did on Linux.

## 2. Export the models and render previews

```
"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe" -b blender\A240.blend --python-exit-code 1 --python tools\blender\export_a240.py -- --previews --cockpit
```

- [ ] No errors. The export checks that the six control-surface animations, the
      `PROPELLER` / `PROPELLER_Blur` nodes and the `EFIS_Screen` screen are present.
- [ ] `previews\cockpit_panel.png` and `previews\cockpit_pilot_view.png` show the new panel.
      Compare them with the factory image, `panel-garmin-1a.jpg` on
      <https://www.aeropro.sk/eurofox/>. Its layout, left to right:
      - G3X Touch on the left
      - three round gauges across the top
      - radio, switch row, key switch and transponder in the middle
      - placards, sockets, breakers and the map box on the right
      - throttle and the yellow lever below
- [ ] `previews\A240_sheet.png`: the exterior from the previous commit looks right. It has
      the swept fin, the tailplane on top of the tail cone and the pointed spinner.

## 3. Build and install

```
powershell -ExecutionPolicy Bypass -File tools\build_sdk.ps1 -SkipModel
powershell -ExecutionPolicy Bypass -File tools\install.ps1
```

`-SkipModel` is used because step 2 already exported the models. `install.ps1` is only
needed the first time; the link picks up later builds.

- [ ] It prints `Built N files with no problems reported`.
- [ ] `sdk-project\Packages\local-aircraft-a240\SimObjects\Airplanes\Local_A240\common\panel\`
      contains both `panel.cfg` and `panel.xml`.

Turn on **developer mode** before starting the sim. Steps 4 and 6 use its tools.

## 4. Cockpit: geometry

Load the A240 on a runway, then once more cold and dark.

- [ ] The panel matches the factory image, from the pilot's seat and from the centre.
- [ ] No flickering surfaces (z-fighting), for example on the labels, the map box or the
      console plate under the throttle.
- [ ] Nothing pokes through anything else: the yellow lever, the throttle and the sticks.
- [ ] The view over the glareshield from the default pilot camera is acceptable. Its top
      over the panel is where it was before. It now reaches the cabin walls and runs
      forward under the windshield to the windshield's base.
- [ ] By day and at night, the black backing beside the panel doesn't look odd.
- [ ] In VR (or by moving the camera around, without a headset), no daylight or
      background shows at any seam, and nothing floats free of the wall. Places to look:
      - lean towards each door window and along the skylight and windshield edges
      - behind the seats, at the edge of the aft bulkhead and the baggage shelf
      - down beside the seats and under the seat backs
      - under the panel into the footwell, and up behind the panel
      - over the glareshield, where it meets the windshield
      - from outside: the wing root, the belly and the door windows

## 5. Cockpit: the G3X Touch

The screen now runs the sim's own Working Title G3X Touch v2. It is configured in
`common\panel\panel.xml`.

- [ ] Cold and dark: the screen is off. It comes on with battery and avionics on
      (circuit `CIRCUIT_AVIONICS`).
- [ ] Runway start: it shows the primary flight display (attitude, airspeed tape,
      altitude tape, heading) and the engine strip along the side.
- [ ] Engine strip, top to bottom:
      - RPM
      - MAN IN (manifold pressure)
      - OIL PRES, OIL TEMP
      - COOLANT, EGT
      - FUEL PRES, FUEL FLOW
      - FUEL QTY (left and right)
      - VOLTS

      All values read sensibly. If the strip is cut off or overlaps, say how.
- [ ] COM 1 frequency and transponder code show on the G3X and change when you tune them
      with keyboard or controller bindings.
- [ ] The picture is not stretched. It should fill the screen inside the bezel exactly,
      at 5:3.
- [ ] Brightness is fine by day and night.
- [ ] Mouse clicks on the screen do or don't work. "Don't work" is expected, because the
      model has no touch-screen behaviour yet.

**If the screen is blank or frozen:**

1. Open <http://127.0.0.1:19999> and pick the `VCockpit...` page.
2. Copy any red errors from its console. A missing or wrong `panel.xml` tag shows up
   there as a JavaScript error.
3. To confirm the model and screen are fine, switch `common\panel\panel.cfg` back to the
   old display. Replace the `htmlgauge00` line with:

   ```
   htmlgauge00 = LocalA240/EFIS/EFIS.html, 128, 0, 1024, 768
   ```

4. Rebuild with `-SkipModel` and restart the sim.

## 6. Engine: Rotax 915 iS

These `engines.cfg` values are estimates:

| Value | Setting |
| --- | --- |
| `power_scalar` | 0.7 |
| `max_design_mp` | 40 inHg |
| `fixed_pitch_beta` | 23 |
| `compression_ratio` | 8.2 |

Read numbers in dev mode with **Tools > SimVar Watcher**:

| Read | SimVar | Unit |
| --- | --- | --- |
| RPM | `GENERAL ENG RPM:1` | rpm |
| Manifold pressure | `RECIP ENG MANIFOLD PRESSURE:1` | inHg |
| Power | `RECIP ENG BRAKE POWER:1` | ask for horsepower; if that fails, use ft-lb/s ÷ 550 |
| Fuel flow | `ENG FUEL FLOW GPH:1` | gph |

On the ground:

- [ ] It starts with Ctrl+E, or with the starter if you have it bound. There is no mixture
      or choke now that it's fuel-injected.
- [ ] Idle RPM is reasonable, around 1400-1800.
- [ ] Full throttle, standing still: note the RPM, manifold pressure, power and fuel flow.
      Aim for about 5200-5500 RPM. Lower `fixed_pitch_beta` for more static RPM, raise it
      for less.

In the air:

- [ ] Full throttle, level, low altitude: about 141 hp at 5800 RPM. RPM must not run far
      past 5800. If it does, the pitch is too fine.
- [ ] About 135 hp at 5500 RPM, the continuous rating.
- [ ] Too much or too little power overall: change `power_scalar`.
- [ ] Full-throttle climb to about 16,000 ft: manifold pressure holds near 40 inHg up to
      about 15,000 ft, then falls. If you change `max_design_mp`, move the MAN IN colour
      marks in `panel.xml` with it.
- [ ] Climb rate and cruise speed: note them. In level flight, watch that full throttle
      doesn't take it past VNE, 124 kt.
- [ ] Temperatures and pressures stay in the G3X green bands, which come from the 915 iS
      placard:

      | Reading | Green band |
      | --- | --- |
      | Oil pressure | 29-73 psi |
      | Oil temperature | 50-110 °C, normally 90-110 |
      | Coolant | below 120 °C |
      | EGT | below 950 °C |
      | Fuel pressure | about 42 psi (2.8-3.2 bar) |

- [ ] Fuel quantity still reads correctly. `fuel_press_max` was raised from 720 to
      6050 psf. In the VL-3 915 mod thread, changing it seemed to upset the fuel gauge.

## 7. Handling after the weight change

The 915 adds about 60 lb in the nose. Empty weight went from 655 to 715 lb, and the
empty CG moved forward from 0.25 to 0.57 ft.

- [ ] Hands-off pitch trim at about 100 kt. Before the change it trimmed near level with
      `htail_incidence` 1.0 in `flight_model.cfg`. A higher value gives more nose-up trim.
- [ ] Takeoff rotation and landing flare still have enough elevator.

## 8. What to send back

- The build output if it reported problems, and any G3X console errors.
- Screenshots: the panel from the pilot's seat, and the G3X with the engine running.
- Any seam where daylight or background shows in VR, with a screenshot and where you
  were looking from.
- The engine numbers: idle RPM; static RPM, manifold pressure, power and fuel flow; full
  throttle and 5500 RPM in level flight (RPM, power, speed); manifold pressure at 10,000,
  15,000 and 17,000 ft.
- The climb rate, cruise speed, and whether it trims hands-off.
