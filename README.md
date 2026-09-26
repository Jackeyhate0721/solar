# Solar Tracker Control Simulation

A MATLAB / Simulink solar-tracker project that answers a question most
submissions leave open: **is tracking actually worth it, and when does it stop
paying?**

**Trend:** Sustainability
**Toolboxes used:** Simulink, Simscape Electrical, Simscape Driveline,
Simscape Foundation

---

## Project structure

```
tracker_sim.slx          Simulink / Simscape multi-axis tracker (main model)
build_tracker_sim.m      One-call script that rebuilds tracker_sim.slx
run_tracker_sim.m        Builds, runs and plots the Simulink model (the
                         runner that was missing; also runs the authority
                         diagnostics)
servo_params.m           Motor and mechanical plant parameters
manual_servo.m           Single-step and sinusoidal servo verification
tracking_day.m           Full-day sun-path tracking run
run_tracker.m            Energy & wear comparison across 4 configs x 3 weathers
sweep_sensitivity.m      Latitude x day-of-year gain map + cloudiness breakeven
sun_position.m           Cooper declination + equation of time
gen_irradiance.m         Clearness-index-scaled global horizontal irradiance
                         with an Erbs beam/diffuse split
gen_kt.m                 Clearness-index time series
gen_wind.m               Wind-speed time series
panel_gain.m             Isotropic-sky + ground-reflected panel model
simulate_tracker.m       One-day tracker run with slew limit, deadband, stow
fig_*.png                Result figures
```

## What the model does

`tracker_sim.slx` implements a single-axis solar tracker as a coupled
electromechanical system:

- **Electrical side (Simscape Electrical):** a permanent-magnet DC motor
  (armature R + L + rotational electromechanical converter, K = Kt) driven by
  a Controlled Voltage Source, closed through an electrical reference. The
  voltage comes from the PID, not from a hard-wired supply.
- **Mechanical transmission (Simscape Driveline):** a worm-and-gear block
  (the project's foundational element) between the motor shaft and the panel.
- **Panel plant (Simscape Foundation):** panel inertia, bearing damping,
  gravity torque and a wind-load disturbance on the tilt axis. Both
  disturbances enter as ideal torque sources with one port on the panel node
  and the other on ground, so they actually apply torque.
- **Control (Simulink):** a PID controller (P=90, I=4, D=5) with
  anti-windup back-calculation and gravity feedforward, driven by a solar
  position reference computed from the Cooper sun-position algorithm. The
  PID feedback is the panel *angle* (the Motion Sensor's position port),
  not its angular velocity, and the reference is in radians to match.

Run the model with `tracker_sim` after creating a `ref_tilt` workspace
variable (a two-column `[time, angle_deg]` matrix, sample time 0.02 s).

## Key findings

Running `run_tracker` compares four panel configurations (fixed tilt,
horizontal single-axis, dual-axis, and a "smart" dual-axis tracker with a
0.5-degree deadband, cloud-sleep and wind-stow rules with hysteresis) across
three weather scenarios.

| Scenario                     | Fixed | Single-axis | Dual-axis | Smart |
|------------------------------|-------|-------------|-----------|-------|
| Clear summer, calm           |  8031 |     8521    |   12868   | 12864 |
| Partly cloudy summer         |  5344 |     5981    |    7320   |  7356 |
| Overcast winter, gusty wind  |  1004 |      884    |    959    |  1003 |

Energies are net Wh/m^2 of **incident irradiance** (captured minus drive
energy). These are not PV electrical output -- no cell or MPPT model is
applied, so the comparison is apples-to-apples on the irradiance side but
should not be read as delivered AC. Highlights:

1. **On a calm clear day, a dual-axis tracker captures +60.2 % over a fixed
   panel**, and the horizontal single-axis tracker +6.1 %; the drive energy is
   ~0.0003 % of what is gained. *Moving the panel is not the cost.* (The gain
   is lower than the naive 74 % you get without backtracking -- near sunrise
   and sunset the tracker backs off its tilt to avoid shading the next row,
   which trades beam capture for row efficiency.)
2. **On an overcast day, a dual-axis tracker loses -4.5 %** versus fixed
   (backtracking limits the loss; without it the number is -10.5 %), because
   it tilts the panel away from the diffuse sky dome.
3. **The smart tracker pays off on diffuse / gusty days and is neutral
   otherwise.** Under clear skies it slews ~1 % less than the plain dual-axis
   tracker (2° deadband trims the fine-tracking chatter) at 99.9 % of the
   dual-axis gain. On the partly cloudy day it slews *more* (150 %) because
   the cloud-sleep logic makes it lie flat (0°) under diffuse sky while the
   dual-axis keeps tracking -- a trade-off that only pays off when the diffuse
   capture advantage outweighs the lost beam tracking. On the overcast winter
   day it is the only config that avoids the loss (0° slew vs 383°). The smart
   logic is a risk-management feature, not a free efficiency win.
4. `sweep_sensitivity` generalises these results: the latitude x day-of-year
   gain map shows that tracking pays most at high latitudes in winter, and the
   cloudiness breakeven curves show the fraction of overcast days at which
   tracking stops being worthwhile at a chosen latitude.

## Reproducibility

All files live in this folder (the project root) -- there is no inner
`solar_tracker` subfolder to `cd` into.

1. Open MATLAB (R2023b or later with Simscape Electrical, Simscape Driveline
   and Simscape Foundation).
2. `cd` to this folder and `addpath(genpath(pwd));`
3. `build_tracker_sim`  to (re)build the Simulink model.
4. `run_tracker_sim`    to run the Simulink model, plot the response and
   regenerate `fig_tracker_sim.png` (also prints the controller-authority
   diagnostics).
5. `run_tracker`        for the energy / wear comparison across configs.
6. `sweep_sensitivity`  for the latitude-day map and breakeven curves.
7. `tracking_day` and `manual_servo` for the MATLAB-side servo verification.

## Figures

- `fig_tracker_sim.png`   -- servo step / day-tracking response
- `fig_irradiance.png`    -- panel irradiance for each config x weather
- `fig_energy.png`        -- net and captured energy comparison
- `fig_wear.png`          -- cumulative slew and wind exposure (wear proxies)
- `fig_sensitivity.png`   -- gain map and cloudiness breakeven

## License

MIT. See [LICENSE](LICENSE).
