# LX200GPS RA Motor Control Experiment

## Overview

This project explores custom control of the Right Ascension (RA) drive system from a **Meade LX200GPS 12” telescope**, using a modern microcontroller-based approach in place of the original control electronics.

The goal is to understand and rebuild a stable, low-speed tracking system capable of maintaining **sidereal-rate motion** suitable for visual observing and astrophotography.

Rather than immediately replicating the original closed system, this project takes an experimental approach:

* Characterizing motor behavior under PWM control
* Investigating open-loop tracking feasibility
* Building toward closed-loop servo control using encoder feedback

---

## Current Hardware Setup

* **Telescope:** Meade LX200GPS 12” on equatorial wedge
* **RA Drive Train:** OEM motor + gearbox + worm gear assembly
* **Controller:** Arduino Uno R4 WiFi
* **Motor Driver:** External H-bridge (PWM + direction control)
* **Power:** 12V supply (separate from original electronics)
* **Feedback (planned):** US Digital E4T quadrature encoder (not yet integrated)

---

## System Architecture (Current Phase)

At present, the system operates in **open-loop mode**:

```
Arduino PWM → Motor Driver → DC Servo Motor → Gearbox → Worm Gear → Telescope RA Axis
```

Motor speed is controlled purely through PWM duty cycle without feedback.

---

## What This Project Is Investigating

### 1. Low-Speed Motor Behavior

* Identifying minimum stable PWM duty cycle
* Measuring stall thresholds
* Observing nonlinear response at very low speeds

### 2. PWM Frequency Effects

* Testing motor smoothness across different PWM frequencies
* Evaluating stiction and torque ripple effects
* Determining optimal switching frequency for RA tracking

### 3. Sidereal Tracking Approximation

* Finding open-loop PWM values that approximate sidereal motion
* Evaluating stability over time and changing load conditions

### 4. Transition to Closed-Loop Control (Planned)

* Integrating quadrature encoder feedback
* Implementing velocity-based PID control
* Closing the loop around sidereal tracking rate

---

## Key Observations So Far

* The motor exhibits a clear stall threshold at low PWM values
* Usable tracking occurs near the boundary of stall behavior
* System behavior is highly sensitive to load and friction
* Open-loop control is likely insufficient for long exposure stability
* Motor sound characteristics vary significantly between stable and unstable regimes

---

## Why This Approach?

Commercial telescope mounts like the LX200GPS historically used **closed-loop servo control**, continuously adjusting motor velocity using encoder feedback.

This project intentionally begins with open-loop control to:

* Build intuition for system dynamics
* Understand mechanical limitations
* Establish baseline behavior before adding feedback complexity

---

## Planned Next Steps

1. Complete PWM frequency characterization
2. Integrate quadrature encoder (E4T)
3. Implement velocity PID loop
4. Calibrate sidereal tracking rate in encoder units
5. Evaluate tracking stability under load and different sky positions

---

## Status

**Current phase:** Open-loop motor characterization
**Next phase:** Closed-loop velocity servo implementation

This project is ongoing and experimental in nature. The current code should be considered a test harness rather than a final control solution.

---

## Notes

This is a research and learning project focused on:

* embedded motor control
* servo systems
* astronomical tracking mechanics
* real-world PID behavior in nonlinear mechanical systems
