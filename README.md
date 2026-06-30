# Proportional Navigation Intercept Guidance Law

## Motivation

I built this project to develop a working understanding of guidance laws from first principles. Guidance is the third pillar of GNC alongside navigation and control, and I wanted to implement and analyze the standard intercept guidance law used in real missile and autonomous vehicle systems rather than just read about it.

## Overview

A 2D pursuer-target intercept simulation in MATLAB comparing three guidance approaches:

1. **Pure pursuit** - naive baseline, always points directly at the target
2. **Proportional navigation (PN)** - commands acceleration proportional to line-of-sight rotation rate
3. **Augmented proportional navigation (APN)** - extends PN by incorporating target acceleration directly

The project includes a 200-trial Monte Carlo analysis that identifies the specific conditions where basic PN fails, and validates that APN resolves those failures.

## Key Results

- PN achieves intercept 8.89s vs pure pursuit's 10.32s on a constant-velocity target, with significantly lower terminal acceleration demand
- Monte Carlo analysis identified a hard PN failure boundary above 25 deg/s target turn rate (97.5% hit rate overall, with all failures clustered above this threshold)
- APN eliminated all previously identified PN failure cases, improving hit rate from 97.5% to 100% across 200 identical randomized trials

## Technical Details

**Guidance law equations:**

Pure pursuit:
```
pursuer_heading = atan2(rel_pos_y, rel_pos_x)
```

Proportional navigation:
```
LOS_rate = (rel_pos x rel_vel) / |rel_pos|^2
a_cmd = N * V_closing * LOS_rate
```

Augmented proportional navigation:
```
a_cmd = N * V_closing * LOS_rate + (N/2) * a_target_perp
```

Where N = navigation constant (set to 4), V_closing = closing velocity along LOS, and a_target_perp = component of target acceleration perpendicular to LOS.

**Monte Carlo setup:**
- 200 trials with fixed random seed (rng = 42) for reproducibility
- Maneuver start time randomized uniformly between 1 and 5 seconds
- Target turn rate randomized uniformly between 5 and 30 deg/s
- Hit threshold: miss distance < 10m

## File Structure

```
pure_pursuit.m                        # Baseline guidance law
proportional_navigation.m             # PN implementation with acceleration plot
pn_maneuvering_target.m               # PN against a target executing a constant turn
pn_monte_carlo_with_failure_analysis.m # 200-trial Monte Carlo with failure identification
pn_vs_apn_monte_carlo.m               # Side-by-side PN vs APN comparison
```

## How to Run

Run scripts individually in MATLAB in the following order to follow the full analysis progression:

```matlab
pure_pursuit                          % baseline
proportional_navigation               % PN vs constant-velocity target
pn_maneuvering_target                 % PN vs maneuvering target
pn_monte_carlo_with_failure_analysis  % identify PN failure boundary
pn_vs_apn_monte_carlo                 % APN comparison and validation
```

## Dependencies

MATLAB R2024a or later. No additional toolboxes required.
