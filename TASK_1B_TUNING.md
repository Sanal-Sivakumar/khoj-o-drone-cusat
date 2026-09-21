# Task 1B: Z-Axis (Altitude) PID Tuning Methodology

## 🎯 Objective
The primary objective of Task 1B was to design and tune a PID controller for the drone's Z-axis (Altitude) to achieve a perfectly stable hover at a designated setpoint. This serves as the foundational prerequisite before introducing multi-axis directional movement in Task 1C.

## ⚙️ Final Verified Configuration
After iterative simulation testing, the following gains were locked in as the optimal configuration for altitude stability:
* **$K_p$ (Proportional):** 14.85
* **$K_i$ (Integral):** 0.168
* **$K_d$ (Derivative):** 18.6

## 🛠️ Step-by-Step Tuning Methodology
The tuning process followed a standard heuristic approach, treating the drone's vertical dynamics similarly to a classic spring-mass-damper system. 

### Step 1: Isolating the Proportional Gain ($K_p$)
* **Action:** Both $K_i$ and $K_d$ were initially set to `0.0`. The $K_p$ value was incrementally increased from zero.
* **Observation:** A low $K_p$ failed to overcome gravity. As $K_p$ approached the 10.0–14.0 range, the drone generated enough thrust to reach the target altitude but began oscillating aggressively around the setpoint.
* **Logic:** The proportional term acts like mechanical spring stiffness. It provides the necessary restoring force to reach the target but lacks the ability to dissipate the kinetic energy, leading to continuous bouncing. The value was capped at **14.85** where the oscillation amplitude was consistent.

### Step 2: Applying the Derivative Gain ($K_d$)
* **Action:** With $K_p$ fixed, the $K_d$ value was gradually introduced and increased.
* **Observation:** As $K_d$ increased, the aggressive bouncing smoothed out. The drone began to slow down predictably as it approached the target altitude, eliminating overshoot.
* **Logic:** The derivative term acts as a mechanical damper (like a shock absorber). It predicts future errors based on the current rate of change and counteracts the aggressive pull of the proportional gain. A final value of **18.6** provided critical damping, stopping the oscillation without making the system sluggish.

### Step 3: Introducing the Integral Gain ($K_i$)
* **Action:** With the drone stable but hovering slightly below the exact target coordinate (steady-state error), a very small $K_i$ value was introduced.
* **Observation:** The drone slowly closed the remaining fractional distance to perfectly align with the target altitude.
* **Logic:** The integral term accumulates past errors over time. Because the drone's weight constantly pulls it down, $K_p$ and $K_d$ alone leave a tiny gap. The $K_i$ value of **0.168** acts as a slow, continuous corrective force to eliminate that final offset.

## 📝 Key Takeaways for Task 1C
* Aggressive $K_p$ values will cause severe instability; always dampen with $K_d$ before assuming the $K_p$ is too high.
* $K_i$ should remain extremely small, as integral windup can cause the drone to shoot past the target unexpectedly. 
* This identical three-step methodology (Proportional lift $\rightarrow$ Derivative damping $\rightarrow$ Integral correction) will be applied to the Pitch and Roll tuning in Task 1C.
