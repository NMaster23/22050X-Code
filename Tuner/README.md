# VEX V5 Interactive PID Tuning Template

A production-grade, interactive PID tuning template for PROS on the VEX V5 system. This template allows you to tune **Linear Drive (Distance)**, **Angular Drive (Heading / Turn)**, and **Mechanism / Subsystem (Lift / Arm / Flywheel)** PID loops live on your robot using the V5 Controller and Brain LCD—without recompiling or reflashing code between iterations.

---

## Architecture & File Structure

```
Tuner/
├── include/
│   ├── pid.hpp            # Reusable PID controller (anti-windup, slew rate, settle criteria, metrics)
│   ├── tuner_config.hpp   # Hardware port mapping, drivetrain geometry, and default gains
│   ├── pid_tuner.hpp      # Interactive tuning engine (controller UI, screen UI, CSV serial telemetry)
│   ├── main.h             # PROS project header
│   └── api.h              # PROS V5 API definitions
├── src/
│   └── main.cpp           # PROS lifecycle entry points (initialize, autonomous, opcontrol)
├── Makefile               # PROS build configuration
├── project.pros           # PROS project metadata
└── README.md              # This guide
```

---

## Hardware Configuration (`include/tuner_config.hpp`)

Before deploying to your robot, adjust the configuration parameters in [`include/tuner_config.hpp`](file:///c:/Users/nihaa/Github/22050X-Code/Tuner/include/tuner_config.hpp):

1. **Motor Ports**:
   ```cpp
   const std::vector<int8_t> LEFT_DRIVE_PORTS = {1, -2, 3};    // Negative denotes reversed motor
   const std::vector<int8_t> RIGHT_DRIVE_PORTS = {-4, 5, -6};
   constexpr int8_t MECHANISM_PORT = 7;
   constexpr uint8_t IMU_PORT = 8;                             // Set to 0 if not installed
   ```

2. **Drivetrain Geometry**:
   ```cpp
   constexpr double WHEEL_DIAMETER_INCHES = 3.25;  // 2.75, 3.25, 4.0, etc.
   constexpr double GEAR_RATIO = 1.0;              // Motor revolutions : Wheel revolutions
   constexpr double TRACK_WIDTH_INCHES = 12.0;     // Center-to-center distance between wheel tracks
   ```

3. **Starting Gains & Test Step Sizes**:
   - `DEFAULT_DRIVE_LINEAR_STEP`: Target distance for linear step tests (default `24.0` inches).
   - `DEFAULT_DRIVE_ANGULAR_STEP`: Target heading for turn tests (default `90.0` degrees).
   - `DEFAULT_MECHANISM_STEP`: Target position for mechanism tests (default `360.0` degrees).

---

## Interactive Controller Cheat-Sheet

During `opcontrol()`, the tuner runs continuously and outputs formatted telemetry to both the V5 Controller and Brain LCD.

| Button / Input | Function | Description |
| :--- | :--- | :--- |
| **Button Y** | **Cycle Mode** | Switch between `DRV_LIN` (Distance), `DRV_ANG` (Turn), and `MECH` (Mechanism). |
| **Button X** | **Cycle Parameter** | Switch active parameter to edit: `Kp` $\rightarrow$ `Ki` $\rightarrow$ `Kd` $\rightarrow$ `Tgt` (Target step size). |
| **DPAD UP** | **Increase Value** | Increase selected gain/target by the current increment step. |
| **DPAD DOWN** | **Decrease Value** | Decrease selected gain/target by the current increment step. |
| **DPAD RIGHT** | **Step Coarser** | Increase adjustment granularity (`0.001` $\rightarrow$ `0.01` $\rightarrow$ `0.1` $\rightarrow$ `1.0` $\rightarrow$ `5.0`). |
| **DPAD LEFT** | **Step Finer** | Decrease adjustment granularity (`5.0` $\rightarrow$ `1.0` $\rightarrow$ `0.1` $\rightarrow$ `0.01` $\rightarrow$ `0.001`). |
| **Button A** | **Execute Step Test** | Zeroes sensors, executes the step response test with current gains, and records performance metrics. |
| **Button B** | **Emergency Abort** | Instantly aborts any active test and brakes all motors. |
| **Joysticks (L/R)** | **Manual Reposition** | Drive the robot manually between tests without needing to reset or pick it up. |
| **Shoulder R1 / R2** | **Manual Mechanism** | Jog mechanism forward/reverse manually when idle. |

---

## Controller Screen Display

The V5 Controller screen displays live feedback in real time:

```
Line 0: [DRV_LIN] SEL:Kp
Line 1: P:8.50 I:0.000 D:25.00
Line 2: Tgt:24.0 Inc:0.500
```

When a test completes, the Brain LCD and terminal report:
- **Rise Time (90%)**: Time in milliseconds to reach 90% of setpoint.
- **Settling Time**: Time until error remains within `small_error` or `large_error` threshold.
- **Peak Overshoot**: Distance/angle past target and percentage overshoot.
- **Steady-State Error**: Final residual error.

---

## Live Serial Telemetry & Plotting

While a test is executing, the template streams high-frequency CSV data to the serial port / terminal:

```
[CSV] time_ms,target,measured,error,output
[CSV] 0,24.000,0.000,24.000,127.00
[CSV] 10,24.000,0.420,23.580,127.00
...
[CSV] 520,24.000,23.950,0.050,4.25
```

### Viewing Real-Time Plots
1. Connect your V5 Brain or Controller via USB.
2. In your terminal, run:
   ```bash
   pros terminal
   ```
3. To visualize step curves in real time, copy or redirect the output into plotting tools such as **PlotJuggler**, **WebSerial Plotter**, or import into Python (`matplotlib` / `pandas`).

---

## Step-by-Step Tuning Methodology (P $\rightarrow$ D $\rightarrow$ I)

1. **Zero Out Gains**: Start with $K_i = 0$ and $K_d = 0$.
2. **Increase $K_p$**:
   - Gradually increase $K_p$ using DPAD UP until the robot moves briskly towards the target.
   - Stop when the robot starts to oscillate or overshoot the setpoint.
3. **Add $K_d$**:
   - Increase $K_d$ to introduce damping.
   - Watch the overshoot decrease on the Brain screen metrics until the robot settles smoothly into the target without oscillation.
4. **Add $K_i$ (if needed)**:
   - If steady-state error remains (e.g. friction causes the robot to stop 0.5" before the target), add a very small $K_i$ (e.g., `0.001` - `0.01`).
   - The template includes built-in anti-windup (`windup_range` and zero-crossing integral reset) to prevent integral windup.
5. **Persist Gains**:
   - Once tuned, record your values and update the defaults in [`include/tuner_config.hpp`](file:///c:/Users/nihaa/Github/22050X-Code/Tuner/include/tuner_config.hpp).
