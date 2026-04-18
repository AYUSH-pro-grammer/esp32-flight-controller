# ESP32 Flight Controller (Custom PCB)

## Description

This project presents a basic custom flight controller that is based on the ESP32 and MPU6050 IMU chip. The project reads data from the sensors, uses PID control to stabilize the drone, and drives four motors through electronic speed controllers (ESC).

## How PCB will look
<img width="1047" height="767" alt="PCB Image" src="https://github.com/user-attachments/assets/52d42a74-ae0f-43de-bda9-856b2e67dfbe" />

## kicad schematic

![Schematic]("image/schematic.png")


The repository contains:

* Code for the flight controller
* Code for IMU calibration
* Files with PCB design (KiCad)
* Gerber files for board manufacture

---

## Project Directory Structure

```
flight-controller-code/
├── flightController.ino   # Flight controller code
├── calibration.ino        # IMU calibration code
├── kicad/                 # PCB design files
├── image/                 # Board images
├── main_grb.zip           # Gerber files for fabrication
└── README.md
```

---

## Features

* Integration of MPU6050 accelerometer and gyroscope
* Complementary filter algorithm for angle calculations
* Double-loop PID control (angle and angle rate)
* PWM signals to control 4 ESC motors
* Input from the RC receiver (6 channels)
* IMU calibration for accurate sensor offset values

---

## Hardware Requirements

* ESP32 microcontroller
* MPU6050 IMU
* 4 electronic speed controllers with brushless motors
* RC receiver (PWM protocol)
* LiPo battery
* Custom PCB (designed in KiCad, files in `kicad/` folder)

---

## Setup Procedure

### Step 1 - Upload Calibration Code

1. Load file `calibration.ino`
2. Program ESP32 with this code
3. Put the drone on the ground
4. Open the Serial Monitor window
5. Read instruction messages and save the output values

---

### Step 2 - Fill IMU Offset Values

Fill the calibration values into the file `flightController.ino`:

```
float rateRollOffset = ...;
float ratePitchOffset = ...;
float rateYawOffset = ...;

float accXOffset = ...;
float accYOffset = ...;
float accZOffset = ...;
```

---

### Step 3 - Upload the Main Code

* Load `flightController.ino`
* Program ESP30 with this code

---

## Controls (RC Mapping)

| Channel | Function  |
| ------- | --------- |
| CH1     | Roll      |
| CH2     | Pitch     |
| CH3     | Throttle  |
| CH4     | Yaw       |
| CH5/CH6 | Auxiliary |

---

## Important Tips

* Make sure your motors are installed correctly
* Do not run any tests with propellers mounted
* Fine-tune PID coefficients for better stability
* Calibrate your sensors carefully for optimal results

---

## Safety

* Do not power the motors with mounted propellers
* Ensure reliable power supply
* Have your hand near the throttle button

---

## Possible Upgrades

* Build-in fail-safe system
* Implement altitude hold function
* Enable GPS functionality
* Improve filtering algorithm (Kalman filter)

---

## Reference Code

Key code fragments are found in the following file:

---

## License

This project is open-source and can be freely modified.
