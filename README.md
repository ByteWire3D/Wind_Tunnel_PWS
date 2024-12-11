# Peregrine Falcon Project

This repository documents the Peregrine Falcon project, a collaborative effort by Faiaz Dindar and Leon Frederiks.

The project is divided into six main modules:

# Main Controller
The main controller is responsible for:
- Sending all commands to the other modules.
- Angle Controller
- Utilizes a PID-controlled N20 brushed encoder motor.
- Achieves an accuracy of up to 1/5 degree.

# Measuring Module
The measuring module collects the following data:
- Voltage
- Current (amperes)
- Power (wattage)
- Energy consumption (mAh used)
- Lift
- Drag

# PID Controller
The PID controller is central to the system and performs the following functions:
- Measures airspeed using a Kalman filter and moving average filter in conjunction with a pitot tube.
- Calculates PID values based on a given setpoint.
- Controls two motors to maintain an airspeed error accuracy of 0.2 m/s.
- Includes a software-based kill switch for safely disabling the system in unsafe conditions.

# Info Display
The info display provides:
- Real-time system data.
- Error messages for diagnosing system states.

# Data Logger
The data logger:
- Logs all data to an SD card at a frequency of 5 Hz (with a maximum capability of 8 Hz).
- Saves test configurations, system settings, and calibration values.
- Due to noise issues, logging the date to the SD card did not work as intended. Instead, data was logged using Excel’s data streamer as an alternative solution.

This modular approach ensures a robust and flexible system capable of precise measurements and control in various testing environments. The use of Excel’s data streamer as a fallback for logging highlights the adaptability of the system to handle unexpected challenges.


 
