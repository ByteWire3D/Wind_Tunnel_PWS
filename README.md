# Peregrine Falcon
This a PWS of Faiaz Dindar and Leon Frederiks.

The project consists of 6 modules:

The main controller:
- sends all the commands to the rest of the modules

The angle controller:
- a PID controlled n20 brushed encode motor
- cable of 1/5 deg acuraty

The meassuring module:
- meassures:
  - voltage
  - ampere
  - wattage
  - mah_used
  - lift
  - drag

The pid controller:
- meassures: airspeed (thru a kalman and moving filter) ( via a pitot tube)
- calculates pid values given a setpoint.
- control's 2 motors to 0.2 ms airspeed error exuracy
- has a (software based) kill switch to disable the system in unsave situations

The info display:
- displays all the recieved data of the system.
- and error messages (to see what state the system is currently in)
  
The datalogger:
- loggs all the data to an sd card at 5 hz (8hz max)
- and saves the test configuration, settings & cofiguration.


 
