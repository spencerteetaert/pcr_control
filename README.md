# pcr_control
Fourth year undergraduate thesis

## System setup 
### Set up physical robot
Checklist: 
* Ensure everything is plugged in and connections are good 
* Switch on Aurora 
* Ensure E-stop is in the "on" position 
* Ensure the workspace (marked in green) is clear 
* Begin with the end effector in the state where both arms are fully outstretched. Manually tigheten each motor to ensure the tendons are taught
* Check that all tendons are properly threaded through pulleys 

### Set up digital environment 
```
source venv/bin/activate 
./scripts/setup.sh
```
### Calibrate Aurora
```
python scripts/calibrate_aurora.py
```
Once the calibration script is running: 
1. Move the tracker to position (1) with the cable pointing in the positive y direction (as indicated on the table)
2. Press 'r' to take a reading. Take 10+ per position. Once you've taken measurements in position (1) press n to save the point. 
3. Repeat steps 1 and 2 in position (2) followed by position (3)

The transformation is saved under 'data/rotation.npy' and 'data/translation.npy'. 
Place the tracker in its position on the robot's end effector. 