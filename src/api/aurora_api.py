import os
import sys
import signal
import time
import threading 
from functools import partial

from pynput import keyboard
import numpy as np

if __name__=='__main__':
    from Aurora import Aurora
else:
    from .Aurora import Aurora

class AuroraAPI:
    def __init__(self, workspace='/home/jimmy/spencer_thesis/pcr_control/data', verbose=False):
        self.workspace = workspace
        self.verbose = verbose

        self.file = None
        self.log_flag = False

        self.tracker = Aurora(baud_rat=9600)
        self.thread = threading.Thread(target=self._read_sensor_data)
        self.kill = False
        self.sensor_data = [[np.array([1., 0., 0., 0.]), np.array([0., 0., 0.])]]
        self.latest_updated_data = [[np.array([1., 0., 0., 0.]), np.array([0., 0., 0.])]]
        self.position = np.array([0, 0, 0])
        self.calibrating = False
        self.tracking = False
        self.calibrated = False

        self.norm_rotation = np.eye(3)
        self.norm_translation = np.array([0, 0, 0])

        try: 
            self.norm_rotation = np.load(open(os.path.join(self.workspace, 'rotation.npy'), 'rb'))
            self.norm_translation = np.load(open(os.path.join(self.workspace, 'translation.npy'), 'rb'))
            self.calibrated = True
        except:
            print("No calibration found. Calibrate workspace before collecting data.")

        # setup sigint handler to disable motor on CTRL+C
        def sigint_handler(signal, frame):
            print("Stop motor and shut down.")
            self.stop_tracking()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)
        
        while not self.tracker._isConnected:
            print('Aurora: Trying to connect...')
            self.tracker.connect()
            time.sleep(1)
        self.tracker.init()

        assert(self.tracker._serial_object.isOpen()), "Tracker serial connection unable to open."
        assert(self.tracker._device_init), "Tracker device initialization failed."

        print("Setting Aurora tracker...")
        time.sleep(0.1)
        self.tracker.portHandles_detectAndAssign_FlowChart(printFeedback=self.verbose)
        time.sleep(0.1)
        self.tracker.portHandles_updateStatusAll()
        print("Aurora tracker set.")

    def __repr__(self):
        pos = self.get_position()
        return f"Aurora position: {pos[0]:0.4}x, {pos[1]:0.4}y, {pos[2]:0.4}z"
        
    def start_tracking(self):
        # Tracking
        print("Starting Aurora tracking...")
        self.tracker.trackingStart()
        time.sleep(3)
        self.thread.start()
        time.sleep(1)
        self.tracking = True
        print("Aurora tracking started.")

    def _read_sensor_data(self):
        while not self.kill:
            self.tracker.sensorData_updateAll()
            new_data = self.tracker.sensorData_get()
            if np.any(new_data[0][1] == self.latest_updated_data[0][1]):
                print("\rERROR: Sensor out of range. Move back to Aurora workspace", end='')
                self.sensor_data = [[np.array([1., 0., 0., 0.]), np.array([0., 0., 0.])]]
            else:
                self.sensor_data = new_data
                self.latest_updated_data = new_data

                self.log()

                if self.calibrating:
                    print("\rCurrent Reading:", new_data[0][1], end='')
                elif self.verbose:
                    print("\rCurrent Position:", self.get_position(), end='')

    def get_sensor_data(self):
        '''
        Data format: [
            [np.array([x,y,z,w]), np.array([x,y,z])],
        ]
        '''
        return self.sensor_data

    def get_position(self):
        '''Returns position in robot frame 
        '''
        if not self.calibrated:
            print("ERROR: Aurora is not calibrated to workspace. Unable to provide position.")
            return None
        return self.norm_rotation @ (self.sensor_data[0][1] + self.norm_translation)

    def enable_log(self, filename):
        self.file = open(filename + "_aurora.txt", 'a')
        self.file.write('time,x,y,z,x_raw,y_raw,z_raw,q1,q2,q3,q4\n')
        self.log_flag = True
    
    def disable_log(self):
        self.log_flag = False
        if self.file is not None:
            self.file.close()

    def log(self):
        if self.log_flag:
            timestep = time.time()
            pos = self.get_position()
            pos_raw = self.sensor_data[0][1]
            q = self.sensor_data[0][0]
            self.file.write(f"{timestep},{pos[0]},{pos[1]},{pos[2]},{pos_raw[0]},{pos_raw[1]},{pos_raw[2]},{q[0]},{q[1]},{q[2]},{q[3]}\n")

    def stop_tracking(self):
        self.tracker._BEEP(2)
        self.tracker.trackingStop()
        self.kill = True
        self.thread.join()
        self.tracker.disconnect()
        self.tracking = False

    #region Calibration Tools
    def calibrate(self, kill_on_completion=True):
        print("Starting calibration process..")
        self.calibrating = True

        if not self.tracking:
            self.start_tracking()

        self.cal_readings = []
        self.cal_point_id = 0
        self.cal_point_readings = [0, 0, 0] # three corner points, labelled on table
        
        listener = keyboard.Listener(
            on_press=partial(self.on_press, ),
            on_release=self.on_release)
        listener.start()

        print("Record points. Press 'R' to read, 'D' to delete last reading, 'N' to move to next area")
        listener.join()
        if kill_on_completion:
            self.stop_tracking()

        # Process points to normalize readings. Goal: Find R and T st. Rp + T is in normalized coordinates
        y_axis = self.cal_point_readings[1] - self.cal_point_readings[0] 
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = self.cal_point_readings[2] - self.cal_point_readings[0] 
        x_axis = x_axis / np.linalg.norm(x_axis)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Recompute this as markings on board may not form orthogonal vectors but are guaranteed to be in the same plane
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        self.norm_rotation = np.hstack([x_axis.reshape(3,1), y_axis.reshape(3,1), z_axis.reshape(3,1)])
        self.norm_translation = -self.cal_point_readings[0]

        print("Rotation", self.norm_rotation, self.norm_rotation.shape)
        print("Translation", self.norm_translation, self.norm_translation.shape)

        np.save(open(os.path.join(self.workspace, 'rotation.npy'), 'wb'), self.norm_rotation)
        np.save(open(os.path.join(self.workspace, 'translation.npy'), 'wb'), self.norm_translation)

        self.calibrating = False
        self.calibrated = True
        
        print("Aurora calibration complete. Any movement in the workspace or Aurora will require recalibration.")

    def on_press(self, key):
        if key == keyboard.KeyCode.from_char('r'):
            self.cal_readings += [self.get_sensor_data()[0][1]]
            print("Recorded", self.cal_readings[-1])
        elif key == keyboard.KeyCode.from_char('d'):
            print("Deleted", self.cal_readings[-1])
            self.cal_readings = self.cal_readings[:-1]
        elif key == keyboard.KeyCode.from_char('n'):
            self.cal_point_readings[self.cal_point_id] = np.average(self.cal_readings, 0)[:3]
            print("Point saved.", self.cal_point_readings[self.cal_point_id])
            self.cal_readings = []
            self.cal_point_id += 1
            
    def on_release(self, key):
        if key == keyboard.Key.esc:
            # Stop listener
            return False
        if self.cal_point_id == 3:
            return False
    #endregion

if __name__=='__main__':
    aurora = AuroraAPI(verbose=True)

    aurora.start_tracking()  
    aurora.enable_log('data/DELETE')
    time.sleep(15)
    aurora.disable_log()
    aurora.stop_tracking()