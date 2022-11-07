import sys
import signal
import time
import threading 

from Aurora import Aurora

class AuroraAPI:
    def __init__(self):
        self.tracker = Aurora(baud_rat=9600)
        self.thread = threading.Thread(target=self._read_sensor_data)
        self.kill = False
        self.sensor_data = None

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
        self.tracker.portHandles_detectAndAssign_FlowChart(printFeedback=True)
        time.sleep(0.1)
        self.tracker.portHandles_updateStatusAll()
        print("Aurora tracker set.")

        
    def start_tracking(self):
        # Tracking
        print("Starting Aurora tracking...")
        self.tracker.trackingStart()
        time.sleep(3)
        self.thread.start()
        time.sleep(1)
        print("Aurora tracking started.")

    def _read_sensor_data(self):
        while not self.kill:
            self.tracker.sensorData_updateAll()
            self.sensor_data = self.tracker.sensorData_get()

    def get_sensor_data(self):
        return self.sensor_data

    def stop_tracking(self):
        self.tracker._BEEP(2)
        self.tracker.trackingStop()
        self.kill = True
        self.thread.join()
        self.tracker.disconnect()


if __name__=='__main__':
    aurora = AuroraAPI()

    aurora.start_tracking()

    for i in range(10):
        print(i, ': ', aurora.get_sensor_data())        
        time.sleep(1)

    aurora.stop_tracking()