import sys, os
from enum import Enum
import time

import numpy as np

class State(Enum):
    ERROR = 0 # Shuts down system, requires user input to recover
    STOPPED = 1 # Robot is stopped, no data collection taking place. 
    RANDOM_TRACKING = 2 # Robot is operating as expected, collecting random samples. 
    REFERENCE_TRACKING = 3 # Robot is operating as expected, tracking provided reference. 
    MANUAL_TRACKING = 4 # Robot is operating as expected, tracking manual inputs. 
    RECOVER = 5 # Robot has moved outside of aurora field of view, attempting to recover without human intervention 
    READY = 6 # Ready to begin data collection 

'''
PCR Controller Class 

Class for full control of the PCR robot. 

Args: 
    motor_api (MotorController): Motor api object 
    aurora_api (AuroraAPI): Aurora api object 
    controller (Controller): Controller object 
    log_dir (str): root path for all logs 
    debug (bool): whether to show debug information 

Methods: 
    enable: enables full system 
    disable: disables full system
    set_mode: sets tracking mode for the robot ('man', 'ref', 'ran')
'''

class PCRController:
    def __init__(self, motor_api, aurora_api, controller, log_dir='logs', debug=False):
        assert controller.type == "Closed_Loop_Controller", "High level operation only supports Closed_Loop_Controller"
        # APIs 
        self.controller = controller 
        self.motor_api = motor_api
        self.aurora_api = aurora_api

        # State information 
        self.state = State.READY 
        self.end_point = np.array([0, 0])
        self.recovery_i = 0 
        self.recovery_sequence = [[0, 0]]
        self.ref_i = 0
        self.ref_squence = [[0, 0]]
        self.error_counter = 0

        # Logging setup 
        self.file = None
        self.log_flag = False
        self.log_dir = log_dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        self.debug = debug
        self.START_TIME = time.time()
    
    def __repr__(self):
        ret = f"PCRController @{time.time() - self.START_TIME}s\n"
        ret += f"  State: {self.state}\n"
        ret += f"  Endpoint: {self.end_point}\n"
        ret += f"  Logging: {self.log_flag}\n"
        ret += f"  Recovery i: {self.recovery_i}\n"
        return ret

    def enable(self):
        if self.debug:
            print("Enabling system.")
        self.motor_api.enable()
        self.aurora_api.start_tracking(self._update_end_point)
        time.sleep(3)

    def disable(self): 
        if self.debug:
            print("Disabling system.")
        self.motor_api.disable()
        self.aurora_api.stop_tracking()

    def set_mode(self, mode, ref=None):
        '''
        Changes the operating mode of the robot. 
        
        Args:
            mode (str): Mode to switch into. Options include 'man' (Manual), 'ran' (Random), and 'ref' (Reference)
            ref (iterable): (Nx2) reference trajectory if mode == 'ref', (2,) reference point if mode == 'man'
        '''
        if mode == 'man':
            self.state = State.MANUAL_TRACKING
            self.ref_point = ref
        elif mode == 'ran':
            self.state = State.RANDOM_TRACKING
        elif mode == 'ref':
            self.state = State.REFERENCE_TRACKING
            assert ref is not None, "Attempted to switch to reference mode without providing a reference trajectory."
            self.ref_i = 0
            self.ref_squence = ref


    def _update_end_point(self, aurora_reading):
        '''Updates sytem using aurora reading 
        '''
        if aurora_reading is None:
            if self.state != State.ERROR and self.state != State.RECOVER:
                self.error_counter += 1
                if self.error_counter >= 3: # If 3 missed measurements in a row consider out of bounds 
                    # If aurora reading is out of range, enter recovery mode 
                    if self.state == State.STOPPED:
                        # If state is stopped, recovery has already been completed and failed.
                        self.state = State.ERROR
                    else:
                        self._start_recovery()
                    self.end_point = None
        else:
            self.end_point = aurora_reading[:2]

        self._step()
    
    def _start_recovery(self):
        '''Puts robot into recovery mode 
        '''
        self.state = State.RECOVER

        # Zero motor commands 
        self.motor_api.set_ref([0, 0]) 

        # Disable logs        
        self.motor_api.disable_log()
        self.aurora_api.disable_log()
        self.disable_log()

        # Switch to recovery controller (open-loop position) 
        self.motor_api.reset_pid(True)
        self._pre_recovery_type = self.motor_api.type
        self.motor_api.type = 'pos'

        if self.debug:
            print(f"Entered recovery mode for {self.recovery_i} steps.")

    def _stop_recovery(self):
        '''Takes robot out of recovery mode 
        '''
        assert self.state == State.RECOVER, "Cannot stop recover if not currently in recover state."
        self.state = State.STOPPED

        # Zero motor commands 
        self.motor_api.pos = [self.motor_api.mtr_data.mtr1.position.value, self.motor_api.mtr_data.mtr2.position.value] 

        # Switch to provided controller 
        self.motor_api.reset_pid(False)
        self.motor_api.type = self._pre_recovery_type

        self.state = State.READY

        if self.debug:
            print("Exiting recovery mode.")
        
    def _step(self): 
        if self.debug: 
            print(self)
        
        # Get reference signal for current state 
        u = self._get_ref()

        if self.state == State.ERROR:
            print("ERROR: Please manually reset the system.")
            raise 

        if self.state == State.RECOVER:
            if np.linalg.norm([self.motor_api.mtr_data.mtr1.position.value, self.motor_api.mtr_data.mtr2.position.value]) < 20:
                # Exit recovery state when goal position is reached
                self._stop_recovery()

        if self.state in [State.RANDOM_TRACKING, State.REFERENCE_TRACKING, State.MANUAL_TRACKING]:
            self.controller.update_end_point(self.end_point)

        
        self.motor_api.set_ref(u) # Sends motor commands, comment to run system without motion 

    def _bound_extension(self, u):
        '''Limits robot to operate on one side of its bending ability
        '''
        # TODO 
        return u 
        

    def _get_ref(self):
        '''Gets setpoint for motor api based on current system state 
        '''
        if self.state in [State.RANDOM_TRACKING, State.REFERENCE_TRACKING]:
            return self._bound_extension(self.controller.u)
        elif self.state == State.MANUAL_TRACKING:
            return self.ref_point
        elif self.state == State.RECOVER: 
            return [0, 0] #self.recovery_sequence[self.recovery_i-1]
        elif self.state in [State.STOPPED, State.READY, State.ERROR]:
            return None

    def enable_log(self):
        file_root = os.path.join(self.log_dir, str(time.time())) # Generates unique filename 

        self.aurora_api.enable_log(file_root)
        self.motor_api.enable_log(file_root)
        self.file = open(file_root + "_controller.txt", 'a')
        self.file.write('time,state\n')
        self.log_flag = True

    def disable_log(self): 
        self.log_flag = False
        if self.file is not None:
            self.file.close()
