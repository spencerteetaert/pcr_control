import sys, os
from enum import Enum
import time

import numpy as np

class State(Enum):
    ERROR = 0 # Shuts down system, requires user input to recover
    STOPPED = 1 # Robot is stopped, no data collection taking place. 
    RANDOM_TRACKING = 2 # Robot is operating as expected, collecting random samples. 
    REFERENCE_TRACKING = 3 # Robot is operating as expected, tracking provided reference. 
    REFERENCE_STARTING = 4
    MANUAL_TRACKING_JOINT = 5 # Robot is operating as expected, tracking manual inputs. 
    MANUAL_TRACKING_TASK = 6 # Robot is operating as expected, tracking manual inputs. 
    RECOVER = 7 # Robot has moved outside of aurora field of view, attempting to recover without human intervention 
    READY = 8 # Ready to begin data collection 
    HOLDING = 9

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
        self.debug = debug
        
        # APIs 
        self.controller = controller 
        self.motor_api = motor_api
        self.aurora_api = aurora_api

        self.bound_buffer = 10 # rad 
        self.workspace_size = np.array([0.5, 0.5]) # m 

        # State information 
        self.state = State.STOPPED 
        self.holdtime = 0 
        self.end_point = np.array([0, 0])
        self.u = None
        self.recovery_i = 0 
        self.recovery_sequence = [[0, 0]]
        self.ref_i = 0
        self.ref_squence = [[0, 0]]
        self.ref_point = [0, 0]
        self.error_counter = 0
        self.generate_random = False

        self.v_end_point = None
        self.last_ee_update = None
        self.traj_start_time = None

        # Logging setup 
        self.log_file = None
        self.log_flag = False
        self.log_dir = log_dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        self.START_TIME = time.time()
        self.meta_log = open(os.path.join(self.log_dir, "meta_data.txt"), 'a')
        self.meta_log.write('trial_prefix,termination_cause,duration,start_x,start_y,end_x,end_y\n')

        self.log_err_msg = None
        self.log_file_start_time = None
        self.log_start_pos = None
    
    def __repr__(self):
        ret = f"PCRController @{time.time() - self.START_TIME}s\n"
        ret += f"  State: {self.state}\n"
        ret += f"  Endpoint: {self.end_point}\n"
        ret += f"  Setpoint: {self.ref_point}\n"
        ret += f"  Control: {self.u}\n"
        ret += f"  Logging: {self.log_flag}\n"
        ret += f"  Recovery i: {self.recovery_i}\n"
        return ret

    def enable(self):
        if self.debug:
            print("Enabling system.")
        self.motor_api.enable()
        self.aurora_api.start_tracking(self._update_end_point)
        time.sleep(3)
        self.state = State.READY

    def disable(self): 
        if self.debug:
            print("Disabling system.")
        self.disable_log('USER TERMINATION')
        self.motor_api.disable()
        self.aurora_api.stop_tracking()

    def set_mode(self, mode, ref=None):
        '''
        Changes the operating mode of the robot. 
        
        Args:
            mode (str): Mode to switch into. Options include 'man' (Manual), 'ran' (Random), and 'ref' (Reference)
            ref (iterable): (Nx2) reference trajectory if mode == 'ref', (2,) reference point if mode == 'man'
        '''
        if self.debug: 
            print("Mode change:", mode)
        self.generate_random = False
        self.disable_log('MODE SWITCH')
        if mode == 'man_joint':
            self.state = State.MANUAL_TRACKING_JOINT
            self.ref_point = ref
        elif mode == 'man_task':
            self.state = State.MANUAL_TRACKING_TASK
            self.controller.update_goal_point(ref)
            self.ref_point = ref
        elif mode == 'ran':
            self.state = State.READY
            self.generate_random = True
        elif mode == 'ref':
            self.state = State.REFERENCE_STARTING
            assert ref is not None, "Attempted to switch to reference mode without providing a reference trajectory."
            self.ref_i = 0
            self.ref_squence = ref
            self.ref_point = self.ref_squence[self.ref_i]
            self.controller.update_goal_point(self.ref_point)


    def _update_end_point(self, aurora_reading):
        '''Updates sytem using aurora reading 
        '''
        if aurora_reading is None:
            if self.state != State.ERROR and self.state != State.RECOVER:
                self.error_counter += 1
                if self.error_counter >= 3: # If 3 missed measurements in a row consider out of bounds 
                    # If aurora reading is out of range, enter recovery mode 
                    if self.state == State.READY:
                        # If state is ready, recovery has already been completed and failed.
                        self.state = State.ERROR
                        self.log_err_msg = 'ERROR: Recovery failed.'
                    else:
                        self._start_recovery()
                    self.end_point = None
        else:
            self.error_counter = 0 
            if self.last_ee_update is not None and self.end_point is not None:
                self.v_end_point = np.linalg.norm(self.end_point - aurora_reading[:2]) / (time.time() - self.last_ee_update)
            self.last_ee_update = time.time()
            self.end_point = aurora_reading[:2]
            

        self._step()
    
    def _start_recovery(self):
        '''Puts robot into recovery mode 
        '''
        self.state = State.RECOVER

        # Zero motor commands 
        self.motor_api.set_ref([0, 0]) 

        # Disable logs        
        self.disable_log('AURORA LOST')

        # Switch to recovery controller (open-loop position) 
        self.motor_api.reset_pid(True)
        self._pre_recovery_type = self.motor_api.type
        self.motor_api.type = 'pos'

        self.enable_log()

        if self.debug:
            print(f"Entered recovery mode.")

    def _stop_recovery(self):
        '''Takes robot out of recovery mode 
        '''
        assert self.state == State.RECOVER, "Cannot stop recover if not currently in recover state."
        self.state = State.STOPPED

        # Disable logs        
        self.disable_log('RECOVERY')

        # Zero motor commands 
        self.motor_api.pos = [self.motor_api.mtr_data.mtr1.position.value, self.motor_api.mtr_data.mtr2.position.value] 

        # Switch to provided controller 
        self.motor_api.reset_pid(False)
        self.motor_api.type = self._pre_recovery_type

        self.state = State.READY

        if self.debug:
            print("Exiting recovery mode.")

    def _check_health(self):
        try:
            if not self.motor_api.thread.is_alive():
                raise
        except:
            self.state = State.ERROR
            self.log_err_msg = 'ERROR: Motor api thread died.'

        try:
            if not self.aurora_api.thread.is_alive():
                raise
        except:
            self.state = State.ERROR
            self.log_err_msg = 'ERROR: Aurora api thread died.'
        
    def check_point(self, pt):
        '''Returns True if pt is reachable, False if not 
        '''
        for link in self.controller.links:
            d = np.linalg.norm(pt - link.BASE_POINT)
            b = np.linalg.norm(pt - self.end_point)
            if d < link.EXCLUSION_RADIUS or d > link.LENGTH - 0.05 or b > 0.15 or b < 0.05:
                return False
        return True

    def _step(self): 
        if self.debug: 
            print(self)

        self._check_health() # Raises error if either aurora or motor thread dies 

        if self.state == State.ERROR:
            print("ERROR: Please manually reset the system.")
            self.disable_log(self.log_err_msg)
            self.disable()
            raise 

        if self.state == State.RECOVER:
            if np.linalg.norm([self.motor_api.mtr_data.mtr1.position.value, self.motor_api.mtr_data.mtr2.position.value]) < 5:
                # Exit recovery state when goal position is reached
                self._stop_recovery()

        if self.state == State.RANDOM_TRACKING:
            if np.linalg.norm(self.end_point - self.ref_point) < 0.02:
                self.disable_log('SUCCESS')
                self.state = State.READY
            elif self.v_end_point < 0.001 and time.time() - self.traj_start_time > 5:
                self.disable_log('STALLED')
                self.state = State.READY

        if self.state == State.READY and self.generate_random: 
            while True: 
                pt = np.random.random(2) * self.workspace_size
                if self.check_point(pt):
                    self.ref_point = pt 
                    break
            self.controller.update_goal_point(self.ref_point)
            self.state = State.RANDOM_TRACKING
            self.traj_start_time = time.time()
            self.enable_log()

        # if self.state in [State.RANDOM_TRACKING, State.REFERENCE_TRACKING, State.REFERENCE_STARTING, State.MANUAL_TRACKING_JOINT, State.MANUAL_TRACKING_TASK]:
        self.controller.update_end_point(self.end_point, tracking=self.state == State.REFERENCE_TRACKING)

        if self.state == State.REFERENCE_TRACKING:
            # Following reference trajectory
            self.ref_point = self.ref_squence[self.ref_i]
            self.controller.update_goal_point(self.ref_point)
            self.ref_i += 1
            if self.ref_i == len(self.ref_squence):
                self.ref_squence = []
                self.ref_i = 0
                self.state = State.READY
                self.disable_log("TRACKING_FINISHED")

        if self.state == State.REFERENCE_STARTING:
            # Moving towards start of reference trajectory 
            if np.linalg.norm(self.end_point - self.ref_squence[0]) < 0.02:
                self.holdtime = time.time() + 5 
                self.state = State.HOLDING

        if self.state == State.HOLDING: 
            if time.time() > self.holdtime: 
                self.holdtime = 0 
                self.state = State.REFERENCE_TRACKING
                self.enable_log()
            self.u = [0, 0]
        else:
            # Get reference signal for current state 
            self.u = self._get_ref()

        # Sends motor commands, comment to run system without motion 
        # This has no effect on learning based models that do not use PID 
        self.motor_api.set_ref(self.u)
        self._log() 

    def _bound_extension(self, u):
        '''Limits robot to operate on one side of its bending ability
        '''
        print("Pre-bounding", u)
        if self.motor_api.type == 'pos':
            return [max(0, u[0]), max(0, u[1])]
        elif self.motor_api.type == 'vel':
            motor_pos = [self.motor_api.mtr_data.mtr1.position.value, self.motor_api.mtr_data.mtr2.position.value]
            if u[0] > 0:
                u0 = 0 if motor_pos[0] > 0 else u[0] * (abs(motor_pos[0]) / self.bound_buffer if motor_pos[0] < self.bound_buffer else 1)
            else: 
                u0 = u[0]
            if u[1] > 0:
                u1 = 0 if motor_pos[1] > 0 else u[1] * (abs(motor_pos[1]) / self.bound_buffer if motor_pos[1] < self.bound_buffer else 1)
            else:
                u1 = u[1]
            ret = [u0, u1]
            return ret

    def _get_ref(self):
        '''Gets setpoint for motor api based on current system state 
        '''
        if self.state in [State.RANDOM_TRACKING, State.REFERENCE_TRACKING, State.REFERENCE_STARTING, State.MANUAL_TRACKING_TASK]:
            return self._bound_extension(self.controller.get_command())
        elif self.state == State.MANUAL_TRACKING_JOINT:
            return self.ref_point
        elif self.state == State.RECOVER: 
            return [0, 0] #self.recovery_sequence[self.recovery_i-1]
        elif self.state in [State.STOPPED, State.READY, State.ERROR]:
            return None

    def enable_log(self):
        self.log_file_start_time = time.time()
        file_root = os.path.join(self.log_dir, str(self.log_file_start_time)) # Generates unique filename

        self.aurora_api.enable_log(file_root)
        self.motor_api.enable_log(file_root)

        self.log_file = open(file_root + "_controller.txt", 'a')
        self.log_file.write('time,state\n')
        self.log_flag = True
        self.log_start_pos = self.end_point

    def disable_log(self, msg = None): 
        if self.log_file_start_time is not None:
            self.meta_log.write(f'{self.log_file_start_time},{msg},{time.time() - self.log_file_start_time},{self.log_start_pos[0]},{self.log_start_pos[1]},{self.end_point[0]},{self.end_point[1]}\n')
        # Catch case where thread dying causes termination 
        try:
            self.motor_api.disable_log(msg)
        except:
            pass
        try:
            self.aurora_api.disable_log(msg)
        except:
            pass

        self.log_flag = False
        if self.log_file is not None:
            if msg is not None and not self.log_file.closed:
                self.log_file.write(msg + '\n')
            self.log_file.close()
        self.log_file_start_time = None
        self.log_start_pos = None

    def _log(self):
        if self.log_flag:
            timestep = time.time()
            self.log_file.write(f"{timestep},{self.state}\n")
    
