#!/usr/bin/env python
"""
https://open-dynamic-robot-initiative.github.io/udriver_firmware/can/can_connection.html 

https://github.com/open-dynamic-robot-initiative/python_blmc

TODO: 
    - Double check velocity units 
    - Tune PID for robot 
    - remove motor 4 override 
"""

import sys
import threading
from functools import partial
import time
import signal

import can
can.rc['interface'] = 'socketcan'

from blmc.motor_data import MotorData, AdcResult, MessageHandler, ArbitrationIds
from blmc.controllers import VelocityController
from blmc.can_helper import start_system, send_mtr_current, stop_system

BITRATE = 1e6
Kp = 3
Kd = 0
Ki = 60

MAX_SPEED = 2
HOLDING_CURRENT = 0.01
MAX_CURRENT = 2.5

class MotorController:
    def __init__(self, auto_tension=True, log_rate=100):
        '''
        Args: 
            auto_tension (bool): whether to enact a minimum holding current in motors
            log_rate (int): frequency (hz) that motors will log at 
        '''
        self.mtr_datas = [MotorData()]#, MotorData()]
        self.adcs = [AdcResult()]#, AdcResult()]
        self.busses = [can.ThreadSafeBus('can0', bitrate=BITRATE)]#, can.ThreadSafeBus('can1', bitrate=BITRATE)]

        self.vels = [0,0,0,0]
        self.manual_override = False
        self.enabled = False
        self.auto_tension = auto_tension
        self.file = None
        self.log_flag = False
        self.log_rate = log_rate
        self.last_timestep0 = time.time()
        # self.last_timestep1 = time.time()

        print("Setup controller with Kp = {}, Ki = {} Kd = {}".format(Kp, Ki, Kd))
        print("Max speed: {}".format(MAX_SPEED))
        self.vctrl = [
            VelocityController(self.busses[0], Kp, Ki, Kd), 
            VelocityController(self.busses[0], Kp, Ki, Kd), 
            # VelocityController(self.busses[1], Kp, Ki, Kd), 
            # VelocityController(self.busses[1], Kp, Ki, Kd)
        ]

        self.msg_handlers = [MessageHandler(), MessageHandler()]

        self.msg_handlers[0].set_id_handler(ArbitrationIds.status, self.mtr_datas[0].set_status)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.current, self.mtr_datas[0].set_current)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.position, self.mtr_datas[0].set_position)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.velocity, partial(self._on_velocity_msg, motor_idx=[0, 1]))
        self.msg_handlers[0].set_id_handler(ArbitrationIds.adc6, self.adcs[0].set_values)

        # self.msg_handlers[1].set_id_handler(ArbitrationIds.status, self.mtr_datas[1].set_status)
        # self.msg_handlers[1].set_id_handler(ArbitrationIds.current, self.mtr_datas[1].set_current)
        # self.msg_handlers[1].set_id_handler(ArbitrationIds.position, self.mtr_datas[1].set_position)
        # self.msg_handlers[1].set_id_handler(ArbitrationIds.velocity, partial(self._on_velocity_msg, motor_idx=[2, 3]))
        # self.msg_handlers[1].set_id_handler(ArbitrationIds.adc6, self.adcs[1].set_values)

        self.handler0 = CanHandler(0, self.busses[0], self.msg_handlers[0])
        # self.handler1 = CanHandler(1, self.busses[1], self.msg_handlers[1])
        self.t0 = threading.Thread(target=self.handler0.loop)
        # self.t1 = threading.Thread(target=self.handler1.loop)

        # setup sigint handler to disable motor on CTRL+C
        def sigint_handler(signal, frame):
            print("Stop motor and shut down.")
            self.disable()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)

    def enable(self):
        '''
        Enables microcontrollers and starts canbus message handling threads
        '''
        print("Enabling motors...")

        self.busses[0].flush_tx_buffer()
        # self.busses[1].flush_tx_buffer()

        start_system(self.busses[0], self.mtr_datas[0], False)
        # start_system(self.busses[1], self.mtr_datas[1], False)

        self.t0.start()
        # self.t1.start()

        self.enabled = True

    def disable(self):
        '''
        Disables microcontrollers and stops canbus message handling threads
        '''
        print("Disabling motors...")
        self.busses[0].flush_tx_buffer()
        # self.busses[1].flush_tx_buffer()
        
        stop_system(self.busses[0])
        # stop_system(self.busses[1])

        self.handler0.kill = True
        # self.handler1.kill = True

        self.t0.join()
        # self.t1.join()
        
        self.busses[0].shutdown()
        # self.busses[1].shutdown()

    def get_sensor_data(self):
        return None

    def enable_log(self, filename):
        self.file0 = open(filename + "_motor0.txt", 'a')
        self.file0.write('time,command_i1,command_i2,set_v1,set_v2,i1,i2,pos1,pos2,vel1,vel2\n')
        # self.file1 = open(filename + "_motor1.txt", 'a')
        # self.file1.write('time,command_i1,command_i2,set_v1,set_v2,i1,i2,pos1,pos2,vel1,vel2\n')
        self.log_flag = True

    def disable_log(self):
        self.log_flag = False
        if self.file0 is not None:
            self.file0.close()
            # self.file1.close()

    def log(self, board, i1, i2):
        if self.log_flag:
            timestep = time.time()
            
            if board:
                if timestep - self.last_timestep1 < 1/self.log_rate:
                    return
                file = self.file1
                self.last_timestep1 = timestep
            else:
                if timestep - self.last_timestep0 < 1/self.log_rate:
                    return
                file = self.file0
                self.last_timestep0 = timestep

            file.write(f"{timestep},{i1},{i2},{self.vels[0+2*board]},{self.vels[1+2*board]},{self.mtr_datas[board].mtr1.current.value},{self.mtr_datas[board].mtr2.current.value},{self.mtr_datas[board].mtr1.position.value},{self.mtr_datas[board].mtr2.position.value},{self.mtr_datas[board].mtr1.velocity.value},{self.mtr_datas[board].mtr2.velocity.value}\n")


    def set_velocity(self, vels):
        '''
        Sets reference velocities for each of 4 motors

        Args: 
            vels (list): 2 tendon velocity values for positive curvature tendon (left tendon)
        '''
        if self.enabled:
            self._set_velocity([vels[0], vels[0], vels[1], vels[1]])
            self.manual_override = False
        else:
            raise AssertionError("Motor is not enabled.")

    def _set_velocity(self, vels):
        '''
        Sets reference velocities for each of 4 motors

        Args: 
            vels (list): 4 motor velocity values (rps)
        '''
        if self.enabled:
            self.vels = vels
            self.manual_override = True
        else:
            raise AssertionError("Motor is not enabled.")

    def _on_velocity_msg(self, msg, motor_idx):
        board_idx = motor_idx[0] // 2

        self.mtr_datas[board_idx].set_velocity(msg)

        # emergency break
        if self.mtr_datas[board_idx].mtr1.velocity.value > MAX_SPEED * 3 or self.mtr_datas[board_idx].mtr2.velocity.value > MAX_SPEED * 3:
            self.disable()
            print("Too fast! EMERGENCY BREAK!")
            sys.exit(0)

        if self.mtr_datas[board_idx].status.mtr1_ready and self.mtr_datas[board_idx].status.mtr2_ready:
            i_m0 = 0 
            i_m1 = 0

            if not self.manual_override:
                if self.vels[motor_idx[0]] >= 0: # Actively driven 
                    self.vctrl[motor_idx[0]].update_data(self.mtr_datas[board_idx].mtr1)
                    self.vctrl[motor_idx[0]].run(self.vels[motor_idx[0]])

                    if self.auto_tension:
                        i_m0 = max(self.vctrl[motor_idx[0]].iqref, HOLDING_CURRENT)
                        i_m1 = -HOLDING_CURRENT
                    else:
                        i_m0 = self.vctrl[motor_idx[0]].iqref # right motor, pos on left bend, neg on right 
                        i_m1 = 0 # left motor, pos on right bend, neg on left 
                
                elif self.vels[motor_idx[1]] < 0: # Actively driven 
                    self.vctrl[motor_idx[1]].update_data(self.mtr_datas[board_idx].mtr2)
                    self.vctrl[motor_idx[1]].run(self.vels[motor_idx[1]])

                    if self.auto_tension:
                        i_m0 = HOLDING_CURRENT
                        i_m1 = min(self.vctrl[motor_idx[1]].iqref, -HOLDING_CURRENT)
                    else:
                        i_m0 = 0 # right motor, pos on left bend, neg on right 
                        i_m1 = self.vctrl[motor_idx[1]].iqref # left motor, po

            else:
                self.vctrl[motor_idx[0]].update_data(self.mtr_datas[board_idx].mtr1)
                self.vctrl[motor_idx[1]].update_data(self.mtr_datas[board_idx].mtr2)
                self.vctrl[motor_idx[0]].run(self.vels[motor_idx[0]])
                self.vctrl[motor_idx[1]].run(self.vels[motor_idx[1]])

                if self.auto_tension:
                    i_m0 = max(self.vctrl[motor_idx[0]].iqref, HOLDING_CURRENT)
                    i_m1 = min(self.vctrl[motor_idx[1]].iqref, -HOLDING_CURRENT)
                else:
                    i_m0 = self.vctrl[motor_idx[0]].iqref # right motor, pos on left bend, neg on right 
                    i_m1 = self.vctrl[motor_idx[1]].iqref # left motor, pos on right bend, neg on left 

            i_m0 = min(MAX_CURRENT, i_m0)
            i_m1 = max(-MAX_CURRENT, i_m1)

            send_mtr_current(self.busses[board_idx], i_m0, i_m1)
            self.log(board_idx, i_m0, i_m1)
        else:
            send_mtr_current(self.busses[board_idx], 0, 0)
            self.log(board_idx, 0, 0)


class CanHandler:
    def __init__(self, handlerID, bus, msg_handler):
        self.bus = bus
        self.msg_handler = msg_handler
        self.handlerID = handlerID
        self.kill = False

    def loop(self):
        for msg in self.bus:
            if self.kill:
                break

            self.msg_handler.handle_msg(msg)

if __name__ == "__main__":
    controller = MotorController(False)

    ''''
    Computer 
    M0     M1 
    M2     M3
    '''
    controller.enable()

    print("Starting trajectory")
    controller.set_velocity([1, 2, 0.2, 0])
    time.sleep(30)


    controller.disable()