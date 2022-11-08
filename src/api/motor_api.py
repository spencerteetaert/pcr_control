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
GEAR_RATIO = 4
Kp = 3
Kd = 0
Ki = 5

MAX_SPEED = 2


class MotorController:
    def __init__(self, auto_tension=True):
        self.mtr_datas = [MotorData(), MotorData()]
        self.adcs = [AdcResult(), AdcResult()]
        self.busses = [can.interface.Bus('can0', bitrate=BITRATE), can.interface.Bus('can1', bitrate=BITRATE)]
        self.vels = [0,0,0,0]
        self.enabled = False
        self.auto_tension = auto_tension

        print("Setup controller with Kp = {}, Ki = {} Kd = {}".format(Kp, Ki, Kd))
        print("Max speed: {}".format(MAX_SPEED))
        self.vctrl = [
            VelocityController(self.busses[0], Kp, Ki, Kd), 
            VelocityController(self.busses[0], Kp, Ki, Kd), 
            VelocityController(self.busses[1], Kp, Ki, Kd), 
            VelocityController(self.busses[1], Kp, Ki, Kd)
        ]

        self.msg_handlers = [MessageHandler(), MessageHandler()]

        self.msg_handlers[0].set_id_handler(ArbitrationIds.status, self.mtr_datas[0].set_status)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.current, self.mtr_datas[0].set_current)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.position, self.mtr_datas[0].set_position)
        self.msg_handlers[0].set_id_handler(ArbitrationIds.velocity, partial(self._on_velocity_msg, motor_idx=[0, 1]))
        self.msg_handlers[0].set_id_handler(ArbitrationIds.adc6, self.adcs[0].set_values)

        self.msg_handlers[1].set_id_handler(ArbitrationIds.status, self.mtr_datas[1].set_status)
        self.msg_handlers[1].set_id_handler(ArbitrationIds.current, self.mtr_datas[1].set_current)
        self.msg_handlers[1].set_id_handler(ArbitrationIds.position, self.mtr_datas[1].set_position)
        self.msg_handlers[1].set_id_handler(ArbitrationIds.velocity, partial(self._on_velocity_msg, motor_idx=[2, 3]))
        self.msg_handlers[1].set_id_handler(ArbitrationIds.adc6, self.adcs[1].set_values)

        self.handler0 = CanHandler(0, self.busses[0], self.msg_handlers[0])
        self.handler1 = CanHandler(1, self.busses[1], self.msg_handlers[1])
        self.t0 = threading.Thread(target=self.handler0.loop)
        self.t1 = threading.Thread(target=self.handler1.loop)

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
        start_system(self.busses[0], self.mtr_datas[0], False)
        start_system(self.busses[1], self.mtr_datas[1], False)

        self.t0.start()
        self.t1.start()

        self.enabled = True

    def disable(self):
        '''
        Disables microcontrollers and stops canbus message handling threads
        '''
        print("Disabling motors...")
        stop_system(self.busses[0])
        stop_system(self.busses[1])

        self.handler0.kill = True
        self.handler1.kill = True

        self.t0.join()
        self.t1.join()

    def get_sensor_data(self):
        return None

    def set_velocity(self, vels):
        '''
        Sets reference velocities for each of 4 motors

        Args: 
            vels (list): 4 motor velocity values (rps)
        '''
        if self.enabled:
            self.vels[0] = vels[0] / GEAR_RATIO
            self.vels[1] = vels[1] / GEAR_RATIO
            self.vels[2] = vels[2] / GEAR_RATIO
            self.vels[3] = 0 / GEAR_RATIO # MANUAL OVERRIDE SO I DONT ACCIDENTALLY TURN THIS ON
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
            self.vctrl[motor_idx[0]].update_data(self.mtr_datas[board_idx].mtr1)
            self.vctrl[motor_idx[1]].update_data(self.mtr_datas[board_idx].mtr2)
            
            self.vctrl[motor_idx[0]].run(self.vels[motor_idx[0]])
            self.vctrl[motor_idx[1]].run(self.vels[motor_idx[1]])

            if self.auto_tension:
                i_m0 = min(self.vctrl[motor_idx[0]].iqref, -0.03)
                i_m1 = min(self.vctrl[motor_idx[1]].iqref, -0.03)
            else:
                i_m0 = self.vctrl[motor_idx[0]].iqref # right motor, pos on left bend, neg on right 
                i_m1 = self.vctrl[motor_idx[1]].iqref # left motor, pos on right bend, neg on left 

            send_mtr_current(self.busses[board_idx], i_m0, i_m1)
        else:
            send_mtr_current(self.busses[board_idx], 0, 0)

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