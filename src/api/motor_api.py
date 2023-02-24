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
import matplotlib.pyplot as plt
import matplotlib.animation as animation 
plt.style.use('fivethirtyeight')

from blmc.motor_data import MotorData, AdcResult, MessageHandler, ArbitrationIds
from blmc.controllers import PositionController, VelocityController
from blmc.can_helper import start_system, send_mtr_current, stop_system

BITRATE = 1e6


MAX_SPEED = 2
HOLDING_CURRENT = -0.2
MAX_CURRENT = 2.5

SHOW_CURRENT = False

class MotorController:
    def __init__(self, type='vel', auto_tension=False, log_rate=100):
        '''
        Args: 
            auto_tension (bool): whether to enact a minimum holding current in motors
            log_rate (int): frequency (hz) that motors will log at 
        '''
        self.mtr_data = MotorData()
        self.adc = AdcResult()
        self.bus = can.ThreadSafeBus('can0', bitrate=BITRATE)
        self.type = type
        assert self.type in ['pos', 'vel'], 'Invalid control type.'

        self.vels = None
        self.pos = None
        self.manual_override = False
        self.enabled = False
        self.auto_tension = auto_tension
        self.file = None
        self.log_flag = False
        self.log_rate = log_rate
        self.last_log_timestep = time.time()
        self.recover = False

        Kp = 20
        Kd = 0.2
        Ki = 10
        self.pctrl = [
            PositionController(Kp, Ki, Kd), 
            PositionController(Kp, Ki, Kd)
        ]
        Kp = 0.05
        Kd = 0.1
        Ki = 0.005
        self.pctrl_recovery = [
            PositionController(Kp, Ki, Kd), 
            PositionController(Kp, Ki, Kd)
        ]
        Kp = 3
        Kd = 0
        Ki = 60
        self.vctrl = [
            VelocityController(self.bus, Kp, Ki, Kd), 
            VelocityController(self.bus, Kp, Ki, Kd)
        ]

        print("Setup controller with Kp = {}, Ki = {} Kd = {}".format(Kp, Ki, Kd))
        print("Max speed: {}".format(MAX_SPEED))

        self.msg_handler = MessageHandler()

        self.msg_handler.set_id_handler(ArbitrationIds.status, self.mtr_data.set_status)
        self.msg_handler.set_id_handler(ArbitrationIds.current, self.mtr_data.set_current)

        self.msg_handler.set_id_handler(ArbitrationIds.position, self._position_msg_switch)
        self.msg_handler.set_id_handler(ArbitrationIds.velocity, self._velocity_msg_switch)

        self.msg_handler.set_id_handler(ArbitrationIds.adc6, self.adc.set_values)

        self.handler = CanHandler(0, self.bus, self.msg_handler)
        self.thread = threading.Thread(target=self.handler.loop)

        # setup sigint handler to disable motor on CTRL+C
        def sigint_handler(signal, frame):
            print("Stop motor and shut down.")
            self.disable()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)

        if SHOW_CURRENT:
            self.current_readings = []
            self.tension_markers = []

            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(1, 1, 1)
            self.xs = list(range(20))

            self.ani = animation.FuncAnimation(self.fig, self.update_current_graph, interval=100)
            # plt.show()

    def reset_pid(self, recovery=False):
        self.vctrl[0]._pid.Initialize()
        self.vctrl[1]._pid.Initialize()
        self.pctrl[0]._pid.Initialize()
        self.pctrl[1]._pid.Initialize()
        self.pctrl_recovery[0]._pid.Initialize()
        self.pctrl_recovery[1]._pid.Initialize()
        self.recover = recovery

    def enable(self):
        '''Enables microcontrollers and starts canbus message handling threads
        '''
        print("Enabling motors...")
        
        self.bus.flush_tx_buffer()
        start_system(self.bus, self.mtr_data, False)
        self.thread.start()
        self.enabled = True

    def disable(self):
        '''Disables microcontrollers and stops canbus message handling threads
        '''
        print("Disabling motors...")
        self.disable_log()
        self.bus.flush_tx_buffer()
        stop_system(self.bus)
        self.handler.kill = True
        self.thread.join()
        self.bus.shutdown()

    def get_sensor_data(self):
        return None

    
    #region Logging 
    def enable_log(self, filename):
        self.file = open(filename + "_motor.txt", 'a')
        self.file.write('time,command_i1,command_i2,set_v1,set_v2,i1,i2,pos1,pos2,vel1,vel2\n')
        self.log_flag = True

    def disable_log(self):
        self.log_flag = False
        if self.file is not None:
            self.file.close()

    def log(self, i1, i2):
        if self.log_flag:
            timestep = time.time()
            if timestep - self.last_log_timestep < 1/self.log_rate:
                # Enforces maximum log rate 
                return

            self.last_log_timestep = timestep
            self.file.write(f"{timestep},{i1},{i2},{self.vels[0]},{self.vels[1]},{self.mtr_data.mtr1.current.value},{self.mtr_data.mtr2.current.value},{self.mtr_data.mtr1.position.value},{self.mtr_data.mtr2.position.value},{self.mtr_data.mtr1.velocity.value},{self.mtr_data.mtr2.velocity.value}\n")
    
    def update_current_graph(self, i):
        self.current_readings += [[self.mtr_datas.mtr1.current.value, self.mtr_datas.mtr2.current.value]]
        self.current_readings = self.current_readings[-20:]

        self.ax.clear()
        self.ax.plot(self.xs[:len(self.current_readings)], self.current_readings)
    #endregion 

    #region Control
    def set_ref(self, ref):
        '''
        Sets reference velocities for each of 4 motors

        Args: 
            pos (list): 2 tendon position values for positive curvature tendon
        '''
        if self.type == 'pos':
            self.pos = ref
        elif self.type == 'vel':
            self.vels = ref

    def _position_msg_switch(self, msg):
        if self.type == 'pos':
            self._on_position_msg(msg)
        else:
            self.mtr_data.set_position(msg)
    def _velocity_msg_switch(self, msg):
        if self.type == 'vel':
            self._on_velocity_msg(msg)
        else:
            self.mtr_data.set_velocity(msg)

    def _on_position_msg(self, msg):
        if self.enabled:
            if self.recover:
                ctrl = self.pctrl_recovery
            else:
                ctrl = self.pctrl
            self.mtr_data.set_position(msg)

            if self.mtr_data.status.mtr1_ready and self.mtr_data.status.mtr2_ready and self.pos is not None:
                i_m0 = 0 
                i_m1 = 0

                ctrl[0].update_data(self.mtr_data.mtr1)
                ctrl[1].update_data(self.mtr_data.mtr2)
                ctrl[0].run(self.pos[0])
                ctrl[1].run(self.pos[1])

                if self.auto_tension:
                    i_m0 = max(ctrl[0].iqref, HOLDING_CURRENT)
                    i_m1 = max(ctrl[1].iqref, HOLDING_CURRENT)
                else:
                    i_m0 = ctrl[0].iqref # right motor, pos on left bend, neg on right 
                    i_m1 = ctrl[1].iqref # left motor, pos on right bend, neg on left 

                i_m0 = max(-MAX_CURRENT, min(MAX_CURRENT, i_m0))
                i_m1 = max(-MAX_CURRENT, min(MAX_CURRENT, i_m1))

                send_mtr_current(self.bus, i_m0, i_m1)
                self.log(i_m0, i_m1)
            else:
                send_mtr_current(self.bus, 0, 0)
                self.log(0, 0)
    
    def _on_velocity_msg(self, msg):
        if self.enabled:
            self.mtr_data.set_velocity(msg)

            # emergency break
            if self.mtr_data.mtr1.velocity.value > MAX_SPEED * 3 or self.mtr_data.mtr2.velocity.value > MAX_SPEED * 3:
                self.disable()
                print("Too fast! EMERGENCY BREAK!")
                sys.exit(0)

            if self.mtr_data.status.mtr1_ready and self.mtr_data.status.mtr2_ready and self.vels is not None:
                i_m0 = 0 
                i_m1 = 0

                self.vctrl[0].update_data(self.mtr_data.mtr1)
                self.vctrl[1].update_data(self.mtr_data.mtr2)
                self.vctrl[0].run(self.vels[0])
                self.vctrl[1].run(self.vels[1])

                if self.auto_tension:
                    i_m0 = max(self.vctrl[0].iqref, HOLDING_CURRENT)
                    i_m1 = max(self.vctrl[1].iqref, HOLDING_CURRENT)
                else:
                    i_m0 = self.vctrl[0].iqref # right motor, pos on left bend, neg on right 
                    i_m1 = self.vctrl[1].iqref # left motor, pos on right bend, neg on left 

                i_m0 = max(-MAX_CURRENT, min(MAX_CURRENT, i_m0))
                i_m1 = max(-MAX_CURRENT, min(MAX_CURRENT, i_m1))

                send_mtr_current(self.bus, i_m0, i_m1)
                self.log(i_m0, i_m1)
            else:
                send_mtr_current(self.bus, 0, 0)
                self.log(0, 0)
    #endregion

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
    controller = MotorController(autotension = False)
    controller.enable()

    if input('This operation will start the motors. Are you sure? [y/N]') == 'y':
        controller.set_velocity([1, 2])
        time.sleep(30)
        controller.disable()
    else:
        print('Operation cancelled.')