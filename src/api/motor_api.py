#!/usr/bin/env python
"""
https://open-dynamic-robot-initiative.github.io/udriver_firmware/can/can_connection.html 
"""

import time
import can
import can
can.rc['interface'] = 'socketcan'
# can.rc['channel'] = 'can0'
can.rc['bitrate'] = 1000000
# can.util.load_config()
from can.interface import Bus

# bus = Bus('can0', interface='socketcan', bitrate=1000000)
import signal
import sys
import traceback
from blmc.motor_data import MotorData, AdcResult, MessageHandler, ArbitrationIds, StampedValue
from blmc.controllers import VelocityController
from blmc.can_helper import start_system, send_mtr_current, send_command, Command

'''
TODO:
- get motor speed feedback, currently none 
- get slow running control working 
- figure out what adcresult is and how to work with it 
- extend message handling to all 4 motors



class Command:
    enable_sys = 1
    enable_mtr1 = 2
    enable_mtr2 = 3
    enable_vspring1 = 4
    enable_vspring2 = 5
    send_current = 12
    send_position = 13
    send_velocity = 14
    send_adc6 = 15
    send_all = 20
'''


BITRATE = 1e6
Kp = 1
Ki = 1
MAX_SPEED = 2


class MotorController:
    def __init__(self):
        self.mtr_data0 = MotorData()
        self.mtr_data1 = MotorData()
        self.adc0 = AdcResult()
        self.adc1 = AdcResult()
        self.bus0 = can.interface.Bus('can0', bitrate=BITRATE)
        self.bus1 = can.interface.Bus('can1', bitrate=BITRATE)

        # setup sigint handler to disable motor on CTRL+C
        def sigint_handler(signal, frame):
            print("Stop motor and shut down.")
            self.disable()
            sys.exit(0)
        signal.signal(signal.SIGINT, sigint_handler)
        signal.signal(signal.SIGINT, sigint_handler)

        print("Setup controller with Kp = {}, Ki = {}".format(Kp, Ki))
        print("Max speed: {}".format(MAX_SPEED))
        self.vctrl0 = VelocityController(self.bus0, Kp, Ki, 0)
        self.vctrl1 = VelocityController(self.bus0, Kp, Ki, 0)
        self.vctrl2 = VelocityController(self.bus1, Kp, Ki, 0)
        self.vctrl3 = VelocityController(self.bus1, Kp, Ki, 0)

        self.msg_handler0 = MessageHandler()
        self.msg_handler0.set_id_handler(ArbitrationIds.status, self.mtr_data0.set_status)
        self.msg_handler0.set_id_handler(ArbitrationIds.current, self.mtr_data0.set_current)
        # self.msg_handler0.set_id_handler(ArbitrationIds.position, self.mtr_data0.set_position)
        # self.msg_handler0.set_id_handler(ArbitrationIds.velocity, on_velocity_msg)
        self.msg_handler0.set_id_handler(ArbitrationIds.adc6, self.temp)


        self.enable()

    def temp(self, msg):
        print("MESSAGE", msg)

    def enable(self):
        # start_system(self.bus0, self.mtr_data0)
        # start_system(self.bus1, self.mtr_data1)
        print("Enable system...")
        send_command(self.bus0, Command.enable_sys, 1)
        send_command(self.bus1, Command.enable_sys, 1)

        print("Enable motor...")
        send_mtr_current(self.bus0, 0, 0)  # start with zero
        send_mtr_current(self.bus0, 0, 0)  # start with zero
        send_command(self.bus0, Command.enable_mtr1, 1)
        send_command(self.bus0, Command.enable_mtr2, 1)
        send_command(self.bus1, Command.enable_mtr1, 1)
        send_command(self.bus1, Command.enable_mtr2, 1)

    def disable(self):
        print("Killing motors...")
        send_command(self.bus0, Command.enable_sys, 0)
        send_command(self.bus1, Command.enable_sys, 0)
        send_mtr_current(self.bus0, 0, 0)  # start with zero
        send_mtr_current(self.bus1, 0, 0)  # start with zero
        send_command(self.bus0, Command.enable_mtr1, 0)
        send_command(self.bus0, Command.enable_mtr2, 0)
        send_command(self.bus1, Command.enable_mtr1, 0)
        send_command(self.bus1, Command.enable_mtr2, 0)


    def set_velocities(self, vels):
        vels[3] = 0 # MANUAL OVERRIDE SO I DONT ACCIDENTALLY TURN THIS ON
        send_mtr_current(self.bus0, *vels[:2])
        send_mtr_current(self.bus1, *vels[2:])

    def on_velocity_msg(self, vels):

        # emergency break
        if self.mtr_data0.mtr1.velocity.value > MAX_SPEED * 3 or self.mtr_data0.mtr2.velocity.value > MAX_SPEED * 3:
            self.disable()
            print("Too fast! EMERGENCY BREAK!")
            sys.exit(0)

        # if self.mtr_data0.status.mtr1_ready and self.mtr_data0.status.mtr2_ready:
        self.vctrl0.update_data(self.mtr_data0.mtr1)
        self.vctrl1.update_data(self.mtr_data0.mtr2)

        self.vctrl0.run(MAX_SPEED * vels[0])
        self.vctrl1.run(MAX_SPEED * vels[1])

        send_mtr_current(self.bus0, self.vctrl0.iqref, self.vctrl1.iqref)
        # else:
        #     send_mtr_current(self.bus0, 0, 0)
        print()


if __name__ == "__main__":
    controller = MotorController()

    ''''
    Computer 
    M0     M1 
    M2     M3
    '''
    start = time.time()

    controller.mtr_data0.mtr1.velocity = StampedValue(0, time.time())
    controller.mtr_data0.mtr2.velocity = StampedValue(0, time.time())

    timestamp = time.time()
    m0_vel = 0
    m1_vel = 0

    # wait for messages and update data
    while time.time() - start < 20:
        dt = time.time() - timestamp
        timestamp = time.time()
        m0_vel += dt * controller.vctrl0.iqref
        m1_vel += dt * controller.vctrl1.iqref

        controller.mtr_data0.mtr1.velocity = StampedValue(m0_vel, timestamp)
        controller.mtr_data0.mtr2.velocity = StampedValue(m1_vel, timestamp)

        controller.on_velocity_msg([0.1, 0, 0, 0])

        time.sleep(0.001)


    # for msg in controller.bus1:
        
    #     # try:
    #     print(msg)
    #     controller.msg_handler0.handle_msg(msg)

    #     if time.time() - start > 3:
    #         break
    #     # except Exception:
    #     #     print("\n\n=========== ERROR ============")
    #     #     print(traceback.format_exc())
    #     #     send_msg(bus, msg_disable_system)
    #     #     break
    #     # 
    controller.disable()