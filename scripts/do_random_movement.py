import sys
sys.path.append('.')

from src.data_generation.generate_movement import do_movement
# from src.cc_model import CC_Model
from src.api.motor_api import MotorController
from src.api.aurora_api import AuroraAPI

if __name__=='__main__':
    # cc_model = CC_Model()
    controller = MotorController()
    aurora = AuroraAPI(verbose=True)

    controller.enable()
    aurora.start_tracking()  

    do_movement('data/DEL/', None, aurora, controller)

    aurora.stop_tracking()
    controller.disable()