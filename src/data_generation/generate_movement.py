import time 
import os 

def do_movement(workspace, model, aurora, controller):
    # Reset position from Aurora 
    # model.update_end_point(aurora.get_position())

    filename = os.path.join(workspace, str(time.time()))

    # model.enable_log(filename)
    aurora.enable_log(filename)
    controller.enable_log(filename)

    time.sleep(10)
    #movement code goes here 


    # model.disable_log()
    aurora.disable_log()
    controller.disable_log()





