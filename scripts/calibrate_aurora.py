import time
import sys
sys.path.append('.')

from src.api.aurora_api import AuroraAPI

aurora = AuroraAPI()
aurora.calibrate(kill_on_completion=False)

# Give time to 
time.sleep(45)

aurora.stop_tracking()

