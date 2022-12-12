import time
import sys
sys.path.append('.')

from src.api.aurora_api import AuroraAPI

aurora = AuroraAPI(verbose=True)
aurora.calibrate(kill_on_completion=False)

# Give time to validate calibration. Point (1) should be ~(0, 0, 0)
time.sleep(45)

aurora.stop_tracking()

