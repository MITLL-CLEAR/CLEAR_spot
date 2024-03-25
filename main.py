# DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.

# This material is based upon work supported by the Under Secretary of Defense for 
# Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions,
# findings, conclusions or recommendations expressed in this material are those 
# of the author(s) and do not necessarily reflect the views of the Under 
# Secretary of Defense for Research and Engineering.

# © 2023 Massachusetts Institute of Technology.

# Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

# The software/firmware is provided to you on an As-Is basis

# Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 
# 252.227-7013 or 7014 (Feb 2014). Notwithstanding any copyright notice, 
# U.S. Government rights in this work are defined by DFARS 252.227-7013 or 
# DFARS 252.227-7014 as detailed above. Use of this work other than as specifically
# authorized by the U.S. Government may violate any copyrights that exist in this work.

"""
launches edge device for controlling spot
"""
from Interface.Interface import Interface
import argparse, os

parser = argparse.ArgumentParser()

defaultWorkerAddress = os.environ.get('WORKER_ADDRESS')

parser.add_argument('--serverAddress', type=str,
  default=defaultWorkerAddress)

# Wifi ip
parser.add_argument('--spotAddress', type=str,
  default="wifi")

serverURL = parser.parse_args().serverAddress
spotURL = parser.parse_args().spotAddress

if str(spotURL).lower == "lan" :
     spotURL = os.environ.get('SPOT_LAN_ADDRESS')
elif str(spotURL).lower == "wifi" :
     spotURL = os.environ.get('SPOT_WIFI_ADDRESS')

print(spotURL)

if __name__ == '__main__':
    robot = Interface(serverAddress = serverURL,
      spotAddress = spotURL)
