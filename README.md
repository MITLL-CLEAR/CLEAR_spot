# CLEAR_spot
The spot service is a CLEAR robot implementation that directly interfaces with Boston Dynamics spot hardware. The spot service controls the robot according to the instructions of the coordinator. The spot service receives various commands from the coordinator, such as velocities, sit, stand, grab, and dock.
 
Run the CLEAR spot service on a Windows or Unix system in a Python 3.8 interpreter accompanied by the packages expressed in setup/requirements.txt. To run the service, use
 
``python main.py --serverAddress <web address>``

-----

DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
 
This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001. Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the Under Secretary of Defense for Research and Engineering.

© 2023 Massachusetts Institute of Technology.

Subject to FAR52.227-11 Patent Rights - Ownership by the contractor (May 2014)

The software/firmware is provided to you on an As-Is basis

SPDX-License-Identifier: MIT
