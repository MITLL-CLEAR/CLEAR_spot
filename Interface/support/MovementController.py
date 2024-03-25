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
This class interpolates commands and from the server to movements on spot
"""

import threading

class MovementController() :
    def __init__(self, drone) :
        self.drone = drone
        self.for_back_velocity = 0
        self.vertical_velocity = 0
        self.left_right_velocity = 0
        self.yaw_velocity = 0
        self.vertical_velocity = 0
        self.command = None
        self.awaitingMoveCompletion = False
        self.lock = threading.Lock()  # create a lock object
        self.counter = 0
        resetRotation = False
    
    def set(self, value) -> None:
        try :
            self.for_back_velocity = float(value[0])
            self.left_right_velocity = float(value[1])
            self.vertical_velocity = float(value[2])
            self.yaw_velocity = float(value[3])   
        except Exception as e :
            with open("log.txt", "w") as file:
                file.write("The exception is {}".format(e))

    #Commands are state changes to spot. While a command is executing,
    # spot will not being receiving additional updates from the controller
    def commandGiven(self, input) :
        try :
            # Changes the camera to the correct view
            if "readyGrab" == input :
                self.drone.controller.arm.readyToGrab = True
                self.drone.feedbackHandler.giveComment("changingCamera")
                return True

            if "readyRegCam" == input:
                self.drone.controller.arm.readyToGrab = False
                self.drone.feedbackHandler.giveComment("changingCamera")

            if "grab" in input :
                point = extractTupleFromString(input)
                try :
                    self.drone.controller.arm.arm_object_grasp(point=point)

                    return True
                except Exception as e :
                    print("grabbing went arry : ", e)

            if input == "stand" :
                return self.drone.controller._stand()

            if input == "sit" :
                return self.drone.controller._sit()
            
            if input == "dock" :
                return self.drone.controller._dock()
            
        except Exception as e :
            with open("log.txt", "w") as file:
                file.write("The exception is {}".format(e))
        
        return False
    
    def readyToMove(self) :
        return (self.thereIsMovement())

    def stop_Movement(self) :
        self.for_back_velocity = 0
        self.vertical_velocity = 0
        self.left_right_velocity = 0
        self.yaw_velocity = 0
        self.resetRotation = True

    # returns True if there is movement    
    def thereIsMovement(self) :
        return (self.for_back_velocity != 0
        or self.vertical_velocity != 0
        or self.left_right_velocity != 0
        or self.yaw_velocity != 0)
    
    def moveDrone(self):
        # try to acquire the lock without blocking
        if not self.lock.acquire(blocking=False):
            return

        try:
            self.drone.controller._autoMove(
                self.for_back_velocity,
                self.left_right_velocity,
                self.yaw_velocity,
                self.vertical_velocity,
                self.resetRotation
            )

            self.resetRotation = False
            self.counter += 1
            with open("where.txt", "w") as file:
                file.write("The num is {}".format(self.counter))

        except Exception as e:
            with open("moveError.txt", "w") as file:
                file.write("The exception is {}".format(e))

        finally:
            # make sure to release the lock in any case
            self.lock.release()


def extractTupleFromString(strVal) :
    start = strVal.find('(') + 1
    end = strVal.find(')')
    numbers = strVal[start:end].split(',')
    return tuple(int(num) for num in numbers)
