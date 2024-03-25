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
Can be used to interface spot with autoDrone-intermediary, or also
to command spot directly. Even while acting as an interface, spot may be controlled
manually. 
"""
from typing import Any
import socketio, requests, threading, os, copy
import base64, requests, cv2, numpy as np, time
from SpotController.SpotDriver import main as startDrone
from Interface.support.MovementController import MovementController
from Interface.support.FeedbackHandler import FeedbackHandler

from urllib3.exceptions import InsecureRequestWarning
import warnings
warnings.simplefilter('ignore', InsecureRequestWarning)


VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

class Interface() :
    def __init__(self, serverAddress, spotAddress, 
          intermediary = True) :
        self.controller = None
        self.camera = None
        self.image = None
        self.audio = None
        self.HOSTNAME = spotAddress
        self.shutdown = False
        self.rotation = None
        self.timeToCheckForReadiness = 5
        self.initTime = time.time()
        self.readyToConnect = False
        self.timeSinceImageSend = 0
        self.timeSinceMove = 0

        self.wasMoving = False

        #Toggle on and off for testing
        self.INTERMEDIARY = intermediary

        self.FPS = 2
        
        self.usingHandCam = True

        #IF being used for autodrone systems
        if self.INTERMEDIARY :
            self.lastSentImage = None
            self.systemReady = False
            self.performingCommand = False

             # For having a persistent connection
            self.session = requests.Session()
            # Trust self-signed certificate
            self.session.verify = False
            self.sio = socketio.Client(http_session=self.session)  

            self.sio.on("instruction_updated")(self.instructionUpdated)
            self.sio.on("System_Ready")(self.soReady)
            self.sio.on("readiness_requested")(self.giveReady)

            self.URL = serverAddress
            self.Command = None

            # For having a persistent connection
            serverThread = threading.Thread(target= self.startListen, daemon=True)
            serverThread.start()
            self.movement = MovementController(self)
            self.feedbackHandler = FeedbackHandler(self)
        else :
            self.videoThread = threading.Thread(target= self.displayDroneView, daemon=True)
            self.videoThread.start()
        
        self.start()
    
    #Waits till controller is ready to go 
    def waitingForConnection(self) :
        url = '{}/readyInfo'.format(self.URL)
        self.session.post(url, json={"drone":"drone"}, verify=False)
 
        print("waiting for system to be ready")
        while not self.systemReady:
            response = self.session.get(url,verify=False)
            print("Waiting : ", response)
            if response.status_code == 200 :
                self.systemReady = True
            time.sleep(2)
        print("Drone Ready!!")
        self.initTime = time.time()

    def giveReady(self, message) :
        url = '{}/readyInfo'.format(self.URL)
        self.session.post(url, json={"drone":"drone"}, verify=False)

    def soReady(self, message) :
        self.systemReady = True
       

    def startListen(self):

        if hasattr(self, "sio") :
            self.sio.connect(self.URL)
    
    def disconnect(self):
        if hasattr(self, "sio"):
            self.sio.disconnect()

    # Recieves various commands and movement controls
    def instructionUpdated(self, message) :
        url = '{}/instructionInfo'.format(self.URL)
        response = self.session.get(url, verify=False)

        if response.status_code == 200:
            data = response.json()
            
            # Initialize an empty dictionary to hold information
            self.information = {}
            
            # Iterate through all keys in the returned JSON
            for key in data.keys():
                #sees if attribute exists
                if key == "velocities" :
                    self.timeSinceMove = time.time()
                    self.movement.set(data[key])

                if '?' in data[key] :
                    self.feedbackHandler.giveFeedback(data[key])

                #Commands means that feedback must be given.
                #When commands are issued the controller stalls

                elif key == "Command" :
                    self.performingCommand = True
                    self.movement.commandGiven(data[key])
                    self.performingCommand = False
                    self.feedbackHandler.giveFeedback("state?")

                    # else : setattr(self, key, data[key])
        else:
            print("Error:", response.status_code, response.text)
    
    def queryImage(self) :
        error = ""
        try :
            # to avoid sending same image twice
            if not (self.image is None or 
              self.image is self.lastSentImage) :
                if (float(1/self.FPS) + self.timeSinceImageSend > time.time()) :
                    return 

                img = copy.deepcopy(self.image)
                self.sendImage(img)
                self.timeSinceImageSend = time.time()

        except Exception as e:
            time.sleep(1.0) 
            error = e
            print ("the last error was : ", e)
        if error != "" :
            print ("erred at ", error) 

    def sendImage(self, img):
        self.lastSentImage = img
        status, buffer = cv2.imencode('.webp', img, [cv2.IMWRITE_WEBP_QUALITY, 80])
        bytes_im = buffer.tobytes()
        img_base64 = base64.b64encode(bytes_im)
        url = '{}/image'.format(self.URL)
        # Use the session object for the request
        response = self.session.post(url, json={'image':img_base64.decode('utf-8')})

    def displayDroneView(self) :
        error = ""
        while not self.shutdown :
            try :
                if not self.image is None :
                    img = copy.deepcopy(self.image)
                    cv2.imwrite("img.jpg", img)
                    win = cv2.imshow("Drone", img)
                    cv2.waitKey(1) & 0xff
                else : time.sleep(0.5)
            except Exception as e:
                time.sleep(1.0)
                error = e
                print ("the last error was : ", e)

        if error != "" :
            print ("erred at ", error)
    
    def checkTime(self) :
        if (time.time() - (self.initTime) > 
          self.timeToCheckForReadiness) :
            self.initTime = time.time()
            self.systemReady = False

    def main(self) :
         if self.INTERMEDIARY :
            while not self.readyToConnect :
                time.sleep(0.5)

            self.waitingForConnection()
            while not self.shutdown :
                if self.wasMoving and time.time() > self.timeSinceMove + 0.5:
                        self.movement.stop_Movement()
                        self.movement.moveDrone()
                        self.wasMoving = False
                elif self.movement.thereIsMovement() :
                    self.wasMoving = True
                    self.movement.moveDrone()
                    
                self.queryImage()

    def start(self) :
        mainThread = threading.Thread(target=self.main)
        mainThread.start()
        startDrone(self, hostname=self.HOSTNAME)
