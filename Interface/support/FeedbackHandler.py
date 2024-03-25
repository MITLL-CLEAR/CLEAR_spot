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
This class platforms communication from drone back to the controller
"""
class FeedbackHandler() :
    def __init__(self, drone) -> None:
        self.drone = drone
        self.PostURL =  "{}/feedbackInfo".format(self.drone.URL)

    def giveFeedback(self, question) :
        try :
            if question == "state?" : 
                data = {"commandOccuring" : self.drone.performingCommand, "grabReady" : self.drone.controller.arm.readyToGrab}
                response = self.drone.session.post(self.PostURL, json={"feedback":data},verify=False)
                        
        except Exception as e : 
            with open("feedbackError.txt", "w") as file:
                    file.write("The error is : {}".format(e))
    
    def giveComment(self, comment):
        data = {"comment" : comment}
        response = self.drone.session.post(self.PostURL, json={"feedback":data},verify=False)




    
