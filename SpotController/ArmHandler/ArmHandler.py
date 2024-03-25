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

import time
import cv2
import numpy as np

import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import estop_pb2, geometry_pb2, image_pb2, manipulation_api_pb2
from bosdyn.client.estop import EstopClient
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient

from bosdyn.client.gripper_camera_param import GripperCameraParamClient

from bosdyn.api import arm_command_pb2, manipulation_api_pb2, robot_state_pb2

from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)

class ArmController () :
    #going to pass wasd as robot
    def __init__(self, elder, controller) :
        self.elder = elder
        self.controller = controller
        self.manipulation_api_client = self.elder._robot.ensure_client(ManipulationApiClient.default_service_name)
        self.wait = False
        # Relates to camera
        self.readyToGrab = False
        self.grabInProcess = False
        self.gripper_camera_param_client = self.elder._robot.ensure_client(GripperCameraParamClient.default_service_name)
        self.image_client = self.elder._robot.ensure_client(ImageClient.default_service_name)


    def get_hand_color_image(self):
        image_responses = self.image_client.get_image_from_sources(['hand_color_image'])
        if len(image_responses) != 1:
            print('Error: did not get exactly one image response.')
            sys.exit(1)

        resp = image_responses[0]

        self.controller.images = image_responses
        # Display the image to the user
        image = image_responses[0]
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(image.shot.image.data, dtype=dtype)
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        return img
    
    def arm_object_grasp(self, point = None):
        self.wait = True
        self.imageClick = None
        self.imageDisplay = None
        
        # self.elder.autoDroneLock.acquire(blocking=True)
        # self.elder.commandLock.acquire(blocking=True)


        self.elder._robot.time_sync.wait_for_sync()
        assert self.elder._robot.has_arm(), 'Robot requires an arm to run this example.'

        if not self.readyToGrab: 
            return False
            
        combined_image = self.controller.grabimage

        # if self.controller.grabimage is not self.controller.image:
        #     xper = point[1]/self.controller.image.shape[0]
        #     yper = point[0]/self.controller.image.shape[1]

        #     xpos = int(xper*self.controller.grabimage.shape[0])
        #     ypos = int(yper*self.controller.grabimage.shape[1])
        #     point = (xpos, ypos)

        self.imageDisplay = combined_image
        if point == None :
            self.elder._robot.logger.info('Click on an object to start grasping...')
            image_title = 'Click to grasp'
            cv2.namedWindow(image_title)
            cv2.setMouseCallback(image_title, self.cv_mouse_callback)
            # I've modified this section to fetch the combined image from the controller
        
            while self.imageClick is None:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    print('"q" pressed, exiting.')
                    exit(0)

        else : self.translatingPointToSource(point)

        # Here we figure out which image the point belongs to and its corresponding point in that image
        clicked_image, clicked_point = self.get_image_and_point()
        pick_vec = geometry_pb2.Vec2(x=clicked_point[0], y=clicked_point[1])

        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=clicked_image.shot.transforms_snapshot,
            frame_name_image_sensor=clicked_image.shot.frame_name_image_sensor,
            camera_model=clicked_image.source.pinhole)

        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        # Send the request
        cmd_response = self.manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        completedAction = False
        initTime = time.time()
        REQUEST_EXPIRE = 10

        # Get feedback from the robot
        while not completedAction and not (time.time() > REQUEST_EXPIRE + initTime):
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = self.manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            print(
                f'Current state: {manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state)}'
            )
            
            # To my current understanding, this is used to close the hand
            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                self.manipulation_client = self.elder._robot.ensure_client(ManipulationApiClient.default_service_name)


                # CarryStateOverride requests for the three possible carry states. See manipulation_api.proto
                # for more details.
                not_carriable_override = manipulation_api_pb2.ApiGraspedCarryStateOverride(
                    override_request=robot_state_pb2.ManipulatorState.CARRY_STATE_NOT_CARRIABLE)
                carriable_override = manipulation_api_pb2.ApiGraspedCarryStateOverride(
                    override_request=robot_state_pb2.ManipulatorState.CARRY_STATE_CARRIABLE)
                carriable_and_stowable_override = manipulation_api_pb2.ApiGraspedCarryStateOverride(
                    override_request=robot_state_pb2.ManipulatorState.CARRY_STATE_CARRIABLE_AND_STOWABLE)

                # GraspOverride requests for the two possible grasp states. See manipulation_api.proto for
                # more details.
                grasp_holding_override = manipulation_api_pb2.ApiGraspOverride(
                    override_request=manipulation_api_pb2.ApiGraspOverride.OVERRIDE_HOLDING)
                grasp_not_holding_override = manipulation_api_pb2.ApiGraspOverride(
                    override_request=manipulation_api_pb2.ApiGraspOverride.OVERRIDE_NOT_HOLDING)

                override_request = manipulation_api_pb2.ApiGraspOverrideRequest(
                api_grasp_override=grasp_holding_override, carry_state_override=carriable_and_stowable_override)
                self.manipulation_client.grasp_override_command(override_request)
                # Wait for the override to take effect before trying to move the arm.
                self.wait_until_grasp_state_updates(override_request, self.elder._robot_state_client)

                self.stowArm()
        return True

    def stowArm(self) :
        robot_cmd = RobotCommandBuilder.arm_stow_command()
        self.elder._robot_command_client.robot_command(robot_cmd)
        return True

    
    def translatingPointToSource(self, position) :
        (x,y) = self.imageClick = position
        self.imageClick = (x, y)

        # Getting the original image width
        original_img_width = self.imageDisplay.shape[1] // 2
        # Checking on which image the user clicked
        if x <= original_img_width:
            self.image_clicked = self.controller.images[1]
            self.showImage = self.controller.showImages[1]
            height = self.showImage.shape[0]

                # User clicked on the second image
            
            self.point_clicked = (y, height - x) # Adjust x to be relative to the second image
            # center = (self.point_clicked[0], self.point_clicked[1])
            # cv2.circle(self.showImage, center, 10, (250,30,30))
            # cv2.imshow("yes", self.showImage)
            # cv2.waitKey(0)
        else:
            self.image_clicked = self.controller.images[0]
            self.showImage = self.controller.showImages[0]
            height = self.showImage.shape[0]
                # User clicked on the second image
            self.point_clicked = (y, height - int(x - (original_img_width*(1-self.controller.imageOverlap)))) # Adjust x to be relative to the second image
            # center = (self.point_clicked[0], self.point_clicked[1])
            # cv2.circle(self.showImage, center, 10, (30,30,30))
            # cv2.imshow("yes", self.showImage)
            # cv2.waitKey(0)


    def cv_mouse_callback(self, event, x, y, flags, param):
        clone = self.imageDisplay.copy()
        if event == cv2.EVENT_LBUTTONUP:
            self.translatingPointToSource((x,y))
        else:
            # Draw some lines on the image.
            # print('mouse', x, y)
            color = (30, 30, 30)
            thickness = 2
            image_title = 'Click to grasp'
            height = clone.shape[0]
            width = clone.shape[1]
            cv2.line(clone, (0, y), (width, y), color, thickness)
            cv2.line(clone, (x, 0), (x, height), color, thickness)
            cv2.imshow(image_title, clone)

    def stopHold(self) :
        robot_cmd = RobotCommandBuilder.claw_gripper_open_command()
        self.elder._robot_command_client.robot_command(robot_cmd)
        # robot.logger.info('Gripper opened.')
        

    def wait_until_grasp_state_updates(self, grasp_override_command, robot_state_client):
        updated = False
        has_grasp_override = grasp_override_command.HasField("api_grasp_override")
        has_carry_state_override = grasp_override_command.HasField("carry_state_override")

        initTime = time.time()
        REQUEST_EXPIRE = 5
        while not updated and not (time.time() > REQUEST_EXPIRE + initTime):
            robot_state = robot_state_client.get_robot_state()

            grasp_state_updated = (robot_state.manipulator_state.is_gripper_holding_item and
                                   (grasp_override_command.api_grasp_override.override_request
                                    == manipulation_api_pb2.ApiGraspOverride.OVERRIDE_HOLDING)) or (
                                        not robot_state.manipulator_state.is_gripper_holding_item and
                                        grasp_override_command.api_grasp_override.override_request
                                        == manipulation_api_pb2.ApiGraspOverride.OVERRIDE_NOT_HOLDING)
            carry_state_updated = has_carry_state_override and (
                robot_state.manipulator_state.carry_state
                == grasp_override_command.carry_state_override.override_request)
            updated = (not has_grasp_override or
                       grasp_state_updated) and (not has_carry_state_override or carry_state_updated)

    def get_image_and_point(self) :
        return self.image_clicked, self.point_clicked

if __name__ == '__main__':
    arm = ArmController(0, 0, 0)
    arm.arm_object_grasp()
