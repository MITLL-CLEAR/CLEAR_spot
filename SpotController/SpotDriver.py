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
This script functions as the main driver of spot. 
"""

import curses
import logging
import os
import signal
import threading
import time
from collections import OrderedDict
from SpotController.CameraHandler.CameraHandler import AsyncImageCapture as Cam
from SpotController.ArmHandler.ArmHandler import ArmController
import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import geometry_pb2
from bosdyn.client import ResponseError, RpcError, create_standard_sdk, robot_command
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.spot_cam.lighting import LightingClient

from bosdyn.util import duration_str, format_metric, secs_to_hms

from bosdyn.client.docking import DockingClient, blocking_dock_robot, blocking_undock, get_dock_id

from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

from bosdyn.geometry import EulerZXY


LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = .35     # m/s
VELOCITY_BASE_ANGULAR = 0.35  # rad/sec
VELOCITY_ROTATION = 0.1  # rad/sec
VELOCITY_PITCH_ROTATION = 0.02  # rad/sec


VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1

HEIGHT_MAX = 0.3  # m
HEIGHT_MIN = -0.3  # m

ROLL_OFFSET_MAX = 0.4  # rad

YAW_OFFSET_MAX = 2  # rad
YAW_OFFSET_MIN = -2 # rad


# PITCH_OFFSET_MAX = 0.7805  # rad
# PITCH_OFFSET_MIN = -0.7805  # rad

PITCH_OFFSET_MAX = 2  # rad
PITCH_OFFSET_MIN = -2  # rad

HEIGHT_CHANGE = 0.1  # m per command

CLEAR_YAW_PERCENT = 0.75
CLEAR_VEL_PERCENT = 0.75

SIZE_OF_ARR = 10

class Options:
    def __init__(self, hostname, verbose=False, time_sync_interval_sec=1.0):
        self.hostname = hostname
        self.verbose = verbose
        self.time_sync_interval_sec = time_sync_interval_sec

class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    def request_exit(self):
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self):
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now


class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""

    def __init__(self, wasd_interface):
        super(CursesHandler, self).__init__()
        self._wasd_interface = wasd_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._wasd_interface.add_message(f'{record.levelname:s} {msg:s}')


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__('robot_state', robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class WasdInterface(object):
    """A curses interface for driving the robot."""

    def __init__(self, robot, controller):
        self._robot = robot
        self.controller = controller
        self.hasBeenSet = False

        #Assuming starts from the ground
        self.standing = False

        self.commandLock = threading.Lock()  # create a lock object
        self.autoDroneLock = threading.Lock()  # create a lock object

        self.body_height = 0.0
        self.stand_yaw = 0.0
        self.stand_roll = 0.0
        self.stand_pitch = 0.0

        self.stand_height_change = False
        self.stand_roll_change = False
        self.stand_pitch_change = False
        self.stand_yaw_change = False

        self.readyForCommand = True

        self.readyToGrab = False

        self.listOfYaw = []

        self.yawIndex = 0

        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)

        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
            print("estop aquired")
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._image_task = Cam(self, robot, controller)
        self._async_tasks = AsyncTasks([self._robot_state_task, self._image_task])
        self._lock = threading.Lock()
        self.arm = ArmController(self, controller)

        self._command_dictionary = {
            27: self._stop,  # ESC key
            ord('\t'): self._quit_program,
            ord('T'): self._toggle_time_sync,
            ord(' '): self._toggle_estop,
            ord('r'): self._self_right,
            ord('P'): self._toggle_power,
            ord('p'): self._toggle_power,
            ord('v'): self._sit,
            ord('b'): self._battery_change_pose,
            ord('f'): self._stand,
            ord('w'): self._move_forward,
            ord('s'): self._move_backward,
            ord('a'): self._strafe_left,
            ord('d'): self._strafe_right,
            ord('q'): self._turn_left,
            ord('e'): self._turn_right,
            ord('I'): self._image_task.take_image,
            ord('O'): self._image_task.toggle_video_mode,
            ord('u'): self._unstow,
            ord('j'): self._stow,
            ord('l'): self._toggle_lease, 
            ord('x'): self._dock,
            ord('9'): self.arm.arm_object_grasp,
            ord('0'): self._orientation_cmd_helper,
            ord('1'): self.arm.stopHold
        }
        self._locked_messages = ['', '', '']  # string: displayed message for user
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease_keepalive = None

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_client.take()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)

        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info('Shutting down WasdInterface.')
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def flush_and_estop_buffer(self, stdscr):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = stdscr.getch()
            if key == ord(' '):
                self._toggle_estop()

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def drive(self, stdscr):
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            curses_handler = CursesHandler(self)
            curses_handler.setLevel(logging.INFO)
            LOGGER.addHandler(curses_handler)

            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(26, 140)
            stdscr.refresh()

            # for debug
            curses.echo()
            
            try:
                while not self._exit_check.kill_now:
                    self._async_tasks.update()
                    self._drive_draw(stdscr, self._lease_keepalive)

                    try:
                        cmd = stdscr.getch()
                        # Do not queue up commands on client
                        self.flush_and_estop_buffer(stdscr)
                        self._drive_cmd(cmd)
                        time.sleep(COMMAND_INPUT_RATE)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                LOGGER.removeHandler(curses_handler)

    def getState(self) :
        data = {}

        leaseState = {"Lease" : self._lease_str(self._lease_keepalive)}
        data.update(leaseState)

        battery = {"Battery" : self._battery_str()}
        data.update(battery)

        estop = {"Estop" : self._estop_str()}
        data.update(estop)

        powerState = {"Power State" : self._power_state_str()}
        data.update(powerState)

        timeSync = {"Time Sync" : self._time_sync_str()}
        data.update(timeSync)

        return data

    def _drive_draw(self, stdscr, lease_keep_alive):
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 140)
        stdscr.addstr(0, 0, f'{self._robot_id.nickname:20s} {self._robot_id.serial_number}')
        stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._estop_str())
        stdscr.addstr(4, 0, self._power_state_str())
        stdscr.addstr(5, 0, self._time_sync_str())
        for i in range(3):
            stdscr.addstr(7 + i, 2, self.message(i))
        stdscr.addstr(10, 0, 'Commands: [TAB]: quit                               ')
        stdscr.addstr(11, 0, '          [T]: Time-sync, [SPACE]: Estop, [P]: Power')
        stdscr.addstr(12, 0, '          [I]: Take image, [O]: Video mode          ')
        stdscr.addstr(13, 0, '          [f]: Stand, [r]: Self-right               ')
        stdscr.addstr(14, 0, '          [v]: Sit, [b]: Battery-change             ')
        stdscr.addstr(15, 0, '          [wasd]: Directional strafing              ')
        stdscr.addstr(16, 0, '          [qe]: Turning, [ESC]: Stop                ')
        stdscr.addstr(17, 0, '          [l]: Return/Acquire lease                 ')
        stdscr.addstr(18, 0, '')
        stdscr.refresh()

    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message(f'Unrecognized keyboard command: \'{chr(key)}\'')

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message(f'Failed {desc}: {err}')
            return None

    def _try_grpc_async(self, desc, thunk):

        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message(f'Failed {desc}: {err}')
                return None

        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_time_sync(self):
        if self._robot.time_sync.stopped:
            self._robot.start_time_sync()
        else:
            self._robot.time_sync.stop()

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc('stopping estop', self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                    return_at_exit=True)
            else:
                if not self._lease_client.lease_wallet.has_valid_lease():
                    self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                        return_at_exit=True)
                else:
                    self._lease_keepalive.shutdown()
                    self._lease_keepalive = None


    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            #just changed, was just robot_command before
            self._robot_command_client.robot_command_async(command=command_proto,
                                                   end_time_secs=end_time_secs)

        if not self.commandLock.acquire(blocking=False): return

        try :
            self._try_grpc(desc, _start_command)

        except Exception as e :
            self.add_message("Exception with command : ", e)

        finally :
            self.commandLock.release()


    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _battery_change_pose(self):
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_robot_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

    def _dock(self) :
        self.autoDroneLock.acquire(blocking=True)

        self.standing = False
        robot_command.blocking_stand(self._robot_command_client)
        blocking_dock_robot(self._robot, 520)
        print('Docking Success')

        self.autoDroneLock.release()

    def _sit(self):
        self.readyForCommand = False
        if self._robot.has_arm() :
            self.arm.stopHold()
            self.arm.stowArm()

        self.standing = False
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())
        self.readyForCommand = True

    def _stand(self):
        self.standing = True
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    # _move(self, left_x, left_y, right_x):
    def _move_forward(self):
        self._move(left_x = 0, left_y = VELOCITY_BASE_SPEED*2, right_x=0)

    def _move_backward(self):
        self._move(left_x = 0, left_y = -VELOCITY_BASE_SPEED*2, right_x=0)

    def _strafe_left(self):
        self._move(left_x = -VELOCITY_BASE_SPEED*2)

    def _strafe_right(self):
        self._move(left_x = VELOCITY_BASE_SPEED*2)

    def _turn_left(self):
        self._move( right_x= -VELOCITY_BASE_ANGULAR)

    def _turn_right(self):
        self._move(right_x = VELOCITY_BASE_ANGULAR)

    def _stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)


    def _orientation_cmd_helper(self, yaw=0.0, roll=0.0, pitch=0):
        """Helper function that commands the robot with an orientation command;
        Used by the other orientation functions.

        Args:
            yaw: Yaw of the robot body. Defaults to 0.0.
            roll: Roll of the robot body. Defaults to 0.0.
            pitch: Pitch of the robot body. Defaults to 0.0.
            height: Height of the robot body from normal stand height. Defaults to 0.0.
        """
        desc = ""

        self.stand_pitch = max(self.stand_pitch + min(VELOCITY_PITCH_ROTATION * pitch, PITCH_OFFSET_MAX), PITCH_OFFSET_MIN)

        self.stand_yaw = -max(self.stand_yaw + min(VELOCITY_ROTATION * yaw, YAW_OFFSET_MAX), YAW_OFFSET_MIN)

        # self.stand_pitch = min(self.stand_pitch + max(pitch * VELOCITY_BASE_ANGULAR, PITCH_OFFSET_MIN), PITCH_OFFSET_MAX)

        orientation = EulerZXY(self.stand_yaw, roll, self.stand_pitch)

        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_stand_command(footprint_R_body=orientation, body_height = self.body_height),
                end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def getYawAvg(self):
        value = 0
        for i in self.listOfYaw:
            value += i
        return value/SIZE_OF_ARR

    # Since being called by a seperate but concurrent thread, 
    # there is weirdness. Without the lock and sleep mechanism _autoMove
    # was calling _move in excess and overwhelming spot with requests. 
    #It seems that the pause implemented in start_robot_command does not effect
    #the interface thread which calls autoMove. Thus a pause is neccesary for calling
    #autoMove to prevent spot from being overwhelmed. 
    def _autoMove(self, left_x, left_y, right_x, height, resetRotation = False) :
        if not self.autoDroneLock.acquire(blocking=False): return

        try :
            if hasattr(self, "standing") :
                if not self.standing :
                    self._stand()
            
            initTime = time.time()

            if len(self.listOfYaw) <= SIZE_OF_ARR:
                self.listOfYaw.append(abs(right_x))
            else:
                self.listOfYaw[self.yawIndex%SIZE_OF_ARR] = abs(right_x)
                self.yawIndex += 1 
            
            if resetRotation:
                self.stand_pitch = 0
                self.stand_yaw = 0

            TOLERANCE = 0.45

            if left_x == 0 and left_y == 0 and self.getYawAvg() <= TOLERANCE:
                self._orientation_cmd_helper(pitch=height, yaw = right_x)
            else:
                self._move(left_x * CLEAR_VEL_PERCENT, left_y * CLEAR_VEL_PERCENT, right_x * CLEAR_YAW_PERCENT, height)

            waitTime = max(VELOCITY_CMD_DURATION + (-time.time()+initTime), 0)
            time.sleep(waitTime)

        except Exception as e :
            LOGGER.error('AutoMove Error : ', e)

        finally :
            self.autoDroneLock.release()


    def _move(self, left_x = 0, left_y = 0, right_x = 0, heightChange = 0):

        try :
            self.readyForCommand = False
            desc = ""
            # Stick left_x controls robot v_y 
            v_y = -left_x * VELOCITY_BASE_SPEED

            # Stick left_y controls robot v_x
            v_x = left_y * VELOCITY_BASE_SPEED

            # Stick right_x controls robot v_rot
            v_rot = -right_x * VELOCITY_BASE_ANGULAR

            self.stand_pitch = 0
            self.stand_yaw = 0

            self.body_height = max(self.body_height + min(heightChange * HEIGHT_CHANGE, HEIGHT_MAX), HEIGHT_MIN)

            self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot, body_height=self.body_height),
                end_time_secs=time.time() + VELOCITY_CMD_DURATION)
            
            # self._orientation_cmd_helper()

            self.readyForCommand = True
        except Exception as e :
            LOGGER.error("Movement exception : ", e)


    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)

    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async('powering-on', self._request_power_on)
        else:
            self._try_grpc('powering-off', self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            try:
                _lease = lease_keep_alive.lease_wallet.get_lease()
                lease = f'{_lease.lease_proto.resource}:{_lease.lease_proto.sequence}'
            except bosdyn.client.lease.Error:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return f'Lease {lease} THREAD:{alive}'

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return f'Power: {state_str[6:]}' 

    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        return f'Estop {estop_status} (thread: {thread_status})'

    def _time_sync_str(self):
        if not self._robot.time_sync:
            return 'Time sync: (none)'
        if self._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self._robot.time_sync.thread_exception
            if exception:
                status = f'{status} Exception: {exception}'
        else:
            status = 'RUNNING'
        try:
            skew = self._robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = f'offset={duration_str(skew)}'
            else:
                skew_str = '(Skew undetermined)'
        except (TimeSyncError, RpcError) as err:
            skew_str = f'({err})'
        return f'Time sync: {status} {skew_str}'

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = f'|{"=" * bar_len}{" " * (10 - bar_len)}|'
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = f'({secs_to_hms(battery_state.estimated_runtime.seconds)})'
        return f'Battery: {status}{bat_bar} {time_left}'
    

    def _change_height(self, direction = 1):
        """Changes robot body height.

        Args:
            direction: 1 to increase height, -1 to decrease height.
        """

        self.body_height = self.body_height + direction * HEIGHT_CHANGE
        self.body_height = min(HEIGHT_MAX, self.body_height)
        self.body_height = max(-HEIGHT_MAX, self.body_height)
        self._orientation_cmd_helper(height=self.body_height)


def _setup_logging(verbose):
    """Log to file at debug level, and log to console at INFO or DEBUG (if verbose).

    Returns the stream/console logger so that it can be removed when in curses mode.
    """
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Save log messages to file wasd.log for later debugging.
    file_handler = logging.FileHandler('wasd.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)

    # The stream handler is useful before and after the application is in curses-mode.
    if verbose:
        stream_level = logging.DEBUG
    else:
        stream_level = logging.INFO

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(stream_level)
    stream_handler.setFormatter(log_formatter)
    LOGGER.addHandler(stream_handler)
    return stream_handler

def main(BigDog, hostname):
    """Command-line interface."""
    # Replace argparse with a simple class
    try :
        options = Options(hostname)

        stream_handler = _setup_logging(options.verbose)

        # Create robot object.
        sdk = create_standard_sdk('WASDClient')
        robot = sdk.create_robot(options.hostname)
        try:
            bosdyn.client.util.authenticate(robot)
            robot.start_time_sync(options.time_sync_interval_sec)
        except RpcError as err:
            LOGGER.error('Failed to communicate with robot: %s', err)
            return False
        
        BigDog.controller = WasdInterface(robot,BigDog)
        
        try:
            BigDog.controller.start()
            BigDog.controller._toggle_estop()
            time.sleep(0.5)
            BigDog.controller._try_grpc_async('powering-on',
              BigDog.controller._request_power_on)
            time.sleep(5)

            if BigDog.INTERMEDIARY :

                BigDog.readyToConnect = True

        except (ResponseError, RpcError) as err:
            LOGGER.error('Failed to initialize robot communication: %s', err)
            return False

        LOGGER.removeHandler(stream_handler)  # Don't use stream handler in curses mode.

        try:
            try:
                # Prevent curses from introducing a 1 second delay for ESC key
                os.environ.setdefault('ESCDELAY', '0')
                # Run wasd interface in curses mode, then restore terminal config.
                curses.wrapper(BigDog.controller.drive)
            finally:
                # Restore stream handler to show any exceptions or final messages.
                LOGGER.addHandler(stream_handler)
        except Exception as e:
            LOGGER.error('WASD has thrown an error: [%r] %s', e, e)
    except Exception as e:
        print("error with : ", e)

    finally:
        # Do any final cleanup steps.
        BigDog.controller.shutdown()
        BigDog.shutdown = True
        return True


if __name__ == '__main__':
    if not main():
        os._exit(1)
    os._exit(0)
