import os 
from datetime import datetime
import time
import asyncio
from typing import List
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega
import threading
import logging

from async_mavros.pixhawk_state import PixState
from async_mavros.pix_param import MAVParmDict

logger = logging.getLogger()
logging.basicConfig()


CS = "127.0.0.1:14551"

async def print_async(print_timeout=0.5): 
    print('here ')
    await asyncio.sleep(print_timeout)
    print(f'afer {print_timeout} secs')

class AsyncMavConnection():

    def __init__(self,loop: asyncio.AbstractEventLoop = None):
        super().__init__()
        self.__loop = loop if loop is not None else asyncio.get_event_loop()
        print(self.__loop)
        self._master: mavutil.mavudp = None
        self.pix_state = PixState()
        self.pix_params = MAVParmDict()
        self.register_message_callbacks()
        self._read_handle = threading.Thread(target=self.dispatch_loop, name="mav_in", daemon=True)
        self._running = False

    def connect(self, cs: str ="127.0.0.1:14551" ) -> mavutil.mavudp:
        if not cs: 
            return None
        while True:
            try:
                self._master: mavutil.mavudp = mavutil.mavlink_connection(cs)
                heartbeat = self._master.wait_heartbeat(timeout=3)
                if heartbeat: 
                    self.__start()
                    return self._master
                else: 
                    logger.exception(
                        f"Failed connecting to pix at {cs}, retrying... ")
            except:
                logger.exception(
                    f"Failed connecting to pix at {cs}, retrying... ", exc_info=True)
                time.sleep(0.5)
        
    async def arm_disarm(self, to_arm: bool = True, timeout: float = 3, *, sync_event: threading.Event = None) -> bool: 
        arm_check_future = asyncio.get_running_loop().create_future()
        
        def arm_check(_, vehicle_armed): 
            nonlocal arm_check_future
            action_success = to_arm == vehicle_armed
            print(f"action {'succeeded!' if action_success else 'failed:(' }")
            if not arm_check_future.cancelled() and not arm_check_future.done():
                self.__loop.call_soon_threadsafe(arm_check_future.set_result, action_success)
        
        try:
            self.send_long_command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, int(to_arm))
            self.pix_state.updates_observers.register_callback("armed", arm_check)
            action_result = await asyncio.wait_for(arm_check_future, timeout=timeout)
            print(f"future result is {action_result}")
        except asyncio.TimeoutError as timeout: 
            logger.warning(f"{'arm' if to_arm else 'disarm'} request timed out! ")
            return False
        except Exception as e: 
            logger.exception(e, exc_info=True)
            return False   
        finally: 
            self.pix_state.updates_observers.unregister_callback(arm_check)
            if sync_event: 
                print(f"setting event !")
                sync_event.set()
        return action_result 

    async def save_parameters(self, params_folder: str,  sync_event: threading.Event = None): 
        full_param_file_path = os.path.join(params_folder, datetime.now().strftime("%Y_%m_%d-%H_%M_%S"))
        self.pix_params.save_to_file(full_param_file_path)
        if sync_event: 
            sync_event.set()
        return True

    async def load_parameters_from_file(self, filename, sync_event: threading.Event = None): 
        success, description = self.pix_params.load(filename, mav=self._master)
        if sync_event: 
            sync_event.set()
        return success, description 

    async def change_mode(self, mode: str): 
        pass 

    #region: message management
    
    def register_message_callbacks(self,): 
        self.pix_state.mav_observers.register_callback("HEARTBEAT", self.heartbeat_cb)
        # self.pix_state.mav_observers.register_callback("COMMAND_ACK", self.print_command_ack)
        self.pix_state.mav_observers.register_callback("PARAM_VALUE", self.param_value_cb)
    
    def param_value_cb(self, name, msg): 
        self.pix_params.on_recieved_parameter(msg)

    def heartbeat_cb(self, name, msg): 
        self.pix_state[name] = msg 
        self.pix_state["armed"] = msg.base_mode & ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED == ardupilotmega.MAV_MODE_FLAG_SAFETY_ARMED

    def print_command_ack(self,name,msg): 
        logger.info(f"recieved command ack ----------")
        logger.info(msg)

    def dispatch_loop(self):
        """
        Wait for message from vehicle and send notification to register callbacks
        """
        while self._running:
            msg = self.__wait_for_message()
            if not msg:
                time.sleep(0.01)
                continue

            # logger.info(str(msg.to_dict()))
            self.pix_state.mav_observers.notify_message_listeners(msg.get_type(), msg)
    #endregion
    #region: control the connection

    def __start(self):
        self._running = True
        logger.info("Start vehicle")
        self._read_handle.start()

    def stop(self):
        self._running = False
        self._master.close()


    #endregion
    #region:  mavlink messaging 
    def send_long_command(self, command_type, param1, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
        """
        Packs and sends a Mavlink COMMAND_LONG message
        Args:
            command_type: the command type, as defined by MAV_CMD_*
            param1: param1 as defined by the specific command
            param2: param2 as defined by the specific command (default: {0})
            param3: param3 as defined by the specific command (default: {0})
            param4: param4 as defined by the specific command (default: {0})
            param5: param5 (x) as defined by the specific command (default: {0})
            param6: param6 (y) as defined by the specific command (default: {0})
            param7: param7 (z) as defined by the specific command (default: {0})
        """
        try:
            confirmation = 0  # may want this as an input.... used for repeat messages
            msg = self._master.mav.command_long_encode(
                self._master.target_system, 
                self._master.target_component,
                command_type,
                confirmation, 
                param1,
                param2,
                param3,
                param4,
                param5,
                param6,
                param7)

            self.send_message(msg)
        except:
            logger.error("Failed to send long command", exc_info=True)
    
  
    def __wait_for_message(self):
        """
        Wait for a new mavlink message calls pymavlink's blocking read function to read
        a next message, blocking for up to a timeout of 1s.
        Returns:
            Mavlink message that was read or `None` if the message was invalid.
        """

        # NOTE: this returns a mavlink message
        # this function should not be called outside of this class!
        msg = self._master.recv_match(blocking=True, timeout=1)
        if msg is None:
            # no message received
            return None
        else:
            if (msg.get_type() == 'BAD_DATA'):
                # no message that is useful
                return None

            # send a heartbeat message back, since this needs to be
            # constantly sent so the autopilot knows this exists
            if msg.get_type() == 'HEARTBEAT':
                # send -> type, autopilot, base mode, custom mode, system status
                outmsg = self._master.mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
                                                           mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                                           mavutil.mavlink.MAV_STATE_ACTIVE)
                self.send_message(outmsg)

            # pass the message along to be handled by this class
            return msg

    def send_message(self, msg):
        self._master.mav.send(msg)
    #endregion

async def main(): 

    async_mav_connection = AsyncMavConnection(asyncio.get_event_loop())

    async_mav_connection.connect()

    await asyncio.gather(print_async(), async_mav_connection.arm_disarm(True, 3))

if __name__ == "__main__": 
    asyncio.run(main())

    print('after everything ')