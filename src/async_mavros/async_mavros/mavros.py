import time
import asyncio
import rclpy
from typing import Awaitable, Optional 
import threading
from example_interfaces.srv import SetBool
from async_mavros.async_mav_connection import AsyncMavConnection
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node 

class AsyncMavRos(Node): 

    def __create_async_mav_task(self, async_mav_func: Awaitable): 
        my_task = asyncio.run_coroutine_threadsafe(async_mav_func, self.__loop)
        return my_task


    def __init__(self, **kwargs): 
        super().__init__('async_mav_ros', **kwargs)
        self.__loop = asyncio.new_event_loop()
        self.__thread = threading.Thread(target=self.__loop.run_forever)
        self.__thread.start()
        self._mav_connection: AsyncMavConnection = AsyncMavConnection(self.__loop)
        self._mav_connection.connect()
        self.done = False
        self._reentrant_cb_group = ReentrantCallbackGroup()
        self.create_service(SetBool, 'arming', self.arming_cb, callback_group=self._reentrant_cb_group)
        self.create_service(SetBool, 'pix_params/save_parameters', self.save_params_cb, callback_group=self._reentrant_cb_group)
        self.create_service(SetBool, 'pix_params/load_parameters', self.load_params_cb, callback_group=self._reentrant_cb_group)
        # armed_timer = self.create_timer(1, self.print_armed_state)
        

    def save_params_cb(self, request, response): 
        started_time = time.time()
        sync_event = threading.Event()
        action_success_fut = self.__create_async_mav_task(self._mav_connection.save_parameters(params_folder="/tmp/", sync_event=sync_event))
        sync_event.wait()
        response.success = action_success_fut.result()
        response.message=f"time it took {time.time() - started_time}"
        
        return response

    def load_params_cb(self, request, response): 
        full_filepath = "/home/user/projects/galactic_projects/dorothy2_ws/src/async_mavros/examples/param_path/test_param_file.parm"
        sync_event = threading.Event()
        action_success_fut = self.__create_async_mav_task(self._mav_connection.load_parameters_from_file(filename=full_filepath, sync_event=sync_event))
        sync_event.wait()
        response.success, response.message = action_success_fut.result()
        return response

    def arming_cb(self,request, response):
        started_time = time.time()
        sync_event = threading.Event()
        action_success_fut = self.__create_async_mav_task(self._mav_connection.arm_disarm(request.data,  sync_event=sync_event))
        sync_event.wait()
        response.success = action_success_fut.result()
        response.message=f"time it took {time.time() - started_time}"
        
        return response

    def print_armed_state(self, ): 
        self.get_logger().info(f"motors are {'armed' if self._mav_connection.pix_state.get('armed') else 'not armed '}")

    def __del__(self):
        print("stopping loop")
        self.done = True

        self.__loop.stop()
        self.__thread.join()

def main(**kwargs): 
    rclpy.init()
    test = AsyncMavRos()
    test.get_logger().warning(str(test.get_node_names_and_namespaces()))
    e = MultiThreadedExecutor()
    e.add_node(test)
    e.spin()

if __name__ == '__main__':
    main()
