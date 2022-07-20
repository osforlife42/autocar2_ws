from typing import Optional, Awaitable
import asyncio
from asyncio import Task
import threading
from concurrent.futures import Future as ConcurrentFuture
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, await_or_execute
from rclpy.callback_groups import ReentrantCallbackGroup
import time 

from example_interfaces.srv import Trigger 
class IncrementPercentDoneServiceNode(Node):


    def __create_task(self, f: Awaitable, ):
        # my_task = self.__loop.run_in_executor(None, f)
        # my_task = self.__loop.call_soon_threadsafe(self.__loop.create_task, f)
        my_task = asyncio.run_coroutine_threadsafe(f, self.__loop)
        return my_task

    def __init__(self):

        super().__init__('increment_percent_done_service_node')

        self.__loop = asyncio.new_event_loop()
        self.__task: Optional[Task] = None
        self.__thread = threading.Thread(target=self.__loop.run_forever)
        self.__thread.start()
        self.reentrant_cb_group = ReentrantCallbackGroup()
        
        self.done = False

        self.create_service(Trigger, 'start_incrementing', self.incrementing_cb, callback_group=self.reentrant_cb_group)
        self.__event_gate = threading.Event()
        self.__event_gate.clear()
        # self.done_future = Future(self.executor)
        # self.done_future.add_done_callback(lambda x: print(f"after done_future {x.done()}"))

    async def incrementing_cb(self, request, response): 
        self.get_logger().info("Starting service"),
        # self.__loop.call_soon_threadsafe(self.__create_task, self.__increment_percent_complete())
        sync_event = threading.Event()
        # finished_future = ConcurrentFuture()
        future_result = self.__create_task(self.__increment_percent_complete(sync_event) )
        sync_event.wait()
        response.success = True
        response.message=f"time it took {future_result.result()}"
        
        return response

    def __del__(self):
        print("stopping loop")
        self.done = True
        if self.__task is not None:
            self.__task.cancel()

        self.__loop.stop()
        self.__thread.join()

    async def __increment_percent_complete(self, sync_event: threading.Event):
        timeout_start = time.time()        
        duration = 5

        while time.time() < (timeout_start + duration):
            time_since_start = time.time() - timeout_start
            percent_complete = (time_since_start / duration) * 100.0
            self.get_logger().info("Percent complete: {}%".format(percent_complete))
            await asyncio.sleep(0.5)

        self.get_logger().info("leaving async function")

        self.done = True
        if sync_event: 
            sync_event.set()
        return time.time() - timeout_start

if __name__ == '__main__':

    rclpy.init()
    test = IncrementPercentDoneServiceNode()
    e = MultiThreadedExecutor()
    e.add_node(test)
    e.spin()