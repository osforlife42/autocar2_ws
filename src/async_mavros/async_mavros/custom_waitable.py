
from rclpy.executors import Waitable, NumberOfEntities
from rclpy.node import Node 
from rclpy.callback_groups import CallbackGroup
import threading 

class OnceWaitable(Waitable): 
    def __init__(self, callback, node: Node, callback_group: CallbackGroup):
        if callback_group is None:
            callback_group = node.default_callback_group
        super().__init__(callback_group)
        self._lock = threading.Lock() 
        self.callback = callback
        self._node = node 
        self._running = False
        self._handle = self._node.handle
        callback_group.add_entity(self)
        self._node.add_waitable(self)

    # Start Waitable API
    def is_ready(self, wait_set):
        """Return True if one or more entities are ready in the wait set."""
        return not self._running

    def take_data(self):
        """Take stuff from lower level so the wait set doesn't immediately wake again."""
        data = {}
        return data

    async def execute(self, taken_data):
        """
        Execute work after data has been taken from a ready wait set.

        This will set results for Future objects for any received service responses and
        call any user-defined callbacks (e.g. feedback).
        """
        await self.callback

    def get_num_entities(self):
        """Return number of each type of entity used in the wait set."""
        # num_entities = self._handle.get_num_entities()
        return NumberOfEntities(0,0,0,0,0,1)

    def add_to_wait_set(self, wait_set):
        """Add entities to wait set."""
        with self._lock:
            wait_set.add(self)
    # End Waitable API