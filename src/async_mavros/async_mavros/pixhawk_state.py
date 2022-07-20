import logging
from sre_constants import ANY

logging.basicConfig(format="[%(levelname)s] %(asctime)s %(message)s", level=logging.DEBUG)
logger = logging.getLogger(__name__)
from typing import Callable, Dict, List, Set, Any
from collections import UserDict

class NotifyDict(): 

    def __init__(self,):
        self.data: Dict[str, Set[Callable]] = dict()

    def register_callback(self, message_name: str, cb: Callable): 
        self.data.setdefault(message_name, set()).add(cb) 

    def unregister_callback(self, cb: Callable): 
        for callback_list in self.data.values(): 
            callback_list.remove(cb)

    def clear(self,): 
        self.data.clear()

    @property
    def callback_dict(self,) -> Dict[str, Set[Callable]]: 
        return self.data

    def notify_message_listeners(self, message_name: str, value: Any):
        """
        Triggers all callbacks for a given message.
        Goes through list of registered listeners (callback functions) for the
        MsgID and calls each function sequentially.
        Args:
            name: `MsgID` name of the message
            msg: message data to pass to each of the listeners
        """
        for fn in self.data.get(message_name, set()):
            try:
                fn(message_name, value)
            except Exception as e:
                logger.error("notify error", exc_info=True)

    def clear_message_listening(self, message_name: str): 
        self.data.pop(message_name) 

class PixState(UserDict):
    '''
    a dictionary containing the last pixhawk state and updates itself based on registration to callbacks to mavlink messages
    '''

    def __init__(self,):
        super().__init__()
        self.mav_observers: NotifyDict = NotifyDict()
        self.updates_observers: NotifyDict = NotifyDict()


    def __setitem__(self, key: str, item: Any) -> None:
        self.updates_observers.notify_message_listeners(key, item)
        return super().__setitem__(key, item)

    def clear(self,): 
        super().clear()
        self.mav_observers.clear()
        self.updates_observers.clear()


    def __repr__(self) -> str:
        return f"values: {super().__repr__()} \n callbacks: {self.callback_dict}"