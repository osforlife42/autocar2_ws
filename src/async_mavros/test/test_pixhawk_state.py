
import unittest
from async_mavros.pixhawk_state import PixState
import os


class GeneralUtilsTest(unittest.TestCase): 


    def callback_name(self, msg_name, msg_value): 
        self.test_pix_state[msg_name] = msg_value

    def notify_name_changes_callback(self, msg_name, value): 
        if self.test_pix_state.get(msg_name) != value: 
            self.name_changes_count += 1

    def setUp(self) -> None:
        self.name_changes_count = 0
        self.test_pix_state = PixState()
    
    def test_add_callback(self,): 
        self.test_pix_state.clear()
        self.test_pix_state.mav_observers.register_callback("name_callback", self.callback_name)
        expected_pix_state_callback_dict = {"name_callback": set((self.callback_name,))}
        self.assertEqual(self.test_pix_state.mav_observers.callback_dict, expected_pix_state_callback_dict)
    
    def test_add_and_notify_cb(self,): 
        self.test_pix_state.clear()
        self.test_pix_state.mav_observers.register_callback("name_callback", self.callback_name)
        expected_pix_state_callback_dict = {"name_callback": set((self.callback_name,))}
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        expected_dict_values = {"name_callback": "my_name"}
        self.assertEqual(self.test_pix_state.data, expected_dict_values)
        self.assertEqual(self.test_pix_state.mav_observers.callback_dict, expected_pix_state_callback_dict)

    def test_notify_and_count_changes(self,): 
        self.test_pix_state.clear()
        # register changes callbacks 
        self.test_pix_state.mav_observers.register_callback("name_callback", self.callback_name)
        self.test_pix_state.updates_observers.register_callback("name_callback", self.notify_name_changes_callback)
        # make changes and count them using the notify name change callback  
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "new_name")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "new_name")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "another_name")
        self.assertEqual(self.name_changes_count, 3)
        # remove the notify callback and make share changes it is actually unregistered 
        self.test_pix_state.updates_observers.unregister_callback(self.notify_name_changes_callback)
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "new_name")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        self.assertEqual(self.name_changes_count, 3)
        
    def test_notify_after_remove_cb(self,):
        self.test_pix_state.clear()
        self.test_pix_state.mav_observers.register_callback("name_callback", self.callback_name)
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        expected_dict_values = {"name_callback": "my_name"}
        self.test_pix_state.mav_observers.clear_message_listening("name_callback")
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "new_name")

        expected_pix_state_callback_dict = dict() 
        self.assertEqual(self.test_pix_state.data, expected_dict_values)
        self.assertEqual(self.test_pix_state.mav_observers.callback_dict, expected_pix_state_callback_dict)

    def test_remove_callback(self,): 
        self.test_pix_state.clear()
        self.test_pix_state.mav_observers.register_callback("name_callback", self.callback_name)
        self.test_pix_state.mav_observers.register_callback("another_callback", self.callback_name)
        self.test_pix_state.mav_observers.notify_message_listeners("name_callback", "my_name")
        self.test_pix_state.mav_observers.unregister_callback(self.callback_name)
        expected_pix_state_callback_dict = {"name_callback": set(), "another_callback": set()}
        self.assertEqual(self.test_pix_state.mav_observers.callback_dict, expected_pix_state_callback_dict)

if __name__ == '__main__':
    unittest.main()
