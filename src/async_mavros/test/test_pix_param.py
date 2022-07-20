import threading
import unittest
from async_mavros.pix_param import MAVParmDict 
import os

this_file_path = os.path.realpath(__file__)
param_file_paths = os.path.join(os.path.dirname(this_file_path), "param_files")

DO_NOTHING = 0 
UPDATE_IMMEDIATE = 1
UPDATE_WITH_LATENCY = 2 

def dict_values_to_float(plain_dict: dict): 
    for k, v in plain_dict.items(): 
        plain_dict[k] = float(v)

class ParamMessageMock():
    def __init__(self, param_id, param_value) -> None:
        self.param_id  = param_id
        self.param_value = param_value
        
class MavMock(): 
    def __init__(self, test_pix_param: MAVParmDict, mode=DO_NOTHING, **kwargs) -> None:
        self.test_pix_param:MAVParmDict = test_pix_param
        self.mode = mode 
        self.latency = kwargs.get('latency') if 'latency' in kwargs else 0.05 

    def param_set_send(self, name, value, parm_type=None): 
        if self.mode == DO_NOTHING: 
            return self.do_nothing_param_set_send(name, value, parm_type)
        elif self.mode == UPDATE_IMMEDIATE: 
            self.update_pix_param_param_set_send(name, value, parm_type)
        elif self.mode == UPDATE_WITH_LATENCY: 
            self.update_pix_param_in_another_thread(name, value)

    def update_pix_param_param_set_send(self,name, value, parm_type=None): 
        if self.test_pix_param is not None: 
            self.test_pix_param.on_recieved_parameter(ParamMessageMock(name, value))

    def update_pix_param_in_another_thread(self,name, value, parm_type=None): 
        if self.test_pix_param is not None: 
            threading.Timer(self.latency, self.test_pix_param.on_recieved_parameter, (ParamMessageMock(name, value), )).start()

    def do_nothing_param_set_send(self,name, value, parm_type=None): 
        pass
    
PARAM_FILE_NAME = "param_file1.parm"
class PixParamTest(unittest.TestCase): 

    def setUp(self) -> None:
        self.test_file_name = os.path.join(param_file_paths, PARAM_FILE_NAME)
        self.test_pix_param = MAVParmDict()

    def test_load_file_to_no_mav(self,): 
        self.test_pix_param.clear()
        result, description = self.test_pix_param.load(self.test_file_name, mav=MavMock(None), check=False)
        expected_pending_params = {}
        expected_load_result = False
        expected_pix_param = {}
        self.assertEqual(self.test_pix_param.data, expected_pix_param)
        self.assertEqual(self.test_pix_param.pending_parameters, expected_pending_params)
        self.assertEqual(result, expected_load_result)

    def test_load_file_to_simple_mav(self,): 
        self.test_pix_param.clear()
        result, description = self.test_pix_param.load(self.test_file_name, mav=MavMock(self.test_pix_param, mode=UPDATE_IMMEDIATE), check=False)
        expected_pending_params = {}
        expected_load_result = True
        expected_pix_param = self.test_pix_param.parse_params_file(self.test_file_name, wildcard="*", use_excludes=True)
        dict_values_to_float(expected_pix_param)
        self.assertEqual(self.test_pix_param.data, expected_pix_param)
        self.assertEqual(self.test_pix_param.pending_parameters, expected_pending_params)
        self.assertEqual(result, expected_load_result)

    def test_load_file_to_latency_mav(self,): 
        self.test_pix_param.clear()
        result, description = self.test_pix_param.load(self.test_file_name, mav=MavMock(self.test_pix_param, mode=UPDATE_WITH_LATENCY, latency=0.1), check=False)
        expected_pending_params = {}
        expected_load_result = True
        expected_pix_param = self.test_pix_param.parse_params_file(self.test_file_name, wildcard="*", use_excludes=True)
        dict_values_to_float(expected_pix_param)
        self.assertEqual(self.test_pix_param.data, expected_pix_param)
        self.assertEqual(self.test_pix_param.pending_parameters, expected_pending_params)
        self.assertEqual(result, expected_load_result)

    def test_load_file_to_high_latency_mav(self,): 
        self.test_pix_param.clear()
        result, description = self.test_pix_param.load(self.test_file_name, mav=MavMock(self.test_pix_param, mode=UPDATE_WITH_LATENCY, latency=1), check=False)
        expected_pending_params = {}
        expected_load_result = False 
        self.assertEqual(self.test_pix_param.pending_parameters, expected_pending_params)
        self.assertEqual(result, expected_load_result)

if __name__ == '__main__':
    unittest.main()