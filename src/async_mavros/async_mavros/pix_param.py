from sys import exc_info
import threading
import time
from copy import deepcopy
from pymavlink import mavutil
import struct
from typing import Dict, Tuple
import fnmatch
import math
import logging
from collections import UserDict
import json

logger = logging.getLogger("pix parameter logger")
logging.basicConfig()

PARAM_RETRIES = 3


class MAVParmDict(UserDict):
    def __init__(self) -> None:
        super().__init__()
        self._pending_parameter_changes: Dict[str,
                                              Tuple[mavutil.mavfile, float, int]] = dict()
        self._params_failed: Dict[str, float] = dict()
        self.exclude_load = [
            'ARSPD_OFFSET',
            'CMD_INDEX',
            'CMD_TOTAL',
            'FENCE_TOTAL',
            'FORMAT_VERSION',
            'GND_ABS_PRESS',
            'GND_TEMP',
            'LOG_LASTFILE',
            'MIS_TOTAL',
            'SYSID_SW_MREV',
            'SYS_NUM_RESETS',
        ]
        self.mindelta = 0.000001
        self.pending_params_lock = threading.Lock()

    def clear(self,):
        self.data.clear()
        self._pending_parameter_changes.clear()
        self._params_failed.clear()

    @property
    def pending_parameters(self,):
        return self._pending_parameter_changes

    def on_recieved_parameter(self, param_msg):
        name = param_msg.param_id
        self.__setitem__(name, float(param_msg.param_value))
        with self.pending_params_lock:
            if str(name).upper() in self._pending_parameter_changes:
                self._pending_parameter_changes.pop(str(name).upper())

    def __mavset_pending_parameters(self, ) -> Dict[str, float]:
        while self._pending_parameter_changes:
            # run until no paramets are pending
            for param_name, (param_mav, param_value, param_retries) in self._pending_parameter_changes.copy().items():
                if param_retries <= 0:
                    self._params_failed[param_name] = param_value
                else:
                    self.mavset_one_parameter(
                        param_mav, param_name, param_value, param_retries)
                    # check if the parameter still exists in the original pending parameters and update it's retries
                    with self.pending_params_lock:
                        if self._pending_parameter_changes.get(param_name):
                            self._pending_parameter_changes[param_name] = (
                                param_mav, param_value, param_retries-1)
            for failed_param in self._params_failed:
                with self.pending_params_lock:
                    self._pending_parameter_changes.pop(failed_param)
        return self._params_failed

    def save_to_file(self,filename: str, wildcard='*', verbose=False): 
        '''save parameters to a file'''
        f = open(filename, mode='w')
        k = list(self.keys())
        k.sort()
        count = 0
        for p in k:
            if p and fnmatch.fnmatch(str(p).upper(), wildcard.upper()):
                value = self.__getitem__(p)
                if isinstance(value, float):
                    f.write("%-16.16s %f\n" % (p, value))
                else:
                    f.write("%-16.16s %s\n" % (p, str(value)))
                count += 1
        f.close()
        if verbose:
            print("Saved %u parameters to %s" % (count, filename))

    def mavset_one_parameter(self, mav: mavutil.mavfile, name: str, value: float, retries, parm_type=None):
        '''set a parameter on a mavlink connection'''

        if parm_type is not None and parm_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            # need to encode as a float for sending
            if parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
                vstr = struct.pack(">xxxB", int(value))
            elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
                vstr = struct.pack(">xxxb", int(value))
            elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
                vstr = struct.pack(">xxH", int(value))
            elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
                vstr = struct.pack(">xxh", int(value))
            elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
                vstr = struct.pack(">I", int(value))
            elif parm_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                vstr = struct.pack(">i", int(value))
            else:
                print("can't send %s of type %u" % (name, parm_type))
                return False
            vfloat, = struct.unpack(">f", vstr)
        else:
            vfloat = float(value)

        try:
            mav.param_set_send(name.upper(), vfloat, parm_type=parm_type)

        except Exception as e:
            logger.exception(
                f"failed sending parameter to mav {mav}", exc_info=True)
        time.sleep(0.1)

    def parse_params_file(self, filename, wildcard, use_excludes) -> Dict[str, float]:
        ret = dict()
        with open(filename, mode='r') as f:
            for line in f:
                line = line.strip()
                if not line or line[0] == "#":
                    continue
                line = line.replace(',', ' ')
                a = line.split()
                if len(a) != 2:
                    print("Invalid line: %s" % line)
                    continue
                # some parameters should not be loaded from files
                if use_excludes and a[0] in self.exclude_load:
                    continue
                if not fnmatch.fnmatch(a[0].upper(), wildcard.upper()):
                    continue
                ret[a[0].upper()] = a[1]
        return ret

    def load(self, filename, wildcard='*', mav=None, check=False, use_excludes=True) -> Tuple[bool, str]:
        '''load parameters from a file'''
        if mav is None:
            return
        params_loaded = self.parse_params_file(
            filename, wildcard, use_excludes)
        # TODO: use check to save time
        # load parameters to pending parameters
        for param in params_loaded:
            self._pending_parameter_changes[param] = (
                mav, params_loaded[param], PARAM_RETRIES)

        # flash to mavfile pending parameters
        failed_updates = self.__mavset_pending_parameters()
        # describe result and cleanup
        success = False
        description = ""
        if failed_updates:
            description = f"loaded {len(params_loaded) - len(failed_updates)} out of {len(params_loaded)} parameters"
            logger.warning(description)
        else:
            description = f"{len(params_loaded)} parameters loaded successfully"
            logger.info(description)
            success = True

        self._params_failed.clear()
        return success, description
