# -*- coding: utf-8 -*-
"""
Created on Fri Aug 30 14:42:36 2024

@author: juandavc
"""

from MonitorData import RobotData
import time

data =  RobotData("169.254.22.200")
data.monitor()
time.sleep(5)
data.stop_monitoring()