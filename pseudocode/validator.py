import glib
import time
import gobject
import os
import hal
import logging
import linuxcnc
import math
import Tkinter, nf, rs274.options
import threading
import cv2
from vaerksValidator import *

from vaerks.job_task_parser import *


class HandlerClass:

    def __init__(self, halcomp, builder, useropts):

        self.halcomp = halcomp  # will be controls.pinname in hal file
        self.halcomp.newpin("scanning-enable", hal.HAL_BIT, hal.HAL_OUT)
  
        
        self.builder = builder
        self.emc = linuxcnc
        self.command = self.emc.command()
        self.status = self.emc.stat()
        self.error = linuxcnc.error_channel()
        self.path_inifile = os.environ.get('INI_FILE_NAME', '/dev/null')
        self.inifile = self.emc.ini(self.path_inifile)
        
        self.tooltable = self.inifile.find("EMCIO","TOOL_TABLE")
        # Enable
        self.button_scan_ready = self.builder.get_object('button_scan_ready')
        self.button_scan_ready.connect('released', self.on_button_scan_ready)
        self.validator = App("C:/Users/Stormholt/Documents/Thesis", False)
        #Table 3d model for pointcloud preprocessing
        table = "table_under_origo_dense.ply"
        self.validator.table = o3d.io.read_point_cloud(self.validator.pcd_path + table)
        
        self.captures = 0
        
        gobject.timeout_add(200, self.periodic)

    def periodic(self):
        self.status.poll()

        return True

    def moveToNextPosition(self, X, Y, Z):
        if (not(isinstance(X, str) and isinstance(Y, str) and isinstance(Z, str))):
            X = str(X)
            Y = str(Y)
            Z = str(Z)
        self.command.mode(linuxcnc.MODE_MDI)
        self.command.wait_complete()
        if(X != None):
            self.command.mdi("G53 G0 X" + X) 
            self.command.wait_complete()
        if(Y != None):
            self.command.mdi("G53 G0 Y" + Y)
            self.command.wait_complete()
        if(Z != None):
            self.command.mdi("G53 G0 Z" + Z)
            self.command.wait_complete()
        return 0

    def on_button_scan_ready(self):
        self.halcomp['button_scan_ready'] = not self.halcomp['button_scan_ready']
        if self.halcomp['button_scan_ready'] == True:
            try:
                
              while True:
                self.moveToNextPosition(X,Y,Z) #Pseudo coordinates
                self.validator.camera.stream()
                key = cv2.waitKey(1)
                if key == 27:
                    break
                if key == 32:
                    self.validator.captureScenePointcloud()
                    self.captures += 1
                    self.validator.camera.y += 4.0
                if self.captures > 20:
                    break
            finally:
                self.validator.saveScenePointcloud(self.validator.Filetype.PLY,"comparison-scene-test0")
                model_color = copy.deepcopy(self.validator.model)
                model_color.paint_uniform_color([1., 0.706, 0.])
                o3d.visualization.draw_geometries([self.validator.scene.pcd + model_color])
                self.validator.compareScene2Model()
                self.validator.camera.release()
                self.captures = 0
        
        
        
        return
        


def get_handlers(halcomp, builder, useropts):
    '''
    this function is called by gladevcp at import time (when this module is passed with '-u <modname>.py')
    return a list of widgetect instances whose methods should be connected as callback handlers
    any method whose name does not begin with an underscore ('_') is a  callback candidate
    the 'get_handlers' name is reserved - gladevcp expects it, so do not change
    '''
    return [HandlerClass(halcomp, builder, useropts)]