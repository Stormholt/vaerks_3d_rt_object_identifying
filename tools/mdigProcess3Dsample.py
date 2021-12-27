#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#########################################################################################
#
#  File name: MdigProcess3D.py
#
#  Synopsis: This program shows the use of the MdigProcess() function and its
#  multiple
#             buffering acquisition to do robust real-time 3D acquisition,
#             processing
#             and display.
#
#             The user's processing code to execute is located in a callback
#             function
#             that will be called for each frame acquired (see
#             ProcessingFunction()).
#
#       Note: The average processing time must be shorter than the grab time or
#       some
#             frames will be missed.  Also, if the processing results are not
#             displayed
#             the CPU usage is reduced significantly.
#
#  Copyright © Matrox Electronic Systems Ltd., 1992-2021.
#  All Rights Reserved
#

# Supporting the print function prototype from 3.0
from __future__ import print_function

import sys
import ctypes
import mil as MIL
import numpy as np


# Text input function differs from 2.7 to 3.0.
if sys.hexversion >= 0x03000000:
    get_input = input
else:
    get_input = raw_input

# User's processing function hook data structure.
class HookDataStruct(ctypes.Structure):
   _fields_ = [("MilDigitizer", MIL.MIL_ID),
      ("MilContainerDisp", MIL.MIL_ID),
      ("MilMapContext",MIL.MIL_ID),
      ("MilMapResult", MIL.MIL_ID),
      ("MilImage", MIL.MIL_ID),
      ("ProcessedImageCount", MIL.MIL_INT)]

# Number of images in the buffering grab queue.
# Generally, increasing this number gives a better real-time grab.
BUFFERING_SIZE_MAX = 5

array = (ctypes.c_float*3)(100)

print(array)
#number_scans = 0
filename2 = MIL.MIL_TEXT("C:/Users/Stormholt/Documents/Thesis/pointclouds/matrox-test.ply")
seq_filename = MIL.MIL_TEXT("C:/Users/Stormholt/Documents/Thesis/matrox-test0.avi")
# User's processing function called every time a grab buffer is ready.
# --------------------------------------------------------------------

def ProcessingFunction(HookType, HookId, HookDataPtr):
   
   # Retrieve the MIL_ID of the grabbed buffer.
   ModifiedBufferId = MIL.MIL_ID(0)
   MIL.MdigGetHookInfo(HookId, MIL.M_MODIFIED_BUFFER + MIL.M_BUFFER_ID, ctypes.byref(ModifiedBufferId))
   
   # Extract the userdata structure
   UserData = ctypes.cast(ctypes.c_void_p(HookDataPtr), ctypes.POINTER(HookDataStruct)).contents
   
   
   # Increment the frame counter.
   UserData.ProcessedImageCount += 1
   
   # Print and draw the frame count (remove to reduce CPU usage).
   print("Processing frame #{:d}.\r".format(UserData.ProcessedImageCount), end='')
   
   # Execute the processing and update the display.
   MIL.MbufConvert3d(ModifiedBufferId, UserData.MilContainerDisp, MIL.M_NULL, MIL.M_DEFAULT, MIL.M_COMPENSATE)
   MIL.MbufCopyColor(ModifiedBufferId, UserData.MilImage, MIL.M_ALL_BANDS)
   #filename = MIL.MIL_TEXT("C:/Users/Stormholt/Documents/Thesis/pointclouds/matrox-test{:d}.ply".format(UserData.ProcessedImageCount))
   #MIL.MbufExport(filename, MIL.M_PLY_ASCII, UserData.MilContainerDisp)
   MIL.MbufExportSequence(seq_filename, MIL.M_DEFAULT, UserData.MilImage, 1, MIL.M_DEFAULT, MIL.M_WRITE)
   #MIL.MbufInquire(UserData.MilContainerDisp, MIL.M_ARRAY, ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)))
   #MIL.MbufSetRegion(ModifiedBufferId,MIL.M_NULL,MIL.M_DEFAULT,MIL.M_DELETE,MIL.M_DEFAULT)
   #MIL.MbufGet(ModifiedBufferId,ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)))
   
   #MIL.M3dmapAddScan(UserData.MilMapContext, UserData.MilMapResult, UserData.MilContainerDisp, MIL.M_NULL, MIL.M_NULL, MIL.M_POINT_CLOUD_LABEL(UserData.ProcessedImageCount), MIL.M_DEFAULT  )
   
   
   return 0
   
# Main function.

# ---------------
def MdigProcessExample():
   # Allocate defaults.
   MilApplication = MIL.MappAlloc(MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, None)
   MilSystem = MIL.MsysAlloc(MIL.M_DEFAULT, MIL.M_SYSTEM_DEFAULT, MIL.M_DEFAULT, MIL.M_DEFAULT, None)

   status, MilDisplay, MilContainerDisp, MilMapContext, MilMapResult = Alloc3dDisplayAndContainer(MilSystem)
   
   
   
   if status == False:
      MIL.MsysFree(MilSystem)
      MIL.MappFree(MilApplication)
      get_input()
      return

   MilDigitizer = MIL.MdigAlloc(MilSystem, MIL.M_DEFAULT, MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, None)
   
   MilImage = MIL.MbufAllocColor(MilSystem, MIL.MdigInquire(MilDigitizer, MIL.M_SIZE_BAND, MIL.M_NULL), MIL.MdigInquire(MilDigitizer,MIL.M_SIZE_X, MIL.M_NULL), MIL.MdigInquire(MilDigitizer, MIL.M_SIZE_Y, MIL.M_NULL), 8+MIL.M_UNSIGNED, MIL.M_IMAGE, MIL.M_NULL)
   
     
   # Print a message.
   print("\nMULTIPLE 3D CONTAINERS PROCESSING.")
   print("-----------------------------------\n")

   # Do a first acquisition to determine what is included in the type camera
   # output.
   MIL.MdigGrab(MilDigitizer, MilContainerDisp)

   # Print the acquired MIL Container detailed informations.
   PrintContainerInfo(MilContainerDisp)

   # If the grabbed Container has 3D data and is Displayable and Processable.
   if (MIL.MbufInquireContainer(MilContainerDisp, MIL.M_CONTAINER, MIL.M_3D_DISPLAYABLE, MIL.M_NULL) != MIL.M_NOT_DISPLAYABLE) and (MIL.MbufInquireContainer(MilContainerDisp, MIL.M_CONTAINER, MIL.M_3D_CONVERTIBLE, MIL.M_NULL) != MIL.M_NOT_CONVERTIBLE):
      # Display the Container on the 3D display.
      MIL.M3ddispSelect(MilDisplay, MilContainerDisp, MIL.M_DEFAULT, MIL.M_DEFAULT)

      # Grab continuously on the display and wait for a key press
      MIL.MdigGrabContinuous(MilDigitizer, MilContainerDisp)
      print("Live 3D acquisition in progress...")
      get_input("Press <Enter> to start processing.\n")

      # Halt continuous grab.
      MIL.MdigHalt(MilDigitizer)

      # Allocate the grab buffers and clear them.
      MilGrabBufferList = (MIL.MIL_ID * BUFFERING_SIZE_MAX)()
      MilGrabBufferListSize = 0
      for n in range(0, BUFFERING_SIZE_MAX):
         MilGrabBufferList[n] = (MIL.MbufAllocContainer(MilSystem, MIL.M_PROC + MIL.M_GRAB, MIL.M_DEFAULT, None))
         if (MilGrabBufferList[n] != MIL.M_NULL):
            MIL.MbufClear(MilGrabBufferList[n], 0xFF)
            MilGrabBufferListSize += 1
         else:
            break

      # Initialize the user's processing function data structure.
      UserHookData = HookDataStruct(MilDigitizer, MilContainerDisp, MilMapContext, MilMapResult, MilImage, 0)
      MIL.MbufExportSequence(seq_filename,MIL. M_DEFAULT, MIL.M_NULL, MIL.M_NULL, MIL.M_DEFAULT,MIL. M_OPEN)
      # Start the processing.  The processing function is called with every
      # frame grabbed.
      ProcessingFunctionPtr = MIL.MIL_DIG_HOOK_FUNCTION_PTR(ProcessingFunction)
      MIL.MdigProcess(MilDigitizer, MilGrabBufferList, MilGrabBufferListSize, MIL.M_START, MIL.M_DEFAULT, ProcessingFunctionPtr, ctypes.byref(UserHookData))

      # Here the main() is free to perform other tasks while the processing is
      # executing.
      # ---------------------------------------------------------------------------------

      # Print a message and wait for a key press after a minimum number of
      # frames.
      print("Processing in progress...")
      get_input("Press <Enter> to stop.                    \n\n")

      # Print a message and wait for a key press after a minimum number of
      # frames.
      MIL.MdigProcess(MilDigitizer, MilGrabBufferList, MilGrabBufferListSize, MIL.M_STOP, MIL.M_DEFAULT, ProcessingFunctionPtr, ctypes.byref(UserHookData))
     
      # Print statistics.
      ProcessFrameCount = MIL.MdigInquire(MilDigitizer, MIL.M_PROCESS_FRAME_COUNT, None)
      ProcessFrameRate = MIL.MdigInquire(MilDigitizer, MIL.M_PROCESS_FRAME_RATE, None)
      print("\n{:d} 3D containers grabbed at {:.1f} frames/sec ({:.1f} ms/frame)".format(ProcessFrameCount, ProcessFrameRate, 1000.0 / ProcessFrameRate))
      get_input("Press <Enter> to end.\n")
      MIL.MbufExportSequence(seq_filename,  MIL.M_DEFAULT,  MIL.M_NULL,  MIL.M_NULL, ProcessFrameRate,  MIL.M_CLOSE)

      #MIL.MbufExportSequence(seq_filename,MIL.M_AVI_MIL, )
      #MIL.MbufGet(MilContainerDisp,ctypes.cast(array, ctypes.POINTER(ctypes.c_void_p)))
      #MIL.M3dmapCopyResult(MilMapResult,MIL.M_ALL,UserHookData.MilContainerDisp, MIL.M_POINT_CLOUD_UNORGANIZED ,MIL.M_NO_REFLECTANCE)
      #print(np.asarray(array))
      #for i in range(0, len(MilContainerDisp)):
      #   filename2 = MIL.MIL_TEXT("C:/Users/Stormholt/Documents/Thesis/pointclouds/matrox-test"+str(i)+".ply")
      #   MIL.MbufExport(filename2, MIL.M_PLY_ASCII, MilContainerDisp[i])
      
      # Free the grab buffers.
      for id in range(0, MilGrabBufferListSize):
         MIL.MbufFree(MilGrabBufferList[id])
   else:
      print("ERROR: The camera provides no (or more than one) 3D Component(s) of type Range or Disparity.\nPress <Enter> to end.\n")
      get_input()

   # Release defaults.
   MIL.MbufFree(MilContainerDisp)
   MIL.M3ddispFree(MilDisplay)
   MIL.M3dmapFree(MilMapContext)
   MIL.M3dmapFree(MilMapResult)
   
   MIL.MdigFree(MilDigitizer)
   MIL.MsysFree(MilSystem)
   MIL.MappFree(MilApplication)
   
   return

# Utility function to print the MIL Container detailed informations.
# ------------------------------------------------------------------
def  PrintContainerInfo(MilContainer):
   ComponentCount = MIL.MbufInquire(MilContainer, MIL.M_COMPONENT_COUNT, None)
   print("Container Information:")
   print("----------------------")
   print("Container:    Component Count: {:d}".format(ComponentCount))
   for  c in range(0,ComponentCount):
      ComponentId = MIL.MbufInquireContainer(MilContainer, MIL.M_COMPONENT_BY_INDEX(c), MIL.M_COMPONENT_ID, None)
      StringSize = MIL.MIL_INT(0)
      MIL.MbufInquire(ComponentId, MIL.M_COMPONENT_TYPE_NAME + MIL.M_STRING_SIZE, ctypes.byref(StringSize))
      if StringSize.value > 0:
         ComponentName = MIL.create_c_string_buffer(StringSize.value)
         MIL.MbufInquire(ComponentId, MIL.M_COMPONENT_TYPE_NAME, ComponentName)
         Name = str(ComponentName.value)

      DataType = MIL.MbufInquire(ComponentId, MIL.M_DATA_TYPE, None)
      if DataType == MIL.M_UNSIGNED :
         DataTypeStr = "u"
      elif DataType == MIL.M_SIGNED:
         DataTypeStr = "s"
      elif DataType == MIL.M_FLOAT:
         DataTypeStr = "f"
      else:
         DataTypeStr = ""

      DataFormat = MIL.MbufInquire(ComponentId, MIL.M_DATA_FORMAT, None) & (MIL.M_PACKED | MIL.M_PLANAR)
      if MIL.MbufInquire(ComponentId, MIL.M_SIZE_BAND, None) == 1:
         SizeBandStr = "Mono"
      elif DataFormat == MIL.M_PLANAR:
         SizeBandStr = "Planar"
      else:
         SizeBandStr == "Packed"

      GroupId = MIL.MIL_INT(0)
      SourceId = MIL.MIL_INT(0)
      RegionId = MIL.MIL_INT(0)
      MIL.MbufInquire(ComponentId, MIL.M_COMPONENT_GROUP_ID, ctypes.byref(GroupId))
      MIL.MbufInquire(ComponentId, MIL.M_COMPONENT_SOURCE_ID, ctypes.byref(SourceId))
      MIL.MbufInquire(ComponentId, MIL.M_COMPONENT_REGION_ID, ctypes.byref(RegionId))
      print("Component[{:d}]: {:11s}[{:d}:{:d}:{:d}] Band: {:1d}, Size X: {:4d}, Size Y: {:4d}, Type: {:2d}{:s} ({:6s})".format(c, Name, GroupId.value,
                SourceId.value,
                RegionId.value,
                MIL.MbufInquire(ComponentId, MIL.M_SIZE_BAND, None), MIL.MbufInquire(ComponentId, MIL.M_SIZE_X, None),
                MIL.MbufInquire(ComponentId, MIL.M_SIZE_Y, None), MIL.MbufInquire(ComponentId, MIL.M_SIZE_BIT, None),
                DataTypeStr,
                SizeBandStr))
   print("")

# *****************************************************************************
# Allocates a 3D display and returns its MIL identifier.
# *****************************************************************************
def Alloc3dDisplayAndContainer(MilSystem):
   # First we check if the system is local
   if (MIL.MsysInquire(MilSystem, MIL.M_LOCATION, MIL.M_NULL) != MIL.M_LOCAL):
      print("This example requires a 3D display which is not supported on a remote system.")
      print("Please select a local system as the default.")
      return (False, MIL.M_NULL, MIL.M_NULL)

   MIL.MappControl(MIL.M_DEFAULT, MIL.M_ERROR, MIL.M_PRINT_DISABLE)
   MilDisplay = MIL.M3ddispAlloc(MilSystem, MIL.M_DEFAULT, MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, MIL.M_NULL)
   MilContainerDisp = MIL.MbufAllocContainer(MilSystem, MIL.M_PROC + MIL.M_DISP + MIL.M_GRAB, MIL.M_DEFAULT, None)
   MilMapContext = MIL.M3dmapAlloc(MilSystem,MIL.M_LASER ,MIL.M_CALIBRATED_CAMERA_LINEAR_MOTION,MIL.M_NULL)
   MilMapResult = MIL.M3dmapAllocResult(MilSystem,MIL.M_POINT_CLOUD_RESULT ,MIL.M_DEFAULT, MIL.M_NULL )
  
   if (MilContainerDisp == MIL.M_NULL) or (MilDisplay == MIL.M_NULL):
     ErrorMessage = MIL.create_c_string_buffer(MIL.M_ERROR_MESSAGE_SIZE)
     ErrorMessageSub1 = MIL.create_c_string_buffer(MIL.M_ERROR_MESSAGE_SIZE)
     MIL.MappGetError(MIL.M_DEFAULT, MIL.M_GLOBAL + MIL.M_MESSAGE, ErrorMessage)
     MIL.MappGetError(MIL.M_DEFAULT, MIL.M_GLOBAL_SUB_1 + MIL.M_MESSAGE, ErrorMessageSub1)
     print()
     print("The current system does not support the 3D display.")
     print("   " + str(ErrorMessage.value))
     print("   " + str(ErrorMessageSub1.value))
     print()
     if MilDisplay != MIL.M_NULL:
         MIL.M3ddispFree(MilDisplay)
     if MilContainerDisp != MIL.M_NULL:
         MIL.MbufFree(MilContainerDisp)
     return (False, MIL.M_NULL, MIL.M_NULL)

   MIL.MappControl(MIL.M_DEFAULT, MIL.M_ERROR, MIL.M_PRINT_ENABLE)

   return (True, MilDisplay, MilContainerDisp,MilMapContext,MilMapResult)

if __name__ == "__main__":
   MdigProcessExample()
