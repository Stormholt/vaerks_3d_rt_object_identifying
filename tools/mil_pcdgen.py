from __future__ import print_function

import sys
import ctypes
import mil as  MIL
from mil64 import M_DEFAULT, M_PROC, MdigControlFeature

filename = MIL.MIL_TEXT("C:/Users/Stormholt/Documents/Thesis/pointclouds/matrox-pytest.ply")

#Allocate containers and contexts
MilSystem = MIL.MsysAlloc(MIL.M_DEFAULT, MIL.M_SYSTEM_DEFAULT, MIL.M_DEFAULT, MIL.M_DEFAULT, None) # System
MilDigitizer = MIL.MdigAlloc(MilSystem, MIL.M_DEV0, MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, None) #Digitizer aka Matrox AltiZ
MilGrabId=  MIL.MbufAllocContainer(MilSystem, MIL.M_GRAB+ MIL.M_PROC + MIL.M_DISP , MIL.M_DEFAULT, None) # Container for grabbing the image
MilImageId = MIL.MbufAllocDefault(MilSystem,MilDigitizer, M_PROC, M_DEFAULT, M_DEFAULT, None ) # Container to export pointcloud



MIL.MdigGrab(MilDigitizer, MilGrabId) # Grab image
MIL.MbufConvert3d(MilGrabId, MilImageId, MIL.M_NULL, M_DEFAULT,M_DEFAULT)# Convert to 3D-processable and displayable data, required to export to file
MIL.MbufExport(filename, MIL.M_PLY_ASCII, MilImageId) #Export to a ascii .ply file.

#Free container and contexts.
MIL.MbufFree(MilImageId)
MIL.MbufFree(MilGrabId)
MIL.MdigFree(MilDigitizer)
MIL.MsysFree(MilSystem)
