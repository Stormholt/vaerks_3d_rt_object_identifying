from mil import *
from vaerksValidator import *


class AltiZ():
    #def __init__(self):
        
    #Allocate containers and contexts
    def allocAltiz(self):
        self.MilSystem = MsysAlloc(M_DEFAULT, M_SYSTEM_DEFAULT, M_DEFAULT, M_DEFAULT, None) # System
        self.MilDigitizer = MdigAlloc(self.MilSystem, M_DEV0, MIL_TEXT("M_DEFAULT"), M_DEFAULT, None) #Digitizer aka Matrox AltiZ
        self.MilGrabId=  MbufAllocContainer(self.MilSystem, M_GRAB+ M_PROC + M_DISP , M_DEFAULT, None) # Container for grabbing the image
        self.MilImageId = MbufAllocDefault(self.MilSystem,self.MilDigitizer, M_PROC, M_DEFAULT, M_DEFAULT, None ) # Container to export pointcloud

    #Generate pointcloud 
    def getPointcloud(self, filename):
        mil_filename = MIL_TEXT(filename)

        MdigGrab(self.MilDigitizer,self. MilGrabId) # Grab image
        MbufConvert3d(self.MilGrabId, self.MilImageId, M_NULL, M_DEFAULT,M_DEFAULT)# Convert to 3D-processable and displayable data, required to export to file
        MbufExport(mil_filename, M_PLY_ASCII, self.MilImageId) #Export to a ascii .ply file.
    
    # Free allocated resources
    def freeAltiz(self):
        #Free container and contexts.
        MbufFree(self.MilImageId)
        MbufFree(self.MilGrabId)
        MdigFree(self.MilDigitizer)
        MsysFree(self.MilSystem)
        

laser = AltiZ()

