
import mil as MIL

# Camera calibration grid parameters. 
GRID_NB_ROWS           =  13
GRID_NB_COLS           =  12
GRID_ROW_SPACING       =  5.0     # in mm                
GRID_COL_SPACING       =  5.0     # in mm                

# Laser device setup parameters. 
CONVEYOR_SPEED         =  -0.2     # in mm/frame          

# Fully corrected depth map generation parameters. 
DEPTH_MAP_SIZE_X       =  480      # in pixels            
DEPTH_MAP_SIZE_Y       =  480      # in pixels            
GAP_DEPTH              =  1.5      # in mm                

# Peak detection parameters. 
PEAK_WIDTH_NOMINAL_2      =  9
PEAK_WIDTH_DELTA_2        =  7
MIN_CONTRAST_2            = 75

# Everything below this is considered as noise. 
MIN_HEIGHT_THRESHOLD = 1.0 # in mm 


seq_filename = MIL.MIL_TEXT("C:/Users/STORMH~1/AppData/Local/Temp/MilSequence.avi")
MilApplication = MIL.MappAlloc(MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, None)
MilSystem = MIL.MsysAlloc(MIL.M_DEFAULT, MIL.M_SYSTEM_DEFAULT, MIL.M_DEFAULT, MIL.M_DEFAULT, None)
MilDigitizer = MIL.MdigAlloc(MilSystem, MIL.M_DEFAULT, MIL.MIL_TEXT("M_DEFAULT"), MIL.M_DEFAULT, None)
MilLaser = MIL.M3dmapAlloc(MilSystem, MIL.M_LASER, MIL.M_CALIBRATED_CAMERA_LINEAR_MOTION)
MilScan = MIL.M3dmapAllocResult(MilSystem, MIL.M_POINT_CLOUD_RESULT, MIL.M_DEFAULT)
  
MIL.MbufImportSequence(seq_filename, MIL.M_DEFAULT, MIL.M_NULL, MIL.M_NULL, MIL.M_NULL, MIL.M_NULL,
                                                                           MIL.M_NULL, MIL.M_OPEN)

print("The cookie is being scanned to generate 3D data.\n")

# Read and process all images in the input sequence.
StartTime = MIL.MappTimer(MIL.M_DEFAULT, MIL.M_TIMER_READ + MIL.M_SYNCHRONOUS)

for n in range(NumberOfImages):
    # Read image from sequence. 
    MIL.MbufImportSequence(seq_filename, MIL.M_DEFAULT, MIL.M_LOAD, MIL.M_NULL, MilImage,
                                                                    MIL.M_DEFAULT, 1, MIL.M_READ)

    # Analyze the image to extract laser line and correct its depth. 
    MIL.M3dmapAddScan(MilLaser, MilScan, MilImage, MIL.M_NULL, MIL.M_NULL, MIL.M_POINT_CLOUD_LABEL(1), MIL.M_DEFAULT)

    # Wait to have a proper frame rate, if necessary. 
    EndTime = MIL.MappTimer(MIL.M_DEFAULT, MIL.M_TIMER_READ + MIL.M_SYNCHRONOUS)
    WaitTime = (1.0/FrameRate) - (EndTime - StartTime)
    if WaitTime > 0:
        MIL.MappTimer(MIL.M_DEFAULT, MIL.M_TIMER_WAIT, WaitTime)
    StartTime = MIL.MappTimer(MIL.M_DEFAULT, MIL.M_TIMER_READ + MIL.M_SYNCHRONOUS)

# Close the object sequence file. 
MIL.MbufImportSequence(seq_filename, MIL.M_DEFAULT, MIL.M_NULL, MIL.M_NULL, MIL.M_NULL, MIL.M_NULL,
                                                                        MIL.M_NULL, MIL.M_CLOSE)

# Convert to M_CONTAINER for 3D processing. 
MilContainerId = MIL.MbufAllocContainer(MilSystem, MIL.M_PROC | MIL.M_DISP, MIL.M_DEFAULT)
MIL.M3dmapCopyResult(MilScan, MIL.M_ALL, MilContainerId, MIL.M_POINT_CLOUD_UNORGANIZED, MIL.M_DEFAULT)