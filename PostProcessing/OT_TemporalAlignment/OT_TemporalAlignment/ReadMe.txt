OT_Temporal_Alignment
	The application synchronizes between the time stamps received from the Realsense camera and the OptiTrack

Usage:

    OT_Temporal_Alignment -folder <path> -rb <name> <-color | -fe | -gyro | -acc> [-help]


Parameters:

    -folder <path>                Path to input files folder
    -rb <name>                    Name of the camera rigid-body (e.g. "DoubleDome")
    -color | -fe | -gyro | -acc   The RealSense stream(one or more) for syncing with Optitrck (ealiest of chosen streams).
                                  -color sync to the Realsense RGB stream
                                  -fe    sync to the Realsense Fisheye stream
                                  -gyro  sync to the Realsense angular velocity(gyrometer) stream
                                  -acc   sync to the Realsense linear acceleration(accelerometer) stream
    -help                         This command line help


Required data:
	Application should recive a sequence from both Realsense and OptiTrack in 'folder format' (see sample).
	Recorded dataset should start with a static scene (for at least 100 frames/samples)

3rdparty:
    $/SW/CVL/Tools/3rdparty/openCV

