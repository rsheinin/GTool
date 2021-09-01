Projector
	The application projects OT markers poses, from the OT space to the realsense camera space.
	the poses are projected to both camera space and image space.
	if a path to the extracted images is given to the process, the app in addition displays the projected points on the aligned image.
	currently, projector supports projection to color space only.

dependencies
	* $/SW/CVL/Tools/3rdparty/openCV/310

supported configurations:
	* compiler : vc120 / vc140 (recommended)
	* running mode : x64 only. (due to opencv 310 constraints)
	* recommended IDE: visual studio 2015.

location in flow
	this application should run after the temporal alignment application. the input to this application is the output of the record, extract images and temporal alignment tools.

usages:
	*required:
		-folder [path to input files folder]
	*optional:
		-marker [name of object file to be projected]
	*for help menu:
		 -help 
	example: -folder "C:\Users\clopians\Desktop\optitrack\full test" -marker "CB_RigidBody"

input:
	the app should receive a path to a folder with all needed files inside. the folder should include the next components:
	* a folder named "temporalAlignment" with:
		# a text file with the poses of the wanted opti-track recorded object to be projected.
			naming convention:		
			~ a file of a rigid-body poses shoule be named: [rigid-body name]_RigidBody.txt e.g. "CB_Rigidbody.txt"
			~ a file of labled markers should be named "LabledMarker.txt"
		# a text file with the poses of the camera rigid-body. (the rigid-body which is connected to the realsense camera). its name should be: [rigid-body name]Dome_RigidBody.txt e.g. "DoubleDome_Rigidbody.txt"
		both files are a part of the output of the temporal alignment app, which means that the written timestams are after temporal alignment process.
	* a calibration file in ini format. its name should be: [rssdk file name]_settings.ini e.g. rssdk1_settings.ini
	* optional: a folder named "extracted_[rssdk file name]" with all extracted images inside. in case the folder is exist, the app will dispaly the points on the images. 
	to watch an example of a full folder with all needed files inside, go to: $/SW/CVL/Tools/GTool/PostProcessing/Projector/example_folder

ouptput:
	the app provides 2 files in an inner folder named "projection:
	* "projectedCameraSpace.txt" - a file with the poses, projected to the realsense camera (world: x,y,z) space.
	* "projectedImageSpace.txt" - a file with the poses, projected to the realsense image (x,y) space
	the files are in the same format as in the input files.
