#include "stdafx.h"
#include "OT_Projector.h"

//create cv::Mat of intrinsics params
cv::Mat toCV(const Projection::CameraIntrinsics &dpi, cv::Mat &dist = cv::Mat())
{
	cv::Mat cvi = cv::Mat::zeros(3, 3, CV_64F);
	cvi.at<double>(0, 0) = dpi.fx;
	cvi.at<double>(1, 1) = dpi.fy;
	cvi.at<double>(2, 2) = 1.;
	cvi.at<double>(cv::Point(2, 0)) = dpi.px;
	cvi.at<double>(cv::Point(2, 1)) = dpi.py;

	dist.create(1, 5, CV_64F);
	for (int i = 0; i < dist.cols; i++)
		dist.at<double>(cv::Point(i, 0)) = dpi.k[i];

	return cvi;
}

bool readFilesForProjection(std::string mainFolder)
{
	char search_dir_path[200];
	sprintf_s(search_dir_path, "%s\\%s*", mainFolder.c_str(), folderToLoopPrefix.c_str());//look for temporalAlignment folders
	WIN32_FIND_DATAA fd_dir;
	HANDLE hFind_dir = FindFirstFileA(search_dir_path, &fd_dir);
	if (hFind_dir == INVALID_HANDLE_VALUE)
	{
		return false;
	}
	do
	{		
		char search_file_path[200];
		sprintf_s(search_file_path, "%s\\%s\\%s*.csv", mainFolder.c_str(), fd_dir.cFileName, file_prefix.c_str());//look for temporalAlignment folders
		WIN32_FIND_DATAA fd_file;
		HANDLE hFind_file = FindFirstFileA(search_file_path, &fd_file);
		if (hFind_file == INVALID_HANDLE_VALUE)
		{
			break;
		}
		do
		{
			
			std::string path = mainFolder + "\\" + fd_dir.cFileName + "\\" + fd_file.cFileName;
			//if this is the head file
			if (std::string(fd_file.cFileName).find(head_name) != std::string::npos) {
				headFile = path;
			}
			if(path == headFile || rel_element)
				files.push_back(path);
			//if (std::find(files.begin(), files.end(), path) == files.end()) {
			//	// path not in files already, add it
			//	files.push_back(path);
			//}
			
		} while (FindNextFileA(hFind_file, &fd_file));
		
	} while (FindNextFileA(hFind_dir, &fd_dir));
}

//read the file with the data (=poses) of the camera Rigidbody (usually named something with "dome")
Projection::poses readCameraData(std::string fileName, Pose::PoseType type = Pose::PoseType::GROUNDTRUTH)
{
	std::cout << "reading poses from: " + fileName << std::endl;
	std::ifstream file(fileName);
	if (!file.is_open())
	{
		return Projection::poses();
	}

	Projection::poses poses;
	std::string line;
	std::string::size_type sz = 0;
	if(type == Pose::PoseType::GROUNDTRUTH)
		std::getline(file, line);		//read titles
	while (std::getline(file, line))
	{
		Pose::PoseStruct pose;
		if (type == Pose::PoseType::GROUNDTRUTH)
			pose = Pose::GT::str2pose(line);

		else if (type == Pose::ALGORITHM)
			pose = Pose::Source::str2Pose(line);

		if (poses.size() > 0 && abs(pose.ts - poses[poses.size() - 1].ts) < 0.01)//case there is already a pose with the same timestamp, replace it by the later one.
			poses.pop_back();
		poses.push_back(pose);
	}
	return poses;
}

//read the file with the GHC mat - a transformation matrix between the camera lens and the connected RG
std::string readGhcData(std::string fileName)
{
	std::ifstream file(fileName);
	if (!file.is_open())
	{
		return std::string();
	}

	std::string line;
	std::string::size_type sz = 0;
	std::getline(file, line);
	size_t firstSpace = line.find_first_of("=");
	return std::string(line.begin() + firstSpace + 1, line.end());
}

//read the file with the data of the camera intrinsics
short readCalibrationParams(Projection::CameraIntrinsics& intrinsics, std::string fileName, cv::Mat& ghcMat, QuaternionUtils::Quat& q_ghc)
{
	short projectionType = 0b0;
	try
	{
		IOini::CIOini iniSettings;
		iniSettings.setFileName(fileName);
		std::string val;
		std::string section;
		if (stream == Projection::COLOR)
			section = "Color_Cam_Params";
		else if (stream == Projection::FE)
			section = "FE_Cam_Params";
		else
		{
			//implement error msg...
		}
		iniSettings.getKeyValue("fx", section, val, fileName);
		intrinsics.fx = std::stof(val);
		iniSettings.getKeyValue("fy", section, val, fileName);
		intrinsics.fy = std::stof(val);
		iniSettings.getKeyValue("px", section, val, fileName);
		intrinsics.px = std::stof(val);
		iniSettings.getKeyValue("py", section, val, fileName);
		intrinsics.py = std::stof(val);
		iniSettings.getKeyValue("width", section, val, fileName);
		intrinsics.w = std::stoi(val);
		iniSettings.getKeyValue("height", section, val, fileName);
		intrinsics.h = std::stoi(val);
		if (stream == Projection::FE)
		{
			iniSettings.getKeyValue("w", section, val, fileName);
			intrinsics.d_w = std::stof(val);
		}
		//iniSettings.getKeyValue("otCMarker2cm", "Transformation", gHcMatStr, fileName);

		projectionType |= Projection::PROJECTION_2D;
		//return projectionType;
	}
	catch (const std::exception&)
	{
		std::cout << "can not find metadata file of camera. ini file does not exist. projection will not be possible for 2d space" << std::endl;
		
	}
	std::string gHcMatStr = GHCUtils::getGhcByRBName(object_name);
	q_ghc = QuaternionUtils::Quat::matStr2Quat(gHcMatStr);
	if (Pose::str2Matrix(gHcMatStr, ghcMat, ' '))
	{
		OutputUtils::printColoredText("GHC matrix for " + object_name + " is: " + gHcMatStr, logger);
		projectionType |= Projection::PROJECTION_3D;
	}
	else
		OutputUtils::errorMessage("a ghc string for: " + object_name + " is not found or not valid", logger);
	return projectionType;
}

//read user arguments and update variables accordingly
void readUserArgs(int argc, char ** argv)
{
	std::cout << "Reading user arguments" << std::endl;
	
	for (int i = 1; i < argc; i++)
	{
		//required arguments:
		if (strcmp(argv[i], "-folder") == 0)			//a location to a folder with all needed files (OT output files + ini file + extracted images folder for display if wanted)
		{
			folder = std::string(argv[++i]);
		}
		else if (strcmp(argv[i], "-head") == 0)			//a path to the camera RB poses file. (the RB which is connected rigidly to the RealSense camera). default: looks for a file with the suffix "Dome_RigidBody.csv" in the input folder.
		{
			head_name = std::string(argv[++i]);
			Projection::updateProjectedObjectName(head_name);
		}
		else if (strcmp(argv[i], "-src") == 0)			//name of source (6dof) file. need to be txt file and located in given folder
		{
			srcName = std::string(argv[++i]);
			rcName = srcName;
		}
		//else if (strcmp(argv[i], "-marker") == 0)		//the name of the object poses file (the sensor one wants to project its poses). for example: cb_RigidBody, LabledMarkers...
		//{
		//	object_name = std::string(argv[++i]);
		//}
		else if (strcmp(argv[i], "-o") == 0)					//a location to where the output files should be. defualt: an inner folder named "projection" in the input given folder.
		{
			outFile = std::string(argv[++i]);
		}		
		//else if (strcmp(argv[i], "-ts") == 0)			//a double timestamp which global projection will be relatively to its algo pose
		//{
		//	first_ts = atof(argv[++i]);
		//}
		else if (strcmp(argv[i], "-ta") == 0)			//a double timestamp which global projection will be relatively to its algo pose
		{
			folderToLoopPrefix = std::string(argv[++i]);
		}		
		//else if (strcmp(argv[i], "-a") == 0)			//a boolean flag which applys projection on temporaly-aligned data
		//{
		//	file_prefix = TEMPORAL_ALIGNED_FILE_PREFIX;
		//}
		//else if (strcmp(argv[i], "-i") == 0)			//a boolean flag which applys projection on interpolated data
		//{
		//	file_prefix = INTERPOLATED_FILE_PREFIX;
		//}
		else if (strcmp(argv[i], "-ctrl") == 0/*strcmp(argv[i], "-ctrl1") == 0 || strcmp(argv[i], "-ctrl2") == 0*/)			//a boolean flag which applys projection on controller/s (relative component) file/s. default is: false
		{
			rel_element = true;
		}
		else if (strcmp(argv[i], "-hmd") == 0)			//a boolean flag which applys projection on the hmd (origin component) file. default is: false
		{
			org_element = true;
		}

		else if (strcmp(argv[i], "-ghc") == 0)			//a ghc string, for 3d projection only.
		{
			std::vector<std::string> tokens;
			StringUtils::str2tokens(argv[++i], tokens);
			if (tokens.size() != 2)
				OutputUtils::errorMessage("invalid argument: " + std::string(argv[i - 1]));
			if (ghc_map.find(tokens[0]) == ghc_map.end())// object name is not exist already
				ghc_map.insert(std::make_pair(tokens[0], tokens[1]));
		}
		else if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "-help") == 0)
		{
			OutputUtils::errorMessage(helpMenu());
		}
		//arguments for 2D projection
		else if (strcmp(argv[i], "-calib") == 0)		//a path to the device calibration file. (need to include the camera intrinsics and the GHC matrix). default: looks for a file with the suffix ".ini" in the input folder.
		{
			iniSettingsFile = std::string(argv[++i]);
		}
		else if (strcmp(argv[i], "-img") == 0)			//a path to the extracted images folder. default: looks for a folder with the prefix "extract" in the input folder.
		{
			imagesFolder = std::string(argv[++i]);
		}
		else if (strcmp(argv[i], "-fe") == 0)			//a boolean flag which indicates whether the images are of type fisheye and not color
		{
			stream = Projection::FE;
		}
		else if (strcmp(argv[i], "-nd") == 0)			//a boolean flag which indicates whether to cancel the display of the projected points on the compatible device images
		{
			display = false;
		}
		else if (strcmp(argv[i], "-s") == 0)			//a boolean flag which indicates whether to save the images with the projected points drawn on them.
		{
			saveImages = true;
		}
		else
			OutputUtils::printColoredText("Unknown argument: " + std::string(argv[i]), 8);
	}

	/*if (object_name.empty())
		OutputUtils::errorMessage("Not enough arguments have been given to process. Please give the folder path to where needed files are located and the file's name of wanted markers to be projected\nRun with -h for help");
	OTDataFile = folder + "\\" + folderToLoopPrefix + "\\" + TEMPORAL_ALIGNED_FILE_PREFIX + object_name + ".csv";
	Projection::updateProjectedObjectName(object_name);*/
	if (folder.empty() || head_name.empty() || object_name.empty())
		OutputUtils::errorMessage(helpMenu());

	//create folder for output files
	if (outFile.empty())
		outFile = folder + "\\" + CURRENT_DIRECTORY_NAME;
	else
		outFile = folder + "\\" + outFile;
	auto err = _mkdir(outFile.c_str()) != 0;
	char message[200];
	strerror_s(message, 200, errno);
	std::cout << "creating folder for output files: " << message << std::endl;
	
	if (!srcName.empty())
		srcPath = folder + "\\" + srcName + source_file_extension;
	//if (cameraOTDataFile.empty())
	//{
	//	char search_path[200];
	//	sprintf_s(search_path, "%s\\%s\\%s*Dome_RigidBody.csv", folder.c_str(), folderToLoopPrefix, file_prefix);
	//	WIN32_FIND_DATAA fd;
	//	HANDLE hFind = FindFirstFileA(search_path, &fd);
	//	if (hFind == INVALID_HANDLE_VALUE)
	//	{
	//		OutputUtils::errorMessage("can not find a temporal-aligned file for camera rigid body. (searching for path: " + std::string(search_path));
	//	}
	//	cameraOTDataFile = folder +"\\" + folderToLoopPrefix + "\\" + fd.cFileName;
	//	Projection::updateProjectedObjectName(fd.cFileName);
	//}
	//else
	//{
	//	int loc = cameraOTDataFile.find_last_of("\\");
	//	if (loc < 0)//this is only the rb name and not a full path to file 
	//		cameraOTDataFile = folder + "\\" + folderToLoopPrefix + "\\" + file_prefix +  cameraOTDataFile + RIGIDBODY_FILE_EXTENSION + ".csv";
	//	Projection::updateProjectedObjectName(cameraOTDataFile.substr(cameraOTDataFile.find_last_of("\\") + 1));
	//}
	if (iniSettingsFile.empty())
	{
		char search_path[200];
		sprintf_s(search_path, "%s/*.ini", folder.c_str());
		WIN32_FIND_DATAA fd;
		HANDLE hFind = FindFirstFileA(search_path, &fd);
		if (!(hFind == INVALID_HANDLE_VALUE))
		{
			iniSettingsFile = folder + "\\" + fd.cFileName;
		}
	}
	if (imagesFolder.empty())
	{
		char search_path[200];
		sprintf_s(search_path, "%s\\*extract*", folder.c_str());
		WIN32_FIND_DATAA fd;
		HANDLE hFind = FindFirstFileA(search_path, &fd);
		if (hFind != INVALID_HANDLE_VALUE)
		{
			if (stream == Projection::COLOR)
			{
				imagesFolder = folder + "\\" + fd.cFileName + "\\color";
			}
			else if (stream == Projection::FE)
			{
				imagesFolder = folder + "\\" + fd.cFileName + "\\fisheye";
			}
		}
		else
			OutputUtils::printColoredText("no images folder was found", OutputUtils::WARNING_COLOR);
	}
	//}
}

std::string getInformation()
{
	std::string info = "\n----Projection Information----\nmain folder is: " + folder
		+ "\noutput folder is: " + outFile
	    + "\nprojection is for: " + ((file_prefix == INTERPOLATED_FILE_PREFIX) ? "interpolated" : "temporally aligned") + " data"
		+ "\nprefix of TA folders to loop is: " + folderToLoopPrefix
		+ "\nhead file is: " + headFile
		+ "\ninput files for projection are: ";
		for each (std::string file in files)
		{
			info += "\n" + file;
		}
		
		info += "\nsource is: " + srcName
			//+ "\nRC name is: " + rcName
			+ "\nRB name is: " + head_name
			+ "\n";
		return info;
}

//TODO: update
std::string helpMenu()
{
	std::string str = "usage:";
	
	str += "\n-folder\ta location to a folder with all needed files (GT output files, source file and temporal alignment folder)";
	str += "\n-head\tthe name of the hmd GT poses file without extension. for example: ES3_20_12";
	str += "\n-src\tthe name of the hmd source poses file without extension. for example: hmd_calibration_20_12. note that this file poses should be interpolated to the wanted-to-be-projected object timestamps";
	str += "\n-o\ta location to where the output files should be. defualt: an inner folder named 'projection' in the input given folder";
	str += "\n-ta\ta string expression, matching the name of the temporal alignment folder/s. defualt: temporalAlignment. for example: *6dof*temporal";
	str += "\n-hmd\ta boolean flag which applys projection on the hmd (origin component) file. default is: false";
	str += "\n-ctrl\ta boolean flag which applys projection on controller/s (relative component) file/s. default is: false";
	str += "\n-ghc\ttransformation matrix in a string format with 12 values, for each RB involved in projection process. For example: ES3,1 0 0 0 0 1 0 0 0 0 1 0. by default process looks for a ghc file, according to RB name in an agreed folder";
	str += "\n\n arguments from older versions (not supported continuously):";
	str += "\n-calib\ta path to the device calibration file. (need to include the DS intrinsics and the GHC matrix). default: looks for a file with the suffix '.ini' in the input folder.";
	str += "\n-img\ta path to the extracted images folder. default: looks for a folder with the prefix 'extract' in the input folder";
	str += "\n-nd\ta boolean flag which indicates whether to cancel the display of the projected points on the compatible device images";
	str += "\n-s\ta boolean flag which indicates whether to save the device images with the projected points drawn on them";
	str += "\n";
	str += "\nexample:\t-folder C:\\Users\\clopians\\Desktop\\GTool\\calibration_tests\\ctrl_calibration_18_1_18_4 -head ES3_20_12 -src hmd_interpolated_to_all -o try_proj -ta val_6dof_temporalAlignment_ctrl_A_18_1_18 -ctrl";

	return str;
}

//update the projected markers object name
void Projection::updateProjectedObjectName(std::string markerName)
{
	markerName = markerName.substr(markerName.find_last_of('\\') + 1);//extract file name from file path
	//if this name contains the file prefix - remove it.
	if(markerName.find(file_prefix) != std::string::npos)
		markerName = markerName.substr(file_prefix.length());
	object_name = markerName.substr(0, markerName.find_last_of('.'));

	/*auto a = markerName.find_first_of("_");
	if (markerName.substr(0, markerName.find_first_of("_") + 1) == file_prefix)
		markerName = markerName.substr(markerName.find_first_of("_") + 1);
	int loc = markerName.find_last_of("_");
	if (loc != std::string::npos)
		object_name = markerName.substr(0, loc);
	else
		object_name = "labled markers";*/
}

//read device timestamps and store them in a map (key=image number, value=image ts)
Projection::timestamps Projection::ReadDeviceTS(std::string path)
{
	//Read timestamps file
	timestamps device_ts;
	//device_ts.clear();

	std::ifstream file(path);
	if (!file.is_open())
	{
		return device_ts;
	}

	std::string line;
	while (std::getline(file, line))
	{
		double ts = std::stod((line.substr(0, line.find_first_of(Pose::GT::pose_delimiter))).c_str());
		int frameNum = atoi((line.substr(line.find_last_of('e') + 1, line.find_last_of('.'))).c_str());
		device_ts.insert(std::make_pair(frameNum, ts));
	}
	return device_ts;
}

int getIndexOfNearestTS(double ts, Projection::poses& cameraPoses)
{
	/*if (ts == NOTYET)
		return 0;*/
	double nearest_ts = NOTYET;
	double diff = DBL_MAX;

	int loc = NOTYET;
	size_t i;
	for (i = 0; i < cameraPoses.size(); i++)
	{
		if (cameraPoses[i].ts <= ts && abs(cameraPoses[i].ts - ts) < diff)
		{
			diff = abs(cameraPoses[i].ts - ts);
			loc = i;
		}
	}
	return loc;
}

int getIndexOfEqualTS(double ts, const Projection::poses& cameraPoses)
{
	for (size_t i = 0; i < cameraPoses.size(); i++)
	{
		if (abs(ts - cameraPoses[i].ts) < 0.01)
		{
			return i;
		}
	}
	return NOTFOUND;
}

QuaternionUtils::Quat getFirstSrcPose(double& first_ts)
{
	std::ifstream srcFile;
	char search_path[200];
	sprintf_s(search_path, "%s\\*%s", folder.c_str(), source_file_extension.c_str());
	WIN32_FIND_DATAA fd;
	HANDLE hFind = FindFirstFileA(search_path, &fd);
	std::string prevFile = "";
	if (hFind == INVALID_HANDLE_VALUE)
	{
		OutputUtils::printColoredText("Can not find a 6dof file. (Searching for path: " + std::string(search_path) + ")", OutputUtils::TextColor::WARNING_COLOR);
		first_ts = NOTYET;
		return QuaternionUtils::Quat::eye();
	}
	while ((std::string(fd.cFileName).find("rotmat_for_comp") != std::string::npos
		|| std::string(fd.cFileName).find("ot_gt") != std::string::npos
		|| std::string(fd.cFileName).find("ot_pose") != std::string::npos)
		&& fd.cFileName != prevFile)
	{
		prevFile = fd.cFileName;
		FindNextFileA(hFind, &fd);
	}
	if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && fd.cFileName != prevFile)
	{
		std::string fileName= srcName.empty()? fd.cFileName : srcName + source_file_extension;
		std::string srcFileName = folder + "\\" + fileName;
		OutputUtils::printColoredText("source file is: " + srcFileName, OutputUtils::GRAY);
		srcFile = std::ifstream(srcFileName);
		rcName = fileName;
		rcName = rcName.substr(0, rcName.find_first_of('.'));
		QuaternionUtils::Quat q_pose;
		std::string line, pose;
		while (pose.empty() && std::getline(srcFile, line))
		{
			try
			{
				double ts = std::stod(line.substr(0, line.find_first_of(' ')));
				if (first_ts == ts/* * 1000*/ || first_ts == NOTYET)//this is the wanted ts, or if not set, the default ts is the first one 
					pose = line;
			}
			catch (const std::exception&)
			{
				OutputUtils::printColoredText("got invalid pose from source file: " + line, OutputUtils::WARNING_COLOR);
				first_ts = NOTYET;
				return QuaternionUtils::Quat::eye();
			}			
		}
		if (pose == "")//reached end of file and didn't find the wanted timestamp
		{
			OutputUtils::printColoredText("didn't find wanted timestamp " + std::to_string(first_ts) + " in 6dof file: " + srcFileName, OutputUtils::TextColor::WARNING_COLOR);
			first_ts = NOTYET;
			return QuaternionUtils::Quat();
		}

		first_ts = std::stod(pose.substr(0, pose.find_first_of(' '))) * 1000;
		pose = pose.substr(pose.find_first_of(' ') + 1);//remove ts
		std::vector<std::string> tokens;
		StringUtils::str2tokens(pose, tokens, ' ');
		if (tokens.size() < QuaternionUtils::QUATERNIONSIZE + QuaternionUtils::TRANSLATIONSIZE)
		{
			OutputUtils::printColoredText("got invalid pose from source file: " + srcFileName + " for ts: " + std::to_string(first_ts), OutputUtils::TextColor::WARNING_COLOR);
			first_ts = NOTYET;
			return QuaternionUtils::Quat();
		}
		q_pose = QuaternionUtils::Quat(std::stod(tokens[QuaternionUtils::QX + QuaternionUtils::TRANSLATIONSIZE]), 
			std::stod(tokens[QuaternionUtils::QY + QuaternionUtils::TRANSLATIONSIZE]), 
			std::stod(tokens[QuaternionUtils::QZ + QuaternionUtils::TRANSLATIONSIZE]), 
			std::stod(tokens[QuaternionUtils::QW + QuaternionUtils::TRANSLATIONSIZE]),
			std::stod(tokens[QuaternionUtils::TX]) * 1000,
			std::stod(tokens[QuaternionUtils::TY]) * 1000,
			std::stod(tokens[QuaternionUtils::TZ]) * 1000);
		return q_pose;
	}//file was not found
	else
	{
		OutputUtils::printColoredText("6dof file is not found", OutputUtils::TextColor::WARNING_COLOR);
		return QuaternionUtils::Quat();
	}	
}

bool getReferencePoseIndexes(const Projection::poses& srcPoses, const Projection::poses& gtPoses, int* srcIndex, int* gtIndex)
{
	if (srcPoses.size() <= 1)
		return false;
	double spf = ((srcPoses[srcPoses.size() - 1].ts)/1000 - (srcPoses[0].ts)/1000) / (srcPoses.size() - 1);
	*srcIndex = (int)(Pose::Source::initPhase / spf);
	do
	{
		(*srcIndex)++;
		*gtIndex = getIndexOfEqualTS(srcPoses[*srcIndex].ts, gtPoses);
	} while (*gtIndex == NOTFOUND || !(gtPoses[*gtIndex].tracked) && *srcIndex < srcPoses.size() - 2);//while gt reference pose is not tracked, look for another pair of src and gt poses
	
	if (*srcIndex >= 0 && *srcIndex < srcPoses.size() && *gtIndex >= 0 && *gtIndex < gtPoses.size() && abs(srcPoses[*srcIndex].ts - gtPoses[*gtIndex].ts) < 0.0001)
		return true;
	return false;
}

cv::Point2f Projection::distortPoint(cv::Point2f p)
{
	return p;
}

std::string GHCUtils::getGhcByRBName(std::string objName)
{
	if (ghc_map.find(objName) == ghc_map.end())//this object's ghc is not exist in map - have not been specified by user, look for its file in the agreed location
	{
		TCHAR working_dir[MAX_PATH];
		auto wd = GetCurrentDirectory(MAX_PATH, working_dir);
		//std::wcout << "working directory of projector: " << working_dir << std::endl;
		TCHAR full_path[MAX_PATH];
		std::string relativePath = "..\\..\\..\\..\\H&Icalibration\\GHCFiles\\" + objName + ".txt";
		std::wstring w_relativePath = std::wstring(relativePath.begin(), relativePath.end());
		GetFullPathName(w_relativePath.c_str(), MAX_PATH, full_path, NULL);
		std::ifstream ghcFile(full_path);
		//std::wcout << "ghc file of " << std::wstring(objName.begin(), objName.end()) << " is: " << full_path << std::endl;
		if (!ghcFile.is_open())
			return "1 0 0 0 0 1 0 0 0 0 1 0";
			//return "";
		std::string ghc;
		std::getline(ghcFile, ghc);
		ghc = ghc.substr(ghc.find_first_of('=') + 1);
		return ghc;
	}
	else
		return ghc_map[objName];
}

int main(int argc, char ** argv)
{
	OutputUtils::printColoredText("-----------------PROJECTION-----------------", OutputUtils::TextColor::PINK + OutputUtils::TextColor::SHINING);
	
	readUserArgs(argc, argv);//read program arguments from user
	readFilesForProjection(folder);//get all involved files from given folder/s

	logger = std::ofstream(outFile + "\\logger.txt");

	OutputUtils::printColoredText(getInformation(), logger);
	
	//initrinsics settings
	Projection::CameraIntrinsics cameraIntrinsics;
	cv::Mat camera_ghcMat;
	QuaternionUtils::Quat q_camera_ghcMat;

	//read all data relevant for calibration: intrisics, extrinsics, GHC matrix etc.
	short projection_type = readCalibrationParams(cameraIntrinsics, iniSettingsFile, camera_ghcMat, q_camera_ghcMat); // read device intrinsics from ini settings file + GHC mat 

	OutputUtils::printColoredText("3d projection is: " + std::string((projection_type & Projection::PROJECTION_3D) != 0 ? "enabled" : "disabled"), logger);
	OutputUtils::printColoredText("2d projection is: " + std::string((projection_type & Projection::PROJECTION_2D) != 0 ? "enabled" : "disabled"), logger);
	
	if ((projection_type & Projection::PROJECTION_3D) != 0)//projection to 3d space is enabled (valid ghc matrix)
	{
		//read camera GT data
		Projection::poses cameraGTPoses = readCameraData(headFile, Pose::PoseType::GROUNDTRUTH);
		if (cameraGTPoses.size() == 0)
		{
			OutputUtils::errorMessage("there are no poses of camera GT", logger);
		}
		//read camera src data
		Projection::poses cameraSrcPoses = readCameraData(srcPath, Pose::PoseType::ALGORITHM);
		if (cameraSrcPoses.size() == 0)
		{
			OutputUtils::printColoredText("Couldn't find poses of camera source. Projection will be done in relative to GT space and not to source space", logger, OutputUtils::WARNING_COLOR);
		}		

		int gtReferencePoseIndex, srcReferencePoseIndex;
		cv::Mat	cameraSrcReferencePose, cameraGTReferencePose;
		QuaternionUtils::Quat q_cameraSrcReferencePose, q_cameraGTReferencePose;
		bool enableGlobal = getReferencePoseIndexes(cameraSrcPoses, cameraGTPoses, &srcReferencePoseIndex, &gtReferencePoseIndex);
		if (enableGlobal) //global projection is enabled
		{
			cameraSrcReferencePose = cameraSrcPoses[srcReferencePoseIndex].rotationMat;
			q_cameraSrcReferencePose = cameraSrcPoses[srcReferencePoseIndex].q;
			cameraGTReferencePose = cameraGTPoses[gtReferencePoseIndex].rotationMat;
			q_cameraGTReferencePose = cameraGTPoses[gtReferencePoseIndex].q;
			OutputUtils::printColoredText("reference pose index from source is: " + std::to_string(srcReferencePoseIndex) + " with timestamp " + std::to_string(cameraSrcPoses[srcReferencePoseIndex].ts), logger);
			OutputUtils::printColoredText("reference pose index from GT is: " + std::to_string(gtReferencePoseIndex) + " with timestamp " + std::to_string(cameraGTPoses[gtReferencePoseIndex].ts), logger);
			OutputUtils::printColoredText("src: " + Pose::mat2str(cameraSrcReferencePose), logger);
			OutputUtils::printColoredText("gt: " + Pose::mat2str(cameraGTReferencePose), logger);
		}
		else
			OutputUtils::printColoredText("WARNING: global projection is disabled. didn't find a pair of GT pose and source pose with an equal timestamp", logger, OutputUtils::WARNING_COLOR);

		for each (std::string file in files)
		{
			if (file == headFile && org_element || file != headFile && rel_element)//projection is for hmd and this is an hmd file, or projection is for controller and this is a controller file
			{
				Projection::updateProjectedObjectName(file);
				//read wanted markers-object data 
				std::ifstream objectfile(file); //file of markers-object data
				if (!objectfile.is_open())
				{
					OutputUtils::printColoredText("Problem in openning " + object_name + " file. Path is: " + file, logger, OutputUtils::WARNING_COLOR);
					break;
				}

				cv::Mat object_ghcMat;
				if (!Pose::str2Matrix(GHCUtils::getGhcByRBName(object_name), object_ghcMat, ' '))
				{
					OutputUtils::printColoredText("failed to find ghc matrix for: " + object_name + ". " + object_name + " file will not be saved!", logger, OutputUtils::WARNING_COLOR);
					break;
				}
				
				QuaternionUtils::Quat q_object_ghcMat(QuaternionUtils::Quat::matStr2Quat(GHCUtils::getGhcByRBName(object_name)));

				std::string line;
				std::string::size_type sz = 0;

				//copy titles of GT objects data file to all output files	
				std::getline(objectfile, line);
				line += ",q_tx,q_ty,q_tz";
				//TODO: skip relative projection for head file
				std::ofstream outfileR3d(outFile + "\\" + object_name + "_" + relative_projection_suffix + input_file_extension); //output file for projected points to color 3d space
				outfileR3d << line << "\n";
				std::ofstream outfileG3d(outFile + "\\" + object_name + "_" + global_projection_suffix + input_file_extension); //output file for projected points to color 3d space
				std::ofstream outfileG3dTUM;
				std::string tumFile = rcName.empty() ? outFile/*folder*/ + "\\" + object_name + "_" + global_projection_suffix + output_file_extension : outFile/*folder*/ + "\\" + object_name + "_" + rcName + output_file_extension;
				outfileG3dTUM = std::ofstream(tumFile); //output file for projected points to color 3d space
				outfileG3d << line << "\n";
				
				Projection::imageColorSpacePoints.clear();
				Projection::imagePoints_tmp.clear();

				OutputUtils::printColoredText("Projecting poses points of: " + object_name + ". please wait...", OutputUtils::TextColor::PINK + OutputUtils::TextColor::SHINING);
				OutputUtils::printColoredText("ghc for " + object_name + " is: \n" + q_object_ghcMat.toStr(), logger, OutputUtils::TextColor::LIGHTBLUE + OutputUtils::TextColor::SHINING);
				
				int index = 0;
				int GTHeadIndex = 0;

				while (std::getline(objectfile, line))//read the file line by line
				{					
					index++;

					std::string relativeProjectedPose3d;
					std::string projectedPose2d;
					std::string globalProjectedPose3d;
					std::string tumPose;

					Pose::PoseStruct p = Pose::GT::str2pose(line);
					
					if (!p.tracked)//invalid pose
					{
						relativeProjectedPose3d = projectedPose2d = globalProjectedPose3d = tumPose = Pose::invalidPose;
					}
					else
					{
						std::string constants = std::to_string(p.id) + "," + std::to_string(p.ts) + "," + std::to_string(p.frameNumber) + "," + std::to_string(p.tracked) + ",";
						int frameNum = p.frameNumber;
						double ts = p.ts;

						cv::Mat GTObjectPose = p.rotationMat;
						QuaternionUtils::Quat q_GTObjectPose = p.q;

						//remove this option?
						if (file_prefix != INTERPOLATED_FILE_PREFIX)
						{
							GTHeadIndex = getIndexOfNearestTS(ts, cameraGTPoses);
						}
						else
							for (; GTHeadIndex < cameraGTPoses.size() && ts != cameraGTPoses[GTHeadIndex].ts; GTHeadIndex++);

						if (GTHeadIndex == cameraGTPoses.size() || !cameraGTPoses[GTHeadIndex].tracked) //didn't find an equal timestamp, or the parallel head pose is invalid
						{
							relativeProjectedPose3d = Pose::invalidPose;
							projectedPose2d = Pose::invalidPose;
						}

						else
						{
							//GT Pose of the head element (camera) which is the matching pose to the current object pose
							cv::Mat cameraGTPose = cameraGTPoses[GTHeadIndex].rotationMat;
							QuaternionUtils::Quat q_cameraGTPose = cameraGTPoses[GTHeadIndex].q;

							//this is for relative projection
							//transformation matrix from the matching camera pose to the GT space origin.
							cv::Mat camera2GTSpace = camera_ghcMat.inv() * cameraGTPose.inv();
							QuaternionUtils::Quat q_camera2GTSpace = q_camera_ghcMat.inv() * q_cameraGTPose.inv();

							//transformation matrix from the GT space origin to the current object pose.
							cv::Mat GTSpace2Object = GTObjectPose * object_ghcMat;
							QuaternionUtils::Quat q_GTSpace2Object = q_GTObjectPose * q_object_ghcMat;

							//transformation matrix from the matching camera pose to the current object pose. 
							cv::Mat camera2Object = camera2GTSpace * GTSpace2Object;
							QuaternionUtils::Quat q_camera2Object = q_camera2GTSpace * q_GTSpace2Object;
							
							relativeProjectedPose3d += constants;
							relativeProjectedPose3d += Pose::mat2str(camera2Object); //convert final result of pose in device 3d space from mat to string
							relativeProjectedPose3d += "0," + q_camera2Object.toStr();


							//this is for global projection
							if (enableGlobal)
							{
								cv::Mat srcSpace2Object = cameraSrcReferencePose * camera_ghcMat.inv() * cameraGTReferencePose.inv() * GTSpace2Object;
								QuaternionUtils::Quat q_srcSpace2Object = q_cameraSrcReferencePose * q_camera_ghcMat.inv() * q_cameraGTReferencePose.inv() * q_GTSpace2Object;
								globalProjectedPose3d += constants;
								globalProjectedPose3d += Pose::mat2str(srcSpace2Object);
								globalProjectedPose3d += "0," + (q_srcSpace2Object).toStr();
								tumPose = FormatUtils::to_string_with_precision(p.ts / 1000) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_trans.m_x / 1000) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_trans.m_y / 1000) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_trans.m_z / 1000) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_quat.m_x) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_quat.m_y) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_quat.m_z) + " "
									+ FormatUtils::to_string_with_precision(q_srcSpace2Object.m_quat.m_w);
							}
							else
								globalProjectedPose3d = tumPose = Pose::invalidPose;

							if ((projection_type & Projection::PROJECTION_2D) != 0)//projection to 2d space is enabled (valid intrinsics parameters)
							{

								camera2Object.convertTo(camera2Object, CV_64F);
								GTObjectPose.convertTo(GTObjectPose, CV_64F);

								std::vector<cv::Point3d> points;
								cv::Point3f point;
								point.x = camera2Object.at<double>(cv::Point(3, 0));//x translation of projected point 3d
								point.y = camera2Object.at<double>(cv::Point(3, 1));//y translation of projected point 3d
								point.z = camera2Object.at<double>(cv::Point(3, 2));//z translation of projected point 3d
								points.push_back(point);

								std::vector<cv::Point2d> projectedPoints2d, jacobian;
								cv::Mat intrinsics = toCV(cameraIntrinsics);//convert device intrinsics from struct format to cv mat

								cv::projectPoints(points, cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F), intrinsics, cv::Mat::zeros(1, 5, CV_64F), projectedPoints2d); //project points to 2d (image) space

								if (stream == Projection::FE) //fit 2d projected points to distortion fisheye images
								{
									projectedPoints2d[0] = Projection::distortPoint(projectedPoints2d[0]);
									cv::Mat distorted;
									Projection::Fisheye::distortFisheyePoints(projectedPoints2d, distorted, cv::Size(640, 480), intrinsics, cameraIntrinsics.d_w, cv::Mat());
									cv::Vec2d v = distorted.at<cv::Vec2d>(0, 0);
									projectedPoints2d[0].x = v[0];
									projectedPoints2d[0].y = v[1];

								}

								if (OTPreviousMarkerFrameN != NOTYET && frameNum != OTPreviousMarkerFrameN) //this is not the first frame and this is a new frame. push all data of previos frame into vector
								{
									Projection::imageColorSpacePoints.insert(std::make_pair(OTPreviousMarkerTS, Projection::imagePoints_tmp));
									Projection::imagePoints_tmp.clear();
								}
								Projection::imagePoints_tmp.push_back(projectedPoints2d[0]);//push the 2d color image projected point into a temporary vector 

								//concatenate eye rotation matrix, x 2d, y 2d and z 3d to a string
								projectedPose2d += constants;
								projectedPose2d += std::string("1") + Pose::GT::pose_delimiter + std::string("0") + Pose::GT::pose_delimiter + std::string("0") + Pose::GT::pose_delimiter;;
								projectedPose2d += std::to_string(projectedPoints2d[0].x) + Pose::GT::pose_delimiter;
								projectedPose2d += std::string("0") + Pose::GT::pose_delimiter + std::string("1") + Pose::GT::pose_delimiter + std::string("0") + Pose::GT::pose_delimiter;
								projectedPose2d += std::to_string(projectedPoints2d[0].y) + Pose::GT::pose_delimiter;
								projectedPose2d += std::string("0") + Pose::GT::pose_delimiter + std::string("0") + Pose::GT::pose_delimiter + std::string("1") + Pose::GT::pose_delimiter;
								projectedPose2d += std::to_string(point.z) + Pose::GT::pose_delimiter;
								projectedPose2d += "-";

								OTPreviousMarkerFrameN = frameNum; //keeps the current frame number for the next time
								OTPreviousMarkerTS = ts; //keeps the current timestamp for the next time
							}
							if (GTHeadIndex > 0)
								GTHeadIndex--;
						}
					}

					if(relativeProjectedPose3d != Pose::invalidPose) outfileR3d << relativeProjectedPose3d << "\n";//print relative-projected 3d point to the file
					if(globalProjectedPose3d != Pose::invalidPose) outfileG3d << globalProjectedPose3d << "\n";//print global-projected 3d point to the file
					if (!(p.q == QuaternionUtils::Quat::zeros()) && tumPose != Pose::invalidPose)
						outfileG3dTUM << tumPose << "\n";
				}

				if (index == 0)
					OutputUtils::printColoredText("The file of: \"" + object_name + "\" is empty. No projection will be processed for this file", OutputUtils::WARNING_COLOR);

				objectfile.close();
				outfileR3d.close();
				outfileG3d.close();
				outfileG3dTUM.close();

				if (file == headFile)//for HMD DEV
				{
					std::ifstream  src(tumFile, std::ios::binary);
					std::ofstream  dst(folder + "\\" + rcName + ".rc.tum", std::ios::binary);

					dst << src.rdbuf();
				}

				std::cout << "Done!!!" << std::endl;

				//if projection 2d is enabled and an images folder is given by user or found in directory and user didn't cancel the dispaly option, the projected points will be shown on the aligned images
				if ((projection_type & Projection::PROJECTION_2D) != 0 && !imagesFolder.empty() && display)
				{
					std::map<int, double> ts;
					if (stream == Projection::COLOR)
					{
						ts = Projection::ReadDeviceTS(imagesFolder.substr(0, imagesFolder.find_last_of("\\")) + "\\color.txt");
					}
					else if (stream == Projection::FE)
					{
						ts = Projection::ReadDeviceTS(imagesFolder.substr(0, imagesFolder.find_last_of("\\")) + "\\rgb.txt");
					}
					else
					{
						OutputUtils::errorMessage("failed to read device timestamps", logger);
					}
					if (saveImages)
					{
						auto err = _mkdir((imagesFolder + "\\projected").c_str()) != 0;
					}
					if (!GT::Display::drawPoints(imagesFolder, Projection::imageColorSpacePoints, ts, saveImages, stream == Projection::FE ? true : false))
						OutputUtils::errorMessage("can not find images to loop", logger);
				}
			}//if (file == headFile && org_element || file != headFile && rel_element)	//while (FindNextFileA(hFind, &fd)); 
		} //for each (std::string file in files)
	}//if ((projection_type & Projection::PROJECTION_3D) != 0)
	return 0;
}//int main

void Projection::Fisheye::distortFisheyePoints(cv::InputArray undistorted, cv::OutputArray distorted, cv::Size imageSize, cv::InputArray K, float w, cv::InputArray R)
{
	CV_Assert(undistorted.type() == CV_32FC2 || undistorted.type() == CV_64FC2);
	CV_Assert(K.size() == cv::Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));
	CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);

	distorted.create(undistorted.size(), undistorted.type());

	cv::Vec2f f, c;
	if (K.depth() == CV_32F)
	{
		cv::Matx33f camMat = K.getMat();
		f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
		c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
	}
	else
	{
		cv::Matx33d camMat = K.getMat();
		f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
		c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
	}
	cv::Matx33d RR = cv::Matx33d::eye();
	if (!R.empty() && R.total() * R.channels() == 3)
	{
		cv::Vec3d rvec;
		R.getMat().convertTo(rvec, CV_64F);
		RR = cv::Affine3d(rvec).rotation();
	}
	else if (!R.empty() && R.size() == cv::Size(3, 3))
		R.getMat().convertTo(RR, CV_64F);

	const cv::Vec2f* srcf = undistorted.getMat().ptr<cv::Vec2f>();
	const cv::Vec2d* srcd = undistorted.getMat().ptr<cv::Vec2d>();
	cv::Vec2f* dstf = distorted.getMat().ptr<cv::Vec2f>();
	cv::Vec2d* dstd = distorted.getMat().ptr<cv::Vec2d>();

	for (size_t it = 0; it < undistorted.total(); it++)
	{

		cv::Vec2d pi = undistorted.depth() == CV_32F ? (cv::Vec2d)srcf[it] : srcd[it];

		const float u_out = (pi[0] - c[0]) / f[0];
		const float v_out = (pi[1] - c[1]) / f[1];

		//Compute the warp
		float distortionRatio;// = warpFunction(u_out, v_out);

		float ru = sqrt(u_out * u_out + v_out * v_out);
		distortionRatio = 1.f; // = ru / rd

		if (w * w > FOVModelEpsilon)
		{
			float interTerm = 2.0f * std::tan(w * 0.5f);
			if (ru * ru > FOVModelEpsilon)
			{
				distortionRatio = std::atan(interTerm * ru) / (w * ru);
			}
			else    //to limit the fov model as ru goes to 0
			{
				distortionRatio = interTerm / w;
			}
		}
		/* undistort
		if (w * w > m_FOVModelEpsilon)
		{
		float denominator = 2.0f * std::tan(w * 0.5f);
		if (rd * rd > m_FOVModelEpsilon)
		{
		distortionRatio = std::tan(w * rd) / (denominator * rd);
		}
		else            //to limit the fov model as ru goes to 0
		{
		distortionRatio = w / denominator;
		}
		}*/
		//Apply the warp factor
		const float  u_in = u_out * distortionRatio;
		const float  v_in = v_out * distortionRatio;

		//Apply input intrisic parameters
		const float x_in = f[0] * u_in + c[0];
		const float y_in = f[1] * v_in + c[1];


		cv::Vec2d fi(x_in, y_in);
		if (undistorted.depth() == CV_32F)
			dstf[it] = fi;
		else
			dstd[it] = fi;

		cv::Mat d = distorted.getMat();
	}
}



