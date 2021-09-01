// Yehonatan: who owns this file? some basic change log. basic description
#pragma warning( disable : 4996 )
#include "OptitrakUtils.h"
#include <time.h>
#include <direct.h>


#define POSES_NUM 11
#define MAX_POSES_NUM POSES_NUM*2
#define MIN_POSES_NUM 5
#define INVALID_ID -1
#define TS_UNITS 1000
#define SP_POSE_SIZE 12
#define DEFAULT_COLOR 15
#define ERR_THRESHOLD 0.005

using namespace OptitrackUtils;

//Service 
//std::vector < std::tuple<double, float, double*>> Service::cameraRBPosesBuffer;
const std::string rubbish = "e";

bool Service::optitrakInrange()
{
	for (size_t i = 0; i < Data::lastFrame->nRigidBodies; i++)
	{
		if (Data::lastFrame->RigidBodies[i].ID == Data::cameraDome_id)
			return Data::lastFrame->RigidBodies[i].params & 0x01 ? true : false;
	}
	return false;
}

std::string Service::optitrackInit(std::string RBName, std::string serverIP, std::string clientIP, std::string path)
{
	Data::initVariables();

	Output::folder_path = path;

	//read given parameters
	if (RBName != rubbish)
		Data::Defaults::cameraRBName = RBName;
	//connection establishing 
	if (serverIP.c_str() != rubbish)
		Data::Defaults::server_ip = serverIP;
	strcpy(Connection::szServerIPAddress, Data::Defaults::server_ip.c_str());
	printf("Connecting to server at %s...\n", Connection::szServerIPAddress);
	if (clientIP.c_str() != rubbish)
		Data::Defaults::client_ip = clientIP;
	strcpy(Connection::szMyIPAddress, Data::Defaults::client_ip.c_str());
	printf("Connecting from %s...\n", Connection::szMyIPAddress);


	// Create NatNet Client
	int iResult = Connection::CreateClient(Connection::iConnectionType);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");
		return "ERROR";
	}
	else
	{
		printf("Client initialized and ready.\n");
	}
	
	//if (!SendTestRequest())
	//	return false;

	if (!Connection::getDataFromServer())
		return "ERROR";

	printf("\nClient is connected to server and listening for data...\n");

	Output::printColoredText("camera rigid body for tracking is: " + Data::Defaults::cameraRBName, 18);

	return OptitrackUtils::Output::folder_path + "\\";
}

bool Service::optitrackShutdown()
{
	std::cout << "\nShutting down opti-track client\nBye!\n";

	/*if (Data::lastFrame != NULL)
		Connection::theClient->FreeFrame(Data::lastFrame);*/
	// Done - clean up.

	int res = Connection::theClient->Uninitialize();

	for (Data::DescriptorStruct s : Data::objects)
	{
		s.file->close();
		//free(s.file);
	}

	//return res;
	return true;
}

bool Service::getPose(double* pose, double &ts, int &frameNum)
{
	bool tracked = false;
	sFrameOfMocapData* myFrame = new sFrameOfMocapData();

	std::vector<double*>rb_poses;
	/*for (size_t i = 0; i < MAX_RB; i++)
	{
		rb_poses.clear();
	}*/

	double* c_pose = new double[Data::PoseStruct::POSESIZE]();

	Output::printColoredText("Most Recent Frame from OT:", 13);
	std::cout << "number: " << Data::lastFrame->iFrame << std::endl << "timestamp: " << Data::lastFrame->fTimestamp << std::endl;
	
	int poses_num = POSES_NUM;
	for (size_t pose_num = 0; pose_num < poses_num && pose_num < MAX_POSES_NUM; pose_num++)           //loop on wanted number of poses (for getting the median of all of them)
	{
		Connection::theClient->CopyFrame(Data::lastFrame, myFrame);

		for (size_t rb_num = 0; rb_num < myFrame->nRigidBodies; rb_num++) //loop on all rigidbodies in frame, look for the camera rb and get its poses
		{
			if (myFrame->RigidBodies[rb_num].ID == Data::cameraDome_id)   //this is the camera rb
			{
				int markersNum = myFrame->RigidBodies[rb_num].nMarkers;
				/*std::cout << "----------------------------" << std::endl;
				std::cout << "rb markers num: " << markersNum << std::endl;
				for (size_t i = 0; i < markersNum; i++)
				{
					std::cout << "marker num: " << i << std::endl;
					std::cout << "id: " << myFrame->RigidBodies[rb_num].MarkerIDs[i] << std::endl;
					std::cout << "size: " << myFrame->RigidBodies[rb_num].MarkerSizes[i] << std::endl;
					std::cout << "translation: " << myFrame->RigidBodies[rb_num].Markers[i][0] << "," << myFrame->RigidBodies[rb_num].Markers[i][1] << "," << myFrame->RigidBodies[rb_num].Markers[i][2] << "," << std::endl;
				}
				std::cout << "error: " << myFrame->RigidBodies[rb_num].MeanError << std::endl;
				std::cout << "----------------------------" << std::endl;*/

				c_pose[Data::PoseStruct::RB_ID] = myFrame->RigidBodies[rb_num].ID;
				c_pose[Data::PoseStruct::ERR] = myFrame->RigidBodies[rb_num].MeanError;
				c_pose[Data::PoseStruct::TIMESTAMP] = myFrame->fTimestamp;
				c_pose[Data::PoseStruct::FRAMENUMBER] = myFrame->iFrame;
				
				//update tracking status 
				if ((myFrame->RigidBodies[rb_num].params & 0x01) == 0)							//untracked rigid body
					c_pose[Data::PoseStruct::ISTRACKED] = Data::Status::UNTRACKED;
				else if(c_pose[Data::PoseStruct::ERR] > ERR_THRESHOLD)							//mean error is too big
					c_pose[Data::PoseStruct::ISTRACKED] = Data::Status::EXCESS_ERROR;
				else                                                                            //rigid-body is tracked + error is in allowed limits. pose will be considered as "tracked"
					c_pose[Data::PoseStruct::ISTRACKED] = Data::Status::TRACKED;

				if (c_pose[Data::PoseStruct::ISTRACKED] == Data::Status::TRACKED)
				{
					Data::pose2ServiceFormat(myFrame->RigidBodies[rb_num], c_pose);
					rb_poses.push_back(c_pose);
				}
				else
					poses_num++;	//increase the num of taken poses till max value in order to find tracked poses
			}
		}
	}

	if (rb_poses.size() >= MIN_POSES_NUM) //found at least MIN_POSES_NUM (5) tracked poses. can get the median pose
	{
		std::sort(rb_poses.begin(), rb_poses.end(), [](const double r[], const double l[]) {return r[Data::PoseStruct::TX] > l[Data::PoseStruct::TX]; }); //sotr RB poses according to translation x.
		double* median_pose = rb_poses.at(poses_num / 2); //median pose from sorted vector of poses

		Output::printColoredText("Rigid Body # " + std::to_string(median_pose[Data::PoseStruct::RB_ID]) + " :" + Data::ObjectsNames[median_pose[Data::PoseStruct::RB_ID]], 30);

		std::cout << "Tracking Pose from OT : "
			<< median_pose[Data::PoseStruct::R0] << " " << median_pose[Data::PoseStruct::R1] << " " << median_pose[Data::PoseStruct::R2] << " " << median_pose[Data::PoseStruct::TX] << " "
			<< median_pose[Data::PoseStruct::R4] << " " << median_pose[Data::PoseStruct::R5] << " " << median_pose[Data::PoseStruct::R6] << " " << median_pose[Data::PoseStruct::TY] << " "
			<< median_pose[Data::PoseStruct::R8] << " " << median_pose[Data::PoseStruct::R9] << " " << median_pose[Data::PoseStruct::R10] << " " << median_pose[Data::PoseStruct::TZ] << std::endl;

		/*std::cout << "OT quaternion: " << median_pose[Data::PoseStruct::QX] << " " << median_pose[Data::PoseStruct::QY] << " " << median_pose[Data::PoseStruct::QZ] << " " << median_pose[Data::PoseStruct::QW] << std::endl;
		std::cout << "X(Pitch), Y(Yaw), Z(Roll): " << median_pose[Data::PoseStruct::EX] << " " << median_pose[Data::PoseStruct::EY] << " " << median_pose[Data::PoseStruct::EZ] << std::endl;*/

		if (median_pose[Data::PoseStruct::EX] < Math::min_pitch)
			Math::min_pitch = median_pose[Data::PoseStruct::EX];
		if (median_pose[Data::PoseStruct::EX] > Math::max_pitch)
			Math::max_pitch = median_pose[Data::PoseStruct::EX];

		if (median_pose[Data::PoseStruct::EY] < Math::min_yaw)
			Math::min_yaw = median_pose[Data::PoseStruct::EY];
		if (median_pose[Data::PoseStruct::EY] > Math::max_yaw)
			Math::max_yaw = median_pose[Data::PoseStruct::EY];

		if (median_pose[Data::PoseStruct::EZ] < Math::min_roll)
			Math::min_roll = median_pose[Data::PoseStruct::EZ];
		if (median_pose[Data::PoseStruct::EZ] > Math::max_roll)
			Math::max_roll = median_pose[Data::PoseStruct::EZ];

		std::cout << "pitch-- max: " << Math::max_pitch << " min:" << Math::min_pitch << " yaw-- max: " << Math::max_yaw << " min: " << Math::min_yaw << " roll-- max: " << Math::max_roll << " min: " << Math::min_roll << std::endl;

		for (size_t j = 0; j < 12; j++)
		{
			pose[j] = median_pose[j];
		}

		ts = median_pose[Data::PoseStruct::TIMESTAMP];
		frameNum = median_pose[Data::PoseStruct::FRAMENUMBER]; //to check what happens in case of frame drop

		return true;
	}

	if (c_pose[Data::PoseStruct::ISTRACKED] == Data::Status::EXCESS_ERROR)
		Output::printColoredText("WARNING: pose mean error is bigger than the allowed value. rb will be considered as untracked", 24);

	/*for (size_t j = 0; j < 12; j++)
	{
		pose[j] = c_pose[j];
	}*/

	return false;
}
// in order to supprt the quality indicator in calibration service as well:
/*bool Service::getPose(double* pose, double &ts, int &frameNum, float &qualityInd)
{
	bool tracked = false;
	sFrameOfMocapData* myFrame = new sFrameOfMocapData();

	std::vector<double*>rb_poses(MAX_RB);
	for (size_t i = 0; i < 5; i++)
	{
		rb_poses.clear();
	}

	double* c_pose = new double[Data::PoseStruct::POSESIZE]();

	Output::printColoredText("Most Recent Frame from OT:", 13);
	std::cout << "number: " << Data::lastFrame->iFrame << std::endl << "timestamp: " << Data::lastFrame->fTimestamp << std::endl;

	for (size_t pose_num = 0; pose_num < POSES_NUM; pose_num++) //loop on wanted number of poses (for getting the median of all of them)
	{
		Connection::theClient->CopyFrame(Data::lastFrame, myFrame);

		for (size_t rb_num = 0; rb_num < myFrame->nRigidBodies; rb_num++) //loop on all rigidbodies in frame, look for camera rb and get its poses
		{
			if (myFrame->RigidBodies[rb_num].ID == Data::cameraDome_id) //this is the camera rb
			{
				c_pose[Data::PoseStruct::RB_ID] = myFrame->RigidBodies[rb_num].ID;
				c_pose[Data::PoseStruct::ERR] = myFrame->RigidBodies[rb_num].MeanError;
				c_pose[Data::PoseStruct::TIMESTAMP] = myFrame->fTimestamp;
				c_pose[Data::PoseStruct::FRAMENUMBER] = myFrame->iFrame;
				c_pose[Data::PoseStruct::ISTRACKED] = myFrame->RigidBodies[rb_num].params & 0x01 != 0 ? Data::Status::TRACKED : Data::Status::UNTRACKED;
				Data::pose2ServiceFormat(myFrame->RigidBodies[rb_num], c_pose);
				rb_poses.push_back(c_pose);
			}
		}
	}

	if (rb_poses.size() > 0)
	{
		std::sort(rb_poses.begin(), rb_poses.end(), [](const double r[], const double l[]) {return r[Data::PoseStruct::TX] > l[Data::PoseStruct::TX]; }); //sotr RB poses according to translation x.
		double* median_pose = rb_poses.at(POSES_NUM / 2); //median pose from sorted vector of poses

		Output::printColoredText("Rigid Body # " + std::to_string(median_pose[Data::PoseStruct::RB_ID]) + " :" + Data::ObjectsNames[median_pose[Data::PoseStruct::RB_ID]], 30);

		std::cout << "Tracking Pose from OT : "
			<< median_pose[Data::PoseStruct::R0] << " " << median_pose[Data::PoseStruct::R1] << " " << median_pose[Data::PoseStruct::R2] << " " << median_pose[Data::PoseStruct::TX] << " "
			<< median_pose[Data::PoseStruct::R4] << " " << median_pose[Data::PoseStruct::R5] << " " << median_pose[Data::PoseStruct::R6] << " " << median_pose[Data::PoseStruct::TY] << " "
			<< median_pose[Data::PoseStruct::R8] << " " << median_pose[Data::PoseStruct::R9] << " " << median_pose[Data::PoseStruct::R10] << " " << median_pose[Data::PoseStruct::TZ] << std::endl;

		//std::cout << "OT quaternion: " << median_pose[Data::PoseStruct::QX] << " " << median_pose[Data::PoseStruct::QY] << " " << median_pose[Data::PoseStruct::QZ] << " " << median_pose[Data::PoseStruct::QW] << std::endl;
		//std::cout << "X(Pitch), Y(Yaw), Z(Roll): " << median_pose[Data::PoseStruct::EX] << " " << median_pose[Data::PoseStruct::EY] << " " << median_pose[Data::PoseStruct::EZ] << std::endl;

		if (median_pose[Data::PoseStruct::ISTRACKED] == Data::Status::TRACKED)
			tracked = true;

		if (median_pose[Data::PoseStruct::EX] < Math::min_pitch)
			Math::min_pitch = median_pose[Data::PoseStruct::EX];
		if (median_pose[Data::PoseStruct::EX] > Math::max_pitch)
			Math::max_pitch = median_pose[Data::PoseStruct::EX];

		if (median_pose[Data::PoseStruct::EY] < Math::min_yaw)
			Math::min_yaw = median_pose[Data::PoseStruct::EY];
		if (median_pose[Data::PoseStruct::EY] > Math::max_yaw)
			Math::max_yaw = median_pose[Data::PoseStruct::EY];

		if (median_pose[Data::PoseStruct::EZ] < Math::min_roll)
			Math::min_roll = median_pose[Data::PoseStruct::EZ];
		if (median_pose[Data::PoseStruct::EZ] > Math::max_roll)
			Math::max_roll = median_pose[Data::PoseStruct::EZ];

		std::cout << "pitch-- max: " << Math::max_pitch << " min:" << Math::min_pitch << " yaw-- max: " << Math::max_yaw << " min: " << Math::min_yaw << " roll-- max: " << Math::max_roll << " min: " << Math::min_roll << std::endl;

		for (size_t j = 0; j < 12; j++)
		{
			pose[j] = median_pose[j];
		}

		ts = median_pose[Data::PoseStruct::TIMESTAMP];
		frameNum = median_pose[Data::PoseStruct::FRAMENUMBER]; //to check what happens in case of frame drop
		qualityInd = median_pose[Data::PoseStruct::ERR]; 
	}

	return tracked;
}*/
void Service::optitrackStartGetPoses()
{
	if (Data::Defaults::serviceMode == Service::RECORD)
	{
		Data::recordPosesNum = 0;
		Data::wasTrackedBefore = false;
		std::string current_objects = "";

		for each (Data::DescriptorStruct obj in Data::objects)
		{
			std::string fileStatus = obj.file->is_open() ? "open" : "not open";
			current_objects += obj.name + " of type: " + to_string(obj.type) + " with id: " + std::to_string(obj.id) + ". file is: " + fileStatus + "\n";
		}

		Output::printColoredText("Start record\nRecording poses for: " + current_objects, 8);
		Data::record = true;
	}
}

void Service::optitrackStopGetPoses() {
	//std::cout << "Stop record" << std::endl;
	Output::printColoredText("Stop record", 8);
	Data::record = false;
}

//Data
int Data::cameraDome_id;
bool Data::record;
std::vector<Data::DescriptorStruct> Data::objects;
sFrameOfMocapData* Data::lastFrame;
int Data::recordPosesNum;
int Data::analogSamplesPerMocapFrame;
int Data::frameNum;
bool Data::wasTrackedBefore;
double Data::first_ts;
std::map<int, std::string> Data::ObjectsNames;

std::string Data::Defaults::cameraRBName = "9bot_Dome";
int Data::Defaults::serviceMode = Service::ServiceMode::RECORD;
std::string Data::Defaults::MW_name = "sp";
std::string Data::Defaults::server_ip = "10.12.144.144";
std::string Data::Defaults::client_ip = "";


void Data::initVariables()
{
	//data-structures
	ObjectsNames.clear();
	objects.clear();
	//Service::cameraRBPosesBuffer.clear();

	cameraDome_id = -1;
	analogSamplesPerMocapFrame = 0;
	lastFrame = new sFrameOfMocapData();

	//boolean flags
	record = false;

	//counter
	frameNum = 0;

	//output
	char szMyIPAddress[128] = "";
	char szServerIPAddress[128] = "";
	Output::hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	std::cout.precision(12);
}

//convert quaternion, given from Opti-track, to rotation matrix via Euler angles 
void Data::pose2ServiceFormat(sRigidBodyData myRB, double* pose) {
	Math::Quat q;
	Math::HMatrix M;
	Math::EulerAngles ea;
	int order = EulOrdXYZr;

	q.x = myRB.qx;
	q.y = myRB.qy;
	q.z = myRB.qz;
	q.w = myRB.qw;

	Mat_FromQuat(q, order, M);
	
	//std::cout << "Rotation matrix:\n";
	for (int i = 0; i <= 3; i++)
	{
		for (int j = 0; j <= 3; j++)
		{
			pose[i * 4 + j] = M[i][j];
		}
	}

	// Yehonatan: commented out code...
	//convert Euler angles from radians to degrees
	/*ea.x = NATUtils::RadiansToDegrees(ea.x);
	ea.y = NATUtils::RadiansToDegrees(ea.y);
	ea.z = NATUtils::RadiansToDegrees(ea.z);*/

	// Yehonatan: what is this *1000 about? (move to mm?)
	pose[TX] = myRB.x * 1000;
	pose[TY] = myRB.y * 1000;
	pose[TZ] = myRB.z * 1000;

	pose[QX] = myRB.qx;
	pose[QY] = myRB.qy;
	pose[QZ] = myRB.qz;
	pose[QW] = myRB.qw;

	/*pose[EX] = Math::RadiansToDegrees(ea.x);
	pose[EY] = Math::RadiansToDegrees(ea.y);
	pose[EZ] = Math::RadiansToDegrees(ea.z);*/

}

//Connection
NatNetClient* Connection::theClient;
char Connection::szMyIPAddress[128];
char Connection::szServerIPAddress[128];
unsigned int Connection::MyServersDataPort = 3130;
unsigned int Connection::MyServersCommandPort = 3131;
int Connection::iConnectionType = ConnectionType_Unicast;

void Connection::resetClient()
{
	int iSuccess;

	std::cout << "\n\nre-setting Client\n\n.";

	iSuccess = theClient->Uninitialize();
	if (iSuccess != 0)
		std::cout << "error un-initting Client\n";

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if (iSuccess != 0)
		std::cout << "error re-initting Client\n";
}

// Establish a NatNet Client connection
int Connection::CreateClient(int iConnectionType)
{
	// release previous server
	if (theClient)
	{
		theClient->Uninitialize();
		delete theClient;
	}

	// create NatNet client
	theClient = new NatNetClient(iConnectionType);

	// set the callback handlers
	theClient->SetVerbosityLevel(Verbosity_Warning);
	theClient->SetMessageCallback(MessageHandler);
	theClient->SetDataCallback(DataHandler, theClient);	// this function will receive data from the server
	// [optional] use old multicast group
	//theClient->SetMulticastAddress("224.0.0.1");

	// print version info
	unsigned char ver[4];
	theClient->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Init Client and connect to NatNet server
	// to use NatNet default port assignments
	int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	// to use a different port for commands and/or data:
	//int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return ErrorCode_Internal;
	}
	else
	{
		// get # of analog samples per mocap frame of data
		void* pResult;
		int ret = 0;
		int nBytes = 0;
		ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
		if (ret == ErrorCode_OK)
		{
			Data::analogSamplesPerMocapFrame = *((int*)pResult);
			printf("Analog Samples Per Mocap Frame : %d", Data::analogSamplesPerMocapFrame);
		}

		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		theClient->GetServerDescription(&ServerDescription);
		if (!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		printf("[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", szMyIPAddress);
		printf("Server IP:%s\n", szServerIPAddress);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;

}

// Yehonatan: doc..
bool Connection::SendTestRequest()
{
	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	int iResult;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
		return true;
	}
	return false;
}

// Yehonatan: doc..
bool Connection::getDataFromServer()
{
	void* response;
	int nBytes;
	int iResult;

	//bool rigidBodyExist = false;

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if (!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
		return false;
	}
	else
	{
		printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);

		for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
		{
			printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);

			Data::DescriptorStruct obj;
			obj.type = (Data::DataDescriptors)pDataDefs->arrDataDescriptions[i].type;

			if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
			{
				// MarkerSet
				sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
				printf("MarkerSet Name : %s\n", pMS->szName);

				if (false)
				{
					obj.id = INVALID_ID;
					obj.name = std::string(pMS->szName);
					obj.file = new std::ofstream(Output::folder_path + "\\" + obj.name/* + "_" + to_string(obj.type)*/ + ".csv");
					Data::objects.push_back(obj);
				}

				for (int i = 0; i < pMS->nMarkers; i++)
					printf("%s\n", pMS->szMarkerNames[i]);

			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
			{
				// RigidBody
				//rigidBodyExist = true;
				sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;

				if (std::strcmp(Data::Defaults::cameraRBName.c_str(), pRB->szName) == 0)
					Data::cameraDome_id = pRB->ID;

				Data::ObjectsNames.insert(std::make_pair(pRB->ID, pRB->szName));

				if (Data::Defaults::serviceMode == Service::RECORD)
				{
					obj.id = pRB->ID;
					obj.name = std::string(pRB->szName);
					obj.file = new std::ofstream(Output::folder_path + "\\" + obj.name/* + "_" + to_string(obj.type)*/ + ".csv");
					Output::writeTitles(obj.file);
					Data::objects.push_back(obj);
				}

				printf("RigidBody Name : %s\n", pRB->szName);
				printf("RigidBody ID : %d\n", pRB->ID);
				printf("RigidBody Parent ID : %d\n", pRB->parentID);
				printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
			{
				// Skeleton
				sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
				printf("Skeleton Name : %s\n", pSK->szName);
				printf("Skeleton ID : %d\n", pSK->skeletonID);
				printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);

				Data::ObjectsNames.insert(std::make_pair(pSK->skeletonID, pSK->szName));

				if (Data::Defaults::serviceMode == Service::RECORD)
				{
					obj.id = pSK->skeletonID;
					obj.name = std::string(pSK->szName);
					obj.file = new std::ofstream(Output::folder_path + "\\" + obj.name/* + "_" + to_string(obj.type)*/ + ".csv");
					Output::writeTitles(obj.file);
					Data::objects.push_back(obj);
				}

				for (int j = 0; j < pSK->nRigidBodies; j++)
				{
					sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
					printf("  RigidBody Name : %s\n", pRB->szName);
					printf("  RigidBody ID : %d\n", pRB->ID);
					printf("  RigidBody Parent ID : %d\n", pRB->parentID);
					printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
				}
			}
			else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
			{
				// Force Plate
				sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
				printf("Force Plate ID : %d\n", pFP->ID);
				printf("Force Plate Serial : %s\n", pFP->strSerialNo);
				printf("Force Plate Width : %3.2f\n", pFP->fWidth);
				printf("Force Plate Length : %3.2f\n", pFP->fLength);
				printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX, pFP->fOriginY, pFP->fOriginZ);

				if (Data::Defaults::serviceMode == Service::RECORD)
				{
					obj.id = pFP->ID;
					obj.name = std::string(pFP->strSerialNo);
					obj.file = new std::ofstream(Output::folder_path + "\\" + obj.name/* + "_" + to_string(obj.type)*/ + ".csv");
					Output::writeTitles(obj.file);
					Data::objects.push_back(obj);
				}

				for (int iCorner = 0; iCorner < 4; iCorner++)
				{
					printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0], pFP->fCorners[iCorner][1], pFP->fCorners[iCorner][2]);
				}
				printf("Force Plate Type : %d\n", pFP->iPlateType);
				printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
				printf("Force Plate Channel Count : %d\n", pFP->nChannels);
				for (int iChannel = 0; iChannel < pFP->nChannels; iChannel++)
				{
					printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);

				}
			}
			else
			{
				printf("Unknown data type.");
				// Unknown
			}
		}
	}

	return true;
}

// DataHandler receives data from the server
void __cdecl Connection::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*)pUserData;

	pClient->CopyFrame(data, Data::lastFrame);
	//Output::printColoredText("Data::lastFrame->RigidBodies[0].translation: " + std::to_string(Data::lastFrame->RigidBodies[0].x) + ", " + std::to_string(Data::lastFrame->RigidBodies[0].y) + ", " + std::to_string(Data::lastFrame->RigidBodies[0].z), 13);

	if (Data::record)
	{
		/*double* camera_pose = new double[SP_POSE_SIZE];
		std::fill_n(camera_pose, SP_POSE_SIZE, NAN);
		*/
		int pose_size = Data::PoseStruct::POSESIZE;

		Data::recordPosesNum++;
		int i;
		double ts = Data::lastFrame->fTimestamp * TS_UNITS;
		int frameN = Data::lastFrame->iFrame;

		if (Data::recordPosesNum == 1)//first frame
			Data::first_ts = ts;

		if (Data::lastFrame->nRigidBodies > 0) //there is at least 1 rb defined in current frame
		{
			//std::cout << "found " << lastFrame->nRigidBodies << " RB" << std::endl;
			double* rb_pose = new double[pose_size];
			std::fill_n(rb_pose, pose_size, NAN);
			rb_pose[Data::PoseStruct::ISTRACKED] = Data::Status::UNTRACKED;

			for (i = 0; i < Data::lastFrame->nRigidBodies; i++)//loop over all exist rb in current frame
			{
				rb_pose[Data::PoseStruct::RB_ID] = Data::lastFrame->RigidBodies[i].ID;		//update object id
				std::string rb_name(Data::ObjectsNames[rb_pose[Data::PoseStruct::RB_ID]]);	//update object name
				rb_pose[Data::PoseStruct::ERR] = Data::lastFrame->RigidBodies[i].MeanError;	//update object mean error
				bool bTrackingValid = (Data::lastFrame->RigidBodies[i].params & 0x01) != 0 ? true : false;

				//std::cout << "in " << rb_name << " RB" << std::endl;

				if (bTrackingValid)//this rb is tracked
				{
					rb_pose[Data::PoseStruct::ISTRACKED] = Data::Status::TRACKED;
					Data::pose2ServiceFormat(Data::lastFrame->RigidBodies[i], rb_pose);
				}

				//std::cout << "before writing line to " << rb_name << " RB file" << std::endl;
				if (!Output::writeFrame(Output::getFilePtrById(rb_pose[Data::PoseStruct::RB_ID]), rb_pose, ts, frameN))
				{
					std::cout << "can not write to file of rb " << rb_name << std::endl;
				}

				//if (/*bTrackingValid*/true)
				//{
				//	for (int iMarker = 0; iMarker < Data::lastFrame->RigidBodies[i].nMarkers; iMarker++)//loop on all markers in current rb
				//	{
				//		double* rb_marker_pose = new double[pose_size];
				//		std::fill_n(rb_marker_pose, pose_size, 0);

				//		rb_marker_pose[Data::PoseStruct::RB_ID] = Data::lastFrame->RigidBodies[i].MarkerIDs[iMarker];

				//		rb_marker_pose[Data::PoseStruct::ISTRACKED] = Data::Status::MARKER;

				//		rb_marker_pose[Data::PoseStruct::R0] = 1;	//rotation of a marker is the identity matrix
				//		rb_marker_pose[Data::PoseStruct::R5] = 1;	//rotation of a marker is the identity matrix
				//		rb_marker_pose[Data::PoseStruct::R10] = 1;	//rotation of a marker is the identity matrix

				//		rb_marker_pose[Data::PoseStruct::TX] = Data::lastFrame->RigidBodies[i].Markers[iMarker][0] * 1000;
				//		rb_marker_pose[Data::PoseStruct::TY] = Data::lastFrame->RigidBodies[i].Markers[iMarker][1] * 1000;
				//		rb_marker_pose[Data::PoseStruct::TZ] = Data::lastFrame->RigidBodies[i].Markers[iMarker][2] * 1000;

				//		Output::writeFrame(Output::getFilePtrById(rb_pose[Data::PoseStruct::RB_ID]), rb_marker_pose, ts, frameN);
				//	}
				//}

				if (std::strcmp(Data::Defaults::cameraRBName.c_str(), rb_name.c_str()) == 0) //special for camera RB 
				{

					/*for (int i = 0; i < SP_POSE_SIZE; i++)
					{
						camera_pose[i] = rb_pose[i];
					}*/
					if (bTrackingValid)							//tracked
					{
						if (!Data::wasTrackedBefore || Data::recordPosesNum == 1)//was un-tracked before or this is the first frame
						{
							Output::printColoredText("TRACKED!!!", 10);
							Data::wasTrackedBefore = true;
						}
					}
					else                                         //untracked
					{
						if (Data::wasTrackedBefore || Data::recordPosesNum == 1)//was tracked before or this is the first frame
						{
							Output::printColoredText("UN-TRACKED :(", 12);
							Data::wasTrackedBefore = false;
						}
					}

					//float meanErr = rb_pose[Data::PoseStruct::ERR]; //get the error of the first available rigidbody
					//auto fullpose = std::make_tuple((ts - Data::first_ts), meanErr > 0 ? meanErr : 0, camera_pose);
					//Service::cameraRBPosesBuffer.push_back(fullpose);
				}
			}
		}

		if (Data::lastFrame->nSkeletons > 0)
		{
			for (i = 0; i < Data::lastFrame->nSkeletons; i++)
			{
				//...
			}
		}
	}
}

// MessageHandler receives NatNet error/debug messages
void __cdecl Connection::MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

//Output
HANDLE Output::hConsole;
FILE* Output::fp;
std::string Output::folder_path;

bool Output::initOutputFolder()
{
	//create output folder + file for each rigid-body

	time_t time_v = time(0);
	struct tm now;
	localtime_s(&now, &time_v);
	std::string date = std::to_string(now.tm_mday) + "_" + std::to_string(now.tm_mon + 1) + "_" + std::to_string(now.tm_year + 1900) + "_" + std::to_string(now.tm_hour) + "_" + std::to_string(now.tm_min) + "_" + std::to_string(now.tm_sec);
	printColoredText("service mode is: " + Service::to_string((Service::ServiceMode)Data::Defaults::serviceMode), 11);
	if (Data::Defaults::serviceMode == Service::ServiceMode::RECORD)
		folder_path = "..\\..\\..\\..\\output\\OptitrackResults\\PosesFiles\\";
	else if (Data::Defaults::serviceMode == Service::ServiceMode::CAPTURE)
		folder_path = "..\\..\\..\\..\\output\\Calibration\\";
	folder_path += date;
	folder_path += "_";
	folder_path += Service::to_string(Service::ServiceMode(Data::Defaults::serviceMode));

	char absolute_path[350];
	_fullpath(absolute_path, folder_path.c_str(), 350);
	std::string absolute_path_str(absolute_path);
	folder_path = absolute_path_str;

	if (_mkdir(folder_path.c_str()) == 0)
	{
		printColoredText("Saves files into " + folder_path, 10);
		return true;
	}
	printColoredText("Can not find folder: " + folder_path + "\nfiles will not be saved! ", 12);
	return false;
}

std::ofstream* Output::getFilePtrById(int id)
{
	//std::cout << "in getFilePtrById. id is: " << id << std::endl;
	if (Data::objects.size() == 0)
	{
		std::cout << "no objects in vector" << std::endl;
		return NULL;
	}
	for (Data::DescriptorStruct obj : Data::objects)
	{
		//std::cout << "in objects loop. objects vector is of size " << objects.size() << " object id is: " << obj.id << std::endl;

		if (obj.id == id)
		{
			/*std::cout << "name of object is:";
			std::cout << obj.name;
			std::cout << "type of object is:";
			std::cout << obj.type;
			std::cout << "status of file of object is:";
			std::cout << obj.file->is_open();
			std::cout << "found file pointer. name:  " << obj.name << "type: " << obj.type << "is open: "<< obj.file->is_open() << std::endl;*/
			return obj.file;
		}
	}
	return NULL;
}

//return the first file pointer to an object struct with given type
std::ofstream* Output::getFilePtrByType(int type)
{
	//std::cout << "in getFilePtrByType. type is: " << type << std::endl;
	if (Data::objects.size() == 0)
	{
		std::cout << "no objects in vector" << std::endl;
		return NULL;
	}

	for (Data::DescriptorStruct obj : Data::objects)
	{
		//std::cout << "in objects loop. objects vector is of size " << objects.size() << " object id is: " << obj.id << std::endl;

		if (obj.type == type)
		{
			/*std::cout << "name of object is:";
			std::cout << obj.name;
			std::cout << "type of object is:";
			std::cout << obj.type;
			std::cout << "status of file of object is:";
			std::cout << obj.file->is_open();
			std::cout << "found file pointer. name:  " << obj.name << "type: " << obj.type << "is open: " << obj.file->is_open() << std::endl;*/
			return obj.file;
		}
	}
	return NULL;
}

void Output::printColoredText(std::string str, int color)
{
	SetConsoleTextAttribute(hConsole, color);
	std::cout << str << std::endl;
	SetConsoleTextAttribute(hConsole, DEFAULT_COLOR);
}

void Output::writeTitles(std::ofstream* fp)
{
	*fp << "object_id,timestamp,frame_number,tracked,r_0,r_1,r_2,x,r_3,r_4,r_5,y,r_6,r_7,r_8,z,error,qx,qy,qz,qw\n"/*,pitch,yaw,roll\n*/;
}

bool Output::writeFrame(std::ofstream* fp, double* pose, double ts, int frame_muber)
{
	if (fp == NULL || pose == NULL)
		return false;

	//std::cout << "in writeFrame function" << std::endl;

	std::cout.precision(12);
	*fp << std::fixed << pose[Data::PoseStruct::RB_ID] << ",";
	*fp << ts << ",";
	//std::cout.precision(0);
	*fp << frame_muber << ",";
	*fp << pose[Data::PoseStruct::ISTRACKED] << ",";

	//rotation matrix
	for (int i = Data::PoseStruct::R0; i <= Data::PoseStruct::TZ; i++)
	{
		*fp << pose[i] << ",";
	}

	//mean error
	*fp << pose[Data::PoseStruct::ERR];

	//quaternion
	for (int i = Data::PoseStruct::QX; i <= Data::PoseStruct::QW; i++)
	{
	*fp << "," << pose[i];
	}

	//Euler angles
	/*for (int i = EX; i <= EZ; i++)
	{
	*fp << pose[i] << " ";
	}*/

	*fp << std::endl;

	return true;
}

//Math
double Math::min_pitch = 360;
double Math::max_pitch = -360;
double Math::min_yaw = 360;
double Math::max_yaw = -360;
double Math::min_roll = 360;
double Math::max_roll = -360;

// Yehonatan: missing details (size of array, resposibility to free memory. Better to use shared pointer
double* Math::calculateRotationMatrix(sRigidBodyData rigidBody)
{
	double* rotation = new double[9];
	rotation[0] = 1 - 2 * rigidBody.qy*rigidBody.qy - 2 * rigidBody.qz*rigidBody.qz;
	rotation[1] = 2 * rigidBody.qx*rigidBody.qy - 2 * rigidBody.qz*rigidBody.qw;
	rotation[2] = 2 * rigidBody.qx*rigidBody.qz + 2 * rigidBody.qy*rigidBody.qw;

	rotation[3] = 2 * rigidBody.qx*rigidBody.qy + 2 * rigidBody.qz*rigidBody.qw;
	rotation[4] = 1 - 2 * rigidBody.qx*rigidBody.qx - 2 * rigidBody.qz*rigidBody.qz;
	rotation[5] = 2 * rigidBody.qy*rigidBody.qz - 2 * rigidBody.qx*rigidBody.qw;

	rotation[6] = 2 * rigidBody.qx*rigidBody.qz - 2 * rigidBody.qy*rigidBody.qw;
	rotation[7] = 2 * rigidBody.qy*rigidBody.qz + 2 * rigidBody.qx*rigidBody.qw;
	rotation[8] = 1 - 2 * rigidBody.qx*rigidBody.qx - 2 * rigidBody.qy*rigidBody.qy;

	return rotation;
}

double Math::getRoll(sRigidBodyData rigidBody)
{
	double roll = atan2(2 * rigidBody.qy*rigidBody.qw - 2 * rigidBody.qx*rigidBody.qz, 1 - 2 * rigidBody.qy*rigidBody.qy - 2 * rigidBody.qz*rigidBody.qz);
	//conversion-from-radians-to-degrees
	roll = roll * 3.141592653589793 / 180.0; 
	
	return roll;
}

double Math::getYaw(sRigidBodyData rigidBody)
{
	double yaw = asin(2 * rigidBody.qx*rigidBody.qy + 2 * rigidBody.qz*rigidBody.qw);
	//conversion-from-radians-to-degrees
	yaw = yaw * 3.141592653589793 / 180.0;
	return yaw;
}

double Math::getPitch(sRigidBodyData rigidBody)
{
	double pitch = atan2(2 * rigidBody.qx*rigidBody.qw - 2 * rigidBody.qy*rigidBody.qz, 1 - 2 * rigidBody.qx*rigidBody.qx - 2 * rigidBody.qz*rigidBody.qz);
	//conversion-from-radians-to-degrees
	pitch = pitch * 3.141592653589793 / 180.0;
	return pitch;
}

/* Construct matrix from Euler angles (in radians). */
void Math::Eul_ToHMatrix(EulerAngles ea, HMatrix M)
{
	double ti, tj, th, ci, cj, ch, si, sj, sh, cc, cs, sc, ss;
	int i, j, k, h, n, s, f;
	EulGetOrd(ea.w, i, j, k, h, n, s, f);
	if (f == EulFrmR) { double t = ea.x; ea.x = ea.z; ea.z = t; }
	if (n == EulParOdd) { ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z; }
	ti = ea.x;	  tj = ea.y;	th = ea.z;
	ci = cos(ti); cj = cos(tj); ch = cos(th);
	si = sin(ti); sj = sin(tj); sh = sin(th);
	cc = ci*ch; cs = ci*sh; sc = si*ch; ss = si*sh;
	if (s == EulRepYes) {
		M[i][i] = cj;	  M[i][j] = sj*si;    M[i][k] = sj*ci;
		M[j][i] = sj*sh;  M[j][j] = -cj*ss + cc; M[j][k] = -cj*cs - sc;
		M[k][i] = -sj*ch; M[k][j] = cj*sc + cs; M[k][k] = cj*cc - ss;
	}
	else {
		M[i][i] = cj*ch; M[i][j] = sj*sc - cs; M[i][k] = sj*cc + ss;
		M[j][i] = cj*sh; M[j][j] = sj*ss + cc; M[j][k] = sj*cs - sc;
		M[k][i] = -sj;	 M[k][j] = cj*si;    M[k][k] = cj*ci;
	}
	M[W][X] = M[W][Y] = M[W][Z] = M[X][W] = M[Y][W] = M[Z][W] = 0.0; M[W][W] = 1.0;
}

///* Convert matrix to Euler angles (in radians). */
Math::EulerAngles Math::Eul_FromHMatrix(HMatrix M, int order)
{
	EulerAngles ea;
	int i, j, k, h, n, s, f;
	EulGetOrd(order, i, j, k, h, n, s, f);
	if (s == EulRepYes) {
		double sy = sqrt(M[i][j] * M[i][j] + M[i][k] * M[i][k]);
		if (sy > 16 * FLT_EPSILON) {
			ea.x = atan2((double)M[i][j], (double)M[i][k]);
			ea.y = atan2(sy, (double)M[i][i]);
			ea.z = atan2(M[j][i], -M[k][i]);
		}
		else {
			ea.x = atan2(-M[j][k], M[j][j]);
			ea.y = atan2(sy, (double)M[i][i]);
			ea.z = 0;
		}
	}
	else {
		double cy = sqrt(M[i][i] * M[i][i] + M[j][i] * M[j][i]);
		if (cy > 16 * FLT_EPSILON) {
			ea.x = atan2(M[k][j], M[k][k]);
			ea.y = atan2((double)-M[k][i], cy);
			ea.z = atan2(M[j][i], M[i][i]);
		}
		else {
			ea.x = atan2(-M[j][k], M[j][j]);
			ea.y = atan2((double)-M[k][i], cy);
			ea.z = 0;
		}
	}
	if (n == EulParOdd) { ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z; }
	if (f == EulFrmR) { double t = ea.x; ea.x = ea.z; ea.z = t; }
	ea.w = order;
	return (ea);
}

///* Convert quaternion to Euler angles (in radians). */
Math::EulerAngles Math::Eul_FromQuat(Quat q, int order)
{
	HMatrix M;
	double Nq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
	double xs = q.x*s, ys = q.y*s, zs = q.z*s;
	double wx = q.w*xs, wy = q.w*ys, wz = q.w*zs;
	double xx = q.x*xs, xy = q.x*ys, xz = q.x*zs;
	double yy = q.y*ys, yz = q.y*zs, zz = q.z*zs;
	M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
	M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
	M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
	M[W][X] = M[W][Y] = M[W][Z] = M[X][W] = M[Y][W] = M[Z][W] = 0.0; M[W][W] = 1.0;
	return (Eul_FromHMatrix(M, order));
}

void Math::Mat_FromQuat(Quat q, int order, HMatrix M)
{
	//HMatrix M;
	double Nq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
	double xs = q.x*s, ys = q.y*s, zs = q.z*s;
	double wx = q.w*xs, wy = q.w*ys, wz = q.w*zs;
	double xx = q.x*xs, xy = q.x*ys, xz = q.x*zs;
	double yy = q.y*ys, yz = q.y*zs, zz = q.z*zs;
	M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
	M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
	M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
	M[W][X] = M[W][Y] = M[W][Z] = M[X][W] = M[Y][W] = M[Z][W] = 0.0; M[W][W] = 1.0;
}


