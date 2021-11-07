#pragma once
#ifndef OPTITRACKUTILS_H
#define OPTITRACKUTILS_H

#include <tchar.h>
#include <winsock2.h>
#include <string>

#include <iostream>
#include <fstream>

//#include "NatNetTypes.h"
#include "NatNetClient.h"
//#include "NATUtils.h"

#include <tuple>
#include <functional>
#include <vector>
#include <map>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <conio.h>

namespace OptitrackUtils{

	namespace Service
	{
		//extern std::vector<std::tuple<double, float, double*/*std::array<double,12>*/ >> cameraRBPosesBuffer;
		std::string optitrackInit(std::string RBName, /*std::string serviceMode, std::string MWName, */std::string serverIP, std::string clientIP, std::string path);
		bool optitrackShutdown();
		bool getPose(double* pose, double &ts, int &frameNum);
		void optitrackStartGetPoses();
		void optitrackStopGetPoses();
		bool optitrakInrange();

		typedef enum ServiceMode
		{
			CAPTURE = 0,
			RECORD
		} ServiceMode;

		static std::string to_string(ServiceMode mode)
		{
			switch (mode)
			{
			case CAPTURE:
				return "calibration";
			case RECORD:
				return "record";
			}
			return "";
		}

	}

	namespace Data
	{
		typedef enum DataDescriptors
		{
			/*Descriptor_LabledMarker = -1,*/
			Descriptor_MarkerSet = 0,
			Descriptor_RigidBody,
			Descriptor_Skeleton,
			Descriptor_ForcePlate
		} DataDescriptors;

		typedef struct DescriptorStruct
		{
			DataDescriptors type;
			int id;
			std::string name;
			std::ofstream* file;
		} ObjectStruct;

		static std::string to_string(DataDescriptors descriptor)
		{
			switch (descriptor)
			{
			case Descriptor_MarkerSet:
				return "Marker";
			case Descriptor_RigidBody:
				return "RigidBody";
			case Descriptor_Skeleton:
				return "Skeleton";
			case Descriptor_ForcePlate:
				return "ForcePlate";
			/*case Descriptor_LabledMarker:
				return "LabledMarker";*/
			}

			return "";
		}

		typedef enum PoseStruct
		{
			R0 = 0,
			R1,
			R2,
			TX,
			R4,
			R5,
			R6,
			TY,
			R8,
			R9,
			R10,
			TZ,

			QX,
			QY,
			QZ,
			QW,

			EX,
			EY,
			EZ,

			NMARKERS_Blank,
			NMARKERS_OtherMarkers,
			NMARKERS_LabeledMarkers,
			NMARKERS_Rigid,

			ERR,

			RB_ID/* = 0*/,
			TIMESTAMP,
			FRAMENUMBER,
			ISTRACKED,




			POSESIZE
		} PoseStruct;

		typedef enum Status
		{
			UNTRACKED = 0,
			TRACKED,
			MARKER,
			EXCESS_ERROR
		} Status;

		extern std::vector<DescriptorStruct> objects;
		extern sFrameOfMocapData* lastFrame;
		extern int recordPosesNum;
		extern int analogSamplesPerMocapFrame;
		extern int frameNum;
		extern bool wasTrackedBefore;
		extern double first_ts;
		extern std::map<int, std::string> ObjectsNames;
		extern int cameraDome_id;
		extern bool record;

		void initVariables();
		void pose2ServiceFormat(sRigidBodyData myRB, double* pose);

		namespace Defaults
		{
			extern std::string cameraRBName;
			extern int serviceMode;
			extern std::string MW_name;
			extern std::string server_ip;
			extern std::string client_ip;
		}
	}

	namespace Connection
{
	extern NatNetClient* theClient;
	extern char szMyIPAddress[128];
	extern char szServerIPAddress[128];
	void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
	void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
	void resetClient();
	int CreateClient(int iConnectionType);
	bool getDataFromServer();
	bool SendTestRequest();
	extern unsigned int MyServersDataPort;
	extern unsigned int MyServersCommandPort;
	//int iConnectionType = ConnectionType_Multicast;
	extern int iConnectionType;
}

	namespace Output
	{
		bool initOutputFolder();
		extern std::string folder_path;
		std::ofstream* getFilePtrById(int id);
		std::ofstream* getFilePtrByType(int type);
		void printColoredText(std::string str, int color = 15);
		void writeTitles(std::ofstream* fp);
		bool writeFrame(std::ofstream* fp, double* pose, double ts, int frame_number);
		bool writeFrame(std::ofstream* fp, double* pose, double ts, int frame_number, int* nmarkers);
		extern std::ofstream* calibrationFile;
		extern HANDLE hConsole;
		extern FILE* fp;
	}

	// Yehonatan: namespace of std math is the same, so it's not such a good choice 
	namespace Math
	{
		typedef struct { float x, y, z, w; } Quat; /* Quaternion */
		enum QuatPart { X, Y, Z, W };
		typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */
		typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
		/* EulGetOrd unpacks all useful information about order simultaneously. */
		#define EulFrmR	     1
		#define EulFrm(ord)  ((unsigned)(ord)&1)
		#define EulRepNo     0		
		#define EulRepYes    1
		#define EulParOdd    1		
		#define EulSafe	     "\000\001\002\000"	
		#define EulNext	     "\001\002\000\001"
		#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
			n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
		/* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
		#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
		#define EulOrdXYZr    EulOrd(Math::Z,EulParOdd,EulRepNo,EulFrmR)

		extern double min_pitch;
		extern double max_pitch;
		extern double min_yaw;
		extern double max_yaw;
		extern double min_roll;
		extern double max_roll;

		// Yehonatan: missing details (size of array, resposibility to free memory. Better to use shared pointer
		double* calculateRotationMatrix(sRigidBodyData rigidBody);
		double getRoll(sRigidBodyData rigidBody);
		double getYaw(sRigidBodyData rigidBody);
		double getPitch(sRigidBodyData rigidBody);

		void Eul_ToHMatrix(EulerAngles ea, HMatrix M);
		EulerAngles Eul_FromHMatrix(HMatrix M, int order);
		EulerAngles Eul_FromQuat(Quat q, int order);
		void Mat_FromQuat(Quat q, int order, HMatrix M);

		static float RadiansToDegrees(float fRadians)
		{
			return fRadians * (180.0F / 3.14159265F);
		}
	}
}

#endif

