// OptitrackLibWrapper.h
#include <msclr\marshal_cppstd.h>


#pragma once

using namespace System;

namespace OptitrackLibWrapper {

	public ref class OptitrackWrapper
	{
	public:
		static bool init = false;
		//static bool optitrackInit();
		static String^ optitrackInit(String^ RBName, /*String^ serviceMode, String^ MWName, */String^ serverIP, String^ clientIP, String^ folderPath);
		static bool optitrackShutdown();

		static bool optitrackInRange();
		static bool optitrackGetPose(array<double>^ %pose, double %ts, unsigned int %frameNum);
		static void optitrackStartGetPoses();
		static void optitrackStopGetPoses();
		//static bool optitrackGetPoseFromBuffer(array<double>^ %poseMat, double %ts, float %qualityInd);
		//static inline int optitrackGetBufferSize(){ return OptitrackUtils::Service::cameraRBPosesBuffer.size(); }
		//static void optitrackGetPoseBufferbyIndex(array<double>^ %poseMat, double %ts,  float %qualityInd, int index);
	};
}
