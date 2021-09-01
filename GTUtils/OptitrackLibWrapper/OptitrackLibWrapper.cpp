// This is the main DLL file.

#include "stdafx.h"
#include "OptitrackLibWrapper.h"

using namespace System::Runtime::InteropServices;

String^ OptitrackLibWrapper::OptitrackWrapper::optitrackInit(String^ RBName, String^ serverIP, String^ clientIP, String^ folderPath)
{
	msclr::interop::marshal_context context;

	std::string sRBName = (char*)Marshal::StringToHGlobalAnsi(RBName).ToPointer();
	//msclr::interop::marshal_context context;
	sRBName = context.marshal_as<std::string>(RBName);

	//std::string sserviceMode = (char*)Marshal::StringToHGlobalAnsi(serviceMode).ToPointer();
	////msclr::interop::marshal_context context1;
	//sserviceMode = context.marshal_as<std::string>(serviceMode);

	//std::string sMWName = (char*)Marshal::StringToHGlobalAnsi(MWName).ToPointer();
	////msclr::interop::marshal_context context2;
	//sMWName = context.marshal_as<std::string>(MWName);

	std::string sserverIP = (char*)Marshal::StringToHGlobalAnsi(serverIP).ToPointer();
	//msclr::interop::marshal_context context3;
	sserverIP = context.marshal_as<std::string>(serverIP);

	std::string sclientIP = (char*)Marshal::StringToHGlobalAnsi(clientIP).ToPointer();
	//msclr::interop::marshal_context context4;
	sclientIP = context.marshal_as<std::string>(clientIP);

	std::string spath = (char*)Marshal::StringToHGlobalAnsi(folderPath).ToPointer();
	//msclr::interop::marshal_context context4;
	spath = context.marshal_as<std::string>(folderPath);

	return gcnew System::String(OptitrackUtils::Service::optitrackInit(sRBName/*, sserviceMode, sMWName*/, sserverIP, sclientIP, spath).c_str());
}

bool OptitrackLibWrapper::OptitrackWrapper::optitrackShutdown()
{
	return OptitrackUtils::Service::optitrackShutdown();
}

bool OptitrackLibWrapper::OptitrackWrapper::optitrackInRange()
{
	return OptitrackUtils::Service::optitrakInrange();
}

bool OptitrackLibWrapper::OptitrackWrapper::optitrackGetPose(array<double>^ %pose, double %ts, unsigned int %frameNum)
{
	bool res;
	pin_ptr<double> _pose = &pose[0];
	double _ts = ts;
	int _frameNum = frameNum;
	/*double* f = p;
	double* ff = reinterpret_cast<double*>(f);*/
	res = OptitrackUtils::Service::getPose(_pose, _ts, _frameNum);
	ts = _ts;
	frameNum = _frameNum;
	return res;
}

void OptitrackLibWrapper::OptitrackWrapper::optitrackStartGetPoses()
{
	return OptitrackUtils::Service::optitrackStartGetPoses();
}

void OptitrackLibWrapper::OptitrackWrapper::optitrackStopGetPoses()
{
	return OptitrackUtils::Service::optitrackStopGetPoses();
}

//bool OptitrackLibWrapper::OptitrackWrapper::optitrackGetPoseFromBuffer(array<double>^ %poseMat, double %ts, float %qualityInd){
//	
//	if (OptitrackUtils::Service::cameraRBPosesBuffer.size() == 0)
//		return false;
//	auto pose = OptitrackUtils::Service::cameraRBPosesBuffer[0];
//	ts = std::get<0>(pose);
//	qualityInd = std::get<1>(pose);
//	auto rotation = std::get<2>(pose);
//	for (int i = 0; i < 12; i++)
//	{
//		poseMat[i] = rotation[i];
//	}
//	OptitrackUtils::Service::cameraRBPosesBuffer.erase(OptitrackUtils::Service::cameraRBPosesBuffer.begin());//eraze the first element which has been read already from buffer
//	if (OptitrackUtils::Service::cameraRBPosesBuffer.empty())
//		return false;
//	return true;
//}

//void OptitrackLibWrapper::OptitrackWrapper::optitrackGetPoseBufferbyIndex(array<double>^ %poseMat, double %ts, float %qualityInd, int index){
//
//	auto pose = OptitrackUtils::Service::cameraRBPosesBuffer[index];
//	ts = std::get<0>(pose);
//	qualityInd = std::get<1>(pose);
//	auto rotation = std::get<2>(pose);
//	for (int i = 0; i < 12; i++)
//	{
//		poseMat[i] = rotation[i];
//	}
//}

