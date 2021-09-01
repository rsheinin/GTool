
#include "PosData.h"

OTData::OTData()
{
	
}

OTData::~OTData()
{
}

std::string OTData::getFileExtension()
{
	return ".csv";
}

std::string OTData::dataToString()
{
	std::string str = "";
	str += std::to_string(m_id) + ",";
	str += std::to_string(m_timeStamp) + ",";
	str += std::to_string(m_frameNum) + ",";
	str += std::to_string(m_status) + ",";
	for (int i = 0; i < rotMatSize; i++)
	{
		for (int j = 0; j < rotMatSize; j++)
		{
			str += FormatUtils::to_string_with_precision(m_rotationsMat[i][j]) + ",";
		}
		str += FormatUtils::to_string_with_precision(m_transVector[i]) + ",";
	}
	str += std::to_string(m_error);
	for (int i = 0; i < 4; i++)
	{
		str += "," + FormatUtils::to_string_with_precision(m_quaternion[i]);
	}
	str += "\n";
	return str;
}

IData* OTData::stringToData(std::string str)
{
	OTData* pos = new OTData();
	std::vector<std::string> row = getNextLineAndSplitIntoTokens(str);
	if (row.size() != POSESIZE )
		return NULL;
	pos->m_id = parseObjectId(row[ID]);
	pos->m_timeStamp = std::stod(row[OTData::TIMESTAMP]);
	pos->m_frameNum = std::stoi(row[OTData::FRAMENUMBER]);
	pos->m_status = std::stoi(row[OTData::STATUS]);
	if (pos->m_status == OTData::MARKER)
		return NULL;
	if (pos->m_status == OTData::UNTRACKED)
	{
		pos->m_rotationsMat[0][0] = NAN;
		pos->m_rotationsMat[0][1] = NAN;
		pos->m_rotationsMat[0][2] = NAN;
		pos->m_rotationsMat[1][0] = NAN;
		pos->m_rotationsMat[1][1] = NAN;
		pos->m_rotationsMat[1][2] = NAN;
		pos->m_rotationsMat[2][0] = NAN;
		pos->m_rotationsMat[2][1] = NAN;
		pos->m_rotationsMat[2][2] = NAN;
		pos->m_transVector[0] = NAN;
		pos->m_transVector[1] = NAN;
		pos->m_transVector[2] = NAN;
		pos->m_error = NAN;
		pos->m_quaternion[0] = NAN;
		pos->m_quaternion[1] = NAN;
		pos->m_quaternion[2] = NAN;
		pos->m_quaternion[3] = NAN;
	}
	else
	{
		pos->m_rotationsMat[0][0] = std::stod(row[OTData::R0]);
		pos->m_rotationsMat[0][1] = std::stod(row[OTData::R1]);
		pos->m_rotationsMat[0][2] = std::stod(row[OTData::R2]);
		pos->m_rotationsMat[1][0] = std::stod(row[OTData::R4]);
		pos->m_rotationsMat[1][1] = std::stod(row[OTData::R5]);
		pos->m_rotationsMat[1][2] = std::stod(row[OTData::R6]);
		pos->m_rotationsMat[2][0] = std::stod(row[OTData::R8]);
		pos->m_rotationsMat[2][1] = std::stod(row[OTData::R9]);
		pos->m_rotationsMat[2][2] = std::stod(row[OTData::R10]);
		pos->m_transVector[0] = std::stod(row[OTData::TX]);
		pos->m_transVector[1] = std::stod(row[OTData::TY]);
		pos->m_transVector[2] = std::stod(row[OTData::TZ]);
		pos->m_error = std::stod(row[OTData::ERR]);
		pos->m_quaternion[0]= std::stod(row[OTData::QX]);
		pos->m_quaternion[1] = std::stod(row[OTData::QY]);
		pos->m_quaternion[2] = std::stod(row[OTData::QZ]);
		pos->m_quaternion[3] = std::stod(row[OTData::QW]);
	}
	return pos;
}

std::string OTData::getInnerDefPath(std::string folder, void *rb_name)
{
	std::string name = *((std::string*)rb_name);
	return folder + "\\" + name + getFileExtension();
}

std::vector<std::string> OTData::getNextLineAndSplitIntoTokens(std::string line, char delimiter)
{
	std::vector<std::string> result;

	std::stringstream lineStream(line);
	std::string cell;

	while (std::getline(lineStream, cell, delimiter))
	{
		result.push_back(cell);
	}
	return result;
}

IData * OTData::linearInterpolation(IData * next, double timeStamp, int src_id)
{
	OTData* nextObj = (OTData*)next;
	OTData* interData = new OTData();

	if (timeStamp == this->m_timeStamp || next->m_timeStamp == this->m_timeStamp)
		return this;

	double fraction = (timeStamp - this->m_timeStamp) / (next->m_timeStamp - this->m_timeStamp);

	interData->m_timeStamp = timeStamp;
	interData->m_frameNum =  linearInterpolate(this->m_frameNum, nextObj->m_frameNum, fraction);
	interData->m_id = src_id;
	interData->m_error = linearInterpolate(this->m_error, nextObj->m_error, fraction);
	interData->m_status = this->m_status && nextObj->m_status;//???
	if (!interData->m_status)
		return interData;

	interData->poseInterpolation(this->m_quaternion, ((OTData*)next)->m_quaternion, this->m_transVector, ((OTData*)next)->m_transVector, fraction);

	NN_Quat q;
	q.x = interData->m_quaternion[0];
	q.y = interData->m_quaternion[1];
	q.z = interData->m_quaternion[2];
	q.w = interData->m_quaternion[3];
	HMatrix M;
	Mat_FromQuat(q, M);
	for (size_t i = 0; i < rotMatSize; i++)
	{
		for (size_t j = 0; j < rotMatSize; j++)
			interData->m_rotationsMat[i][j] = M[i][j];
	}

	return interData;
}

double OTData::poseInterpolation(quaternion q_prev, quaternion q_next, translation t_prev, translation t_next, double t/*, quaternion & q_res, translation & t_res*/)
{
	QuaternionUtils::Quat q1(q_prev, t_prev);
	QuaternionUtils::Quat q2(q_next, t_next);

	QuaternionUtils::Quat q;
	
	double dot = q1.m_quat.dot(q2.m_quat);
	const double DOT_THRESHOLD = 0.9995;
	if (dot > DOT_THRESHOLD) 
	{
		// If the inputs are too close for comfort, linearly interpolate
		// and normalize the result.
		q.m_quat = q1.m_quat.linearInterpolation(q2.m_quat, t);
	}
	else
	{
		if (dot < 0.0f) 
		{
			q2.m_quat = -q2.m_quat;
			dot = -dot;
		}

		double theta_0 = acos(dot);  // theta_0 = angle between input vectors
		double theta = theta_0*t;    // theta = angle between v0 and result

		q.m_quat = q2.m_quat - q1.m_quat*dot;
		q.m_quat = q1.m_quat*cos(theta) + q.m_quat*sin(theta);
	}

	q.m_trans = q1.m_trans.linearInterpolation(q2.m_trans, t);

	m_quaternion = q.m_quat.axisData2quaternion();
	m_transVector = q.m_trans.axisData2translation();

	return 0.0;
}

std::string OTData::getName()
{
	return "optitrack";
}

bool OTData::hasTitles()
{
	return true;
}

int OTData::parseObjectId(std::string id)
{
	std::transform(id.begin(), id.end(), id.begin(), ::toupper);
	if (id == RIGHT_STR)
		return Hands::RIGHT;
	if (id == LEFT_STR)
		return Hands::LEFT;
	return std::stoi(id);
}