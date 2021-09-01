//#include "Interpolation.h"
//
//void Interpolation::interpolation(std::vector<double> algTimeStamps, std::vector<IData*> gtData)
//{
//	std::vector<IData*> interplatedData;
//	int gtIndx = 0;
//
//	for (int algIndx = 0; algIndx < algTimeStamps.size(); algIndx++)
//	{
//		int prevIndx = findPrevTimeStamp(algTimeStamps[algIndx], gtData, gtIndx);
//		//if there is a gt data for the same time stamp
//		if (algTimeStamps[algIndx] == gtData[prevIndx]->m_timeStamp)
//		{
//			interplatedData.push_back(gtData[prevIndx]);		
//		}		
//		else
//		{
//			int nextIndx = prevIndx + 1;
//			IData* interData=gtData[prevIndx]->interpolateObject(gtData[nextIndx], algTimeStamps[algIndx]);
//			interplatedData.push_back(interData);
//		}
//		gtIndx = prevIndx <= 100 ? gtIndx : prevIndx - 100;
//	}
//
//}
//
//int Interpolation::findPrevTimeStamp(double algTimeStamp, std::vector<IData*>& gtData, int gtStartIndx)
//{
//	int indx = gtStartIndx;
//
//	while (indx < gtData.size() && algTimeStamp < (gtData[indx]->m_timeStamp))
//	{
//		indx++;
//	}
//
//	//if it's the same time stamp like the alg
//	if (indx != gtData.size() && algTimeStamp == (gtData[indx]->m_timeStamp))
//		return indx;
//
//	//else- if the index is not in the begin or end of the vector
//	if (indx < gtData.size() - 1 && indx>0)//if we are not in the first gt timestamp or the last one
//	{
//		return indx - 1;
//	}
//
//	return -1;
//}
//
//
