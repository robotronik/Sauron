#include "Overlord/BoardMemory.hpp"

#include <algorithm>

using namespace std;
using namespace Overlord;

vector<Object> BoardMemory::FindObjects(uint8_t TypeFilter)
{
	vector<Object> outvec;
	outvec.reserve(ObjectsOnBoard.size());
	for (const auto &obj : ObjectsOnBoard)
	{
		if ((uint8_t)obj.Type & TypeFilter)
		{
			outvec.push_back(obj);
		}
	}
	return outvec;
}

vector<Object> BoardMemory::FindObjectsSorted(uint8_t TypeFilter, double posX, double posY)
{
	vector<Object> outvec = FindObjects(TypeFilter);
	sort(outvec.begin(), outvec.end(), [posX, posY](Object a, Object b)
	{
		double deltaxa = a.PosX-posX, deltaya = a.PosY-posY;
		double deltaxb = b.PosX-posX, deltayb = b.PosY-posY;
		return deltaxa*deltaxa+deltaya*deltaya<deltaxb*deltaxb+deltayb*deltayb;
	});
	return outvec;
}