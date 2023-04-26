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

vector<Object> BoardMemory::FindObjectsSorted(uint8_t TypeFilter, Vector2d<double> SearchPos)
{
	vector<Object> outvec = FindObjects(TypeFilter);
	sort(outvec.begin(), outvec.end(), [SearchPos](Object a, Object b)
	{
		return (SearchPos-a.position).lengthsquared()<(SearchPos-b.position).lengthsquared();
	});
	return outvec;
}