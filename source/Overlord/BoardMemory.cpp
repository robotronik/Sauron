#include "Overlord/BoardMemory.hpp"

#include <algorithm>

using namespace std;
using namespace Overlord;

vector<Object> Overlord::FindObjects(const std::vector<Object> &in, uint32_t TypeFilter)
{
	vector<Object> outvec;
	outvec.reserve(in.size());
	for (const auto &obj : in)
	{
		if ((uint8_t)obj.Type & TypeFilter)
		{
			outvec.push_back(obj);
		}
	}
	return outvec;
}

vector<Object> Overlord::FindObjectsSorted(const std::vector<Object> &in, uint32_t TypeFilter, Vector2d<double> SearchPos)
{
	vector<Object> outvec = FindObjects(in, TypeFilter);
	sort(outvec.begin(), outvec.end(), [SearchPos](Object a, Object b)
	{
		return (SearchPos-a.position).lengthsquared()<(SearchPos-b.position).lengthsquared();
	});
	return outvec;
}