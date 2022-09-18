#include "data/DataPacket.hpp"
#include <sstream>
#include <cmath>

using namespace std;

string PositionPacket::ToCSV()
{
	ostringstream fab; 
	fab << (int)type << ", " << (int)numeral
	<< ", " << X << ", " << Y << ", " << rotation*180/M_PI;
	return fab.str();
}