#pragma once

#include <vector>
#include <cstdint>
#include <string>

class ManualProfiler
{
private:
	int currsection;
	std::vector<int64_t> timings;
	std::vector<std::string> names;
	int64_t lastswitchtick;
	int64_t lastprinttick;
public:
	ManualProfiler();

	void EnterSection(int sectionnumber);
	void NameSection(int sectionnumber, std::string name);

	void PrintProfile();
	bool ShouldPrint();
	void PrintIfShould();
};
