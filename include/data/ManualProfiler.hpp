#pragma once

#include <vector>
#include <cstdint>
#include <string>

class ManualProfiler
{
private:
	int currsection;
	std::vector<int64_t> timings;
	std::string ProfilerName;
	std::vector<std::string> names;
	int64_t lastswitchtick;
	int64_t lastprinttick;
public:
	ManualProfiler();

	ManualProfiler(std::string InProfilerName);

	ManualProfiler(std::string InProfilerName, std::vector<std::string> InNames);

	void EnterSection(int sectionnumber);
	void NameSection(int sectionnumber, std::string name);

	void operator+=(ManualProfiler& other);

	void PrintProfile();
	bool ShouldPrint();
	void PrintIfShould();
};
