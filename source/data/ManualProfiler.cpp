#include "data/ManualProfiler.hpp"

#include <opencv2/core.hpp>
#include <iostream>

using namespace std;


ManualProfiler::ManualProfiler()
	:currsection(-1), ProfilerName("")
{
	lastprinttick = cv::getTickCount();
}

ManualProfiler::ManualProfiler(std::string InProfilerName)
	:currsection(-1),
	ProfilerName(InProfilerName)
{
	lastprinttick = cv::getTickCount();
}

ManualProfiler::ManualProfiler(std::string InProfilerName, std::vector<std::string> InNames)
	:currsection(-1),
	ProfilerName(InProfilerName),
	names(InNames)
{
	lastprinttick = cv::getTickCount();
}

void ManualProfiler::EnterSection(int sectionnumber)
{
	int64_t entertick = cv::getTickCount();
	if (currsection != -1)
	{
		if (timings.size() <= currsection)
		{
			timings.resize(currsection +1, 0);
		}
		timings[currsection] += entertick - lastswitchtick;
	}
	currsection = sectionnumber;
	lastswitchtick = cv::getTickCount();
}

void ManualProfiler::NameSection(int sectionnumber, string name)
{
	if (names.size() <= sectionnumber)
	{
		names.resize(sectionnumber +1, string("No Name"));
	}
	names[sectionnumber] = name;
}

void ManualProfiler::operator+=(ManualProfiler& other)
{
	if (other.timings.size() > timings.size())
	{
		timings.resize(other.timings.size(), 0);
	}
	
	for (int i = 0; i < other.timings.size(); i++)
	{
		timings[i] += other.timings[i];
	}
	
}

void ManualProfiler::PrintProfile()
{
	cout << "Profiling " << ProfilerName <<":" << endl;
	int64_t total = 0;
	for (int i = 0; i < timings.size(); i++)
	{
		total += timings[i];
	}
	for (int i = 0; i < timings.size(); i++)
	{
		string name = "No Name";
		if (i < names.size())
		{
			name = names[i];
		}
		cout << "\tSection " << i << " \"" << name << "\" took " << (double)timings[i] / cv::getTickFrequency() << "s, " << (double)timings[i]/total*100.0 << "%" << endl;
	}
	lastprinttick = cv::getTickCount();
}

bool ManualProfiler::ShouldPrint()
{
	return (cv::getTickCount() - lastprinttick) > cv::getTickFrequency() *10;
}

void ManualProfiler::PrintIfShould()
{
	if (ShouldPrint())
	{
		PrintProfile();
	}
}
