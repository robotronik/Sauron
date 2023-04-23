#include "Overlord/Overlord.hpp"


#ifdef WITH_SAURON
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
using namespace std;
using namespace cv;
#endif

void Overlord::Manager::Init()
{
	int numbots = 1;
	RobotControllers.resize(numbots);
	PhysicalRobotStates.resize(numbots);
	for (int i = 0; i < numbots; i++)
	{
		RobotHAL* &RC = RobotControllers[i];
		RC = new RobotHAL();
		RC->PositionLinear = LinearMovement(0.1, 0.2, 0.3, 0); //TODO: add real params 
		RC->ClawExtension = LinearMovement(0.1, 0.3, 1, 0);
		RC->ClawHeight = LinearMovement(0.1, 0.3, 1, 0);
		for (int j = 0; j < 3; j++)
		{
			RC->Trays[j] = LinearMovement(0.1, 0.3, 1, 0);
		}
		
	}
	
	/*
	Gâteaux: symétries % x et y 
	- roses: (725, 775)
	- jaunes: (925, 775)
	- marrons: (375, 725)

	Cerises:
		- 10 cerises / support, cerises: d2.8 cm
		- (1215+i*30, 0) for i in range (10) symétrie % y
		- (-165+i*30, 985) for i in range (10) symétire % x

	Plats :
		- verts : (-375, -775); (1275, -775); (1275, 275); (375, 775)
		- bleus : (375, -775); (1275, -275); (1275; 775); (-375; 775)
	*/
}

void Overlord::Manager::GatherData()
{

}

void Overlord::Manager::Run()
{

}

void Overlord::Manager::Thread()
{
	Init();
#ifdef WITH_SAURON
	Mat posplotdisc = Mat::zeros(Size(1000,800), CV_8UC3);
	Mat posplotcont = posplotdisc.clone();
	LinearMovement dut(0.01, 0.02, 2, 0.1, 540, 30);
	LinearMovement dutdisc = dut, dutcont = dut;
	for (int i = 10; i < 990; i++)
	{
		int time = i-10;
		{
			if (time == 200)
			{
				dutdisc.SetTarget(700);
			}
			
			double budget = 1;
			dutdisc.Tick(budget);
			double red = dutdisc.Speed/dutdisc.MaxSpeed;
			double blue = -red;
			red = red < 0 ? 0 : red;
			blue = blue < 0 ? 0 : blue;
			posplotdisc.at<Vec3b>((int)dutdisc.TargetPos, i) = Vec3b(255, 0, 255);
			posplotdisc.at<Vec3b>((int)dutdisc.Pos, i) = Vec3b(red*255, 255, blue*255);
			imshow("pos over time (discrete)", posplotdisc);
			cout << "Current speed : " << dutdisc.Speed << endl;
		}

		
		{
			dutcont = dut;
			bool hassecondtarget = time > 200;
			double budget1 = hassecondtarget ? 200:time;
			double budget2 = hassecondtarget ? time-200:0;
			dutcont.Tick(budget1);
			if (hassecondtarget)
			{
				dutcont.SetTarget(700);
				dutcont.Tick(budget2);
			}
			
			double red = dutcont.Speed/dutcont.MaxSpeed;
			double blue = -red;
			red = red < 0 ? 0 : red;
			blue = blue < 0 ? 0 : blue;
			posplotcont.at<Vec3b>((int)dutcont.TargetPos, i) = Vec3b(255, 0, 255);
			posplotcont.at<Vec3b>((int)dutcont.Pos, i) = Vec3b(red*255, 255, blue*255);
			imshow("pos over time (continuous)", posplotcont);
			cout << "Current speed : " << dutcont.Speed << endl;
		}
		waitKey(10);
	}
	
	
#endif
	while (true)
	{
		GatherData();
		Run();
		waitKey(10);
	}
}