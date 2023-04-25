#include "Overlord/Overlord.hpp"

#include "Overlord/Objectives/GatherCherries.hpp"

#ifdef WITH_SAURON
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <map>
#include "math3d.hpp"
using namespace std;
using namespace cv;
#endif

using namespace Overlord;

void Manager::Init()
{
	int numbots = 1;
	TimeLeft = 100;
	RobotControllers.resize(numbots);
	PhysicalRobotStates.resize(numbots);
	for (int i = 0; i < numbots; i++)
	{
		RobotHAL* &RC = RobotControllers[i];
		RC = new RobotHAL();
		RC->PositionLinear = LinearMovement(0.1, 0.2, 0.1, 0); //TODO: add real params 
		RC->Rotation = LinearMovement(1, 1, 1, 0.1);
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

	PhysicalBoardState.ObjectsOnBoard.clear();

	static const vector<Object> defaultcakes = {Object(ObjectType::CakePink, 0.725, 0.775),
	Object(ObjectType::CakeYellow, 0.925, 0.775),
	Object(ObjectType::CakeBrown, 0.375, 0.725)};

	for (int i = 0; i < 4; i++) //mirrors
	{
		double mirrorx = i & 1 ? -1:1;
		double mirrory = i & 2 ? -1:1;
		for (int j = 0; j < defaultcakes.size(); j++)
		{
			Object cake = defaultcakes[j];
			cake.PosX*=mirrorx;
			cake.PosY*=mirrory;
			for (int k = 0; k < 3; k++)
			{
				PhysicalBoardState.ObjectsOnBoard.push_back(cake);
			}
		}
	}

	for (int i = 0; i < 2; i++)
	{
		double mirror = i&1 ? -1:1;
		for (int j = 0; j < 10; j++)
		{
			Object cherryx(ObjectType::Cherry, 1.215+j*0.03, 0);
			Object cherryy(ObjectType::Cherry, -0.135+j*0.03, 0.985);
			cherryx.PosX *= mirror;
			cherryy.PosY *= mirror;
			PhysicalBoardState.ObjectsOnBoard.push_back(cherryx);
			PhysicalBoardState.ObjectsOnBoard.push_back(cherryy);
		}
	}

	Objectives.clear();
	Objectives.push_back(make_unique<GatherCherriesObjective>());
}

void Manager::GatherData()
{

}

void Manager::Run()
{
	int numrobots = RobotControllers.size();
	assert(numrobots ==1); //todo: make the following code work for multiple robots. Should be numobjective^robots in complexity tho if we try everything :/
	vector<BaseObjective*> BestObjective(numrobots, nullptr);
	vector<double> BestObjectiveScore(numrobots, 0);

	//Copy the state of the board, and simulate every objective to pick the one that makes the most points
	BoardMemory SimulationBoardState;
	std::vector<RobotMemory> SimulationRobotStates;
	std::vector<RobotHAL> SimulatedControllers;
	SimulatedControllers.resize(numrobots);
	for (auto &objective : Objectives)
	{
		SimulationBoardState = PhysicalBoardState;
		SimulationRobotStates = PhysicalRobotStates;
		for (int ciddx = 0; ciddx < RobotControllers.size(); ciddx++)
		{
			SimulatedControllers[ciddx] = RobotHAL(RobotControllers[ciddx]);
		}
		int robotidx =  0;
		double timebudget = TimeLeft;
		double score = objective->ExecuteObjective(timebudget, &SimulatedControllers[robotidx], &SimulationBoardState, &SimulationRobotStates[robotidx]);
		cout << "Objective gave score " << score << endl;
		if (score > BestObjectiveScore[robotidx])
		{
			BestObjectiveScore[robotidx] = score;
			BestObjective[robotidx] = objective.get();
		}
	}

	//Objectives selected : execute them for real
	for (int robotidx = 0; robotidx < numrobots; robotidx++)
	{
		if (BestObjective[robotidx] == nullptr)
		{
			cerr << "Robot " << robotidx << " did not find a suitable objective" << endl;
			continue;
		}
		double timebudget = 1.0/50; //todo : this should be the cycle time
		BestObjective[robotidx]->ExecuteObjective(timebudget, RobotControllers[robotidx], &PhysicalBoardState, &PhysicalRobotStates[robotidx]);
	}
}

void Manager::Display()
{
#ifdef WITH_SAURON
	vector<ObjectData> dataconcat;
	dataconcat.emplace_back(ObjectIdentity(PacketType::ReferenceAbsolute, 0), 0, 0, 0);

	static const map<ObjectType, PacketType> boardgltypemap = {
		{ObjectType::Unknown, PacketType::Null},
		{ObjectType::Robot, PacketType::Robot},
		{ObjectType::Cherry, PacketType::TopTracker}, //todo : actual mesh for cherries
		{ObjectType::CakeBrown, PacketType::Puck},
		{ObjectType::CakeYellow, PacketType::Puck},
		{ObjectType::CakePink, PacketType::Puck}
	};
	for (const auto &object : PhysicalBoardState.ObjectsOnBoard)
	{
		PacketType type = boardgltypemap.at(object.Type);
		dataconcat.emplace_back(ObjectIdentity(type, 0), object.PosX, object.PosY, 0.01);
	}
	for (int robotidx = 0; robotidx < RobotControllers.size(); robotidx++)
	{
		const auto& robothandle = RobotControllers[robotidx];
		const auto& robotmem = PhysicalRobotStates[robotidx];
		Affine3d robotloc;
		cout << "Robot " << robotidx << " speed is " << robothandle->PositionLinear.Speed << endl;
		robotloc.translation(Vec3d(robothandle->posX, robothandle->posY, 0.0));
		Vec3d xvec(0,0,0), yvec(0,0,0);
		robothandle->GetForwardVector(xvec[0], xvec[1]);
		yvec[1] = xvec[0];
		yvec[0] = -xvec[1];
		robotloc.rotation(MakeRotationFromXY(xvec, yvec));
		dataconcat.emplace_back(ObjectIdentity(PacketType::Robot, 0), robotloc);
		for (int trayidx = 0; trayidx < sizeof(robotmem.CakeTrays) / sizeof(robotmem.CakeTrays[0]); trayidx++)
		{
			const auto& tray = robotmem.CakeTrays[trayidx];
			for (const auto &object : tray)
			{
				PacketType type = boardgltypemap.at(object.Type);
				dataconcat.emplace_back(ObjectIdentity(type, 0), object.PosX, object.PosY, trayidx*0.2+0.1);
			}
		}
	}
	
	
	visualiser.Tick(dataconcat);
#endif
}

void Manager::Thread()
{
	Init();
#ifdef WITH_SAURON
	visualiser.Start();
	/*Mat posplotdisc = Mat::zeros(Size(1000,800), CV_8UC3);
	Mat posplotcont = posplotdisc.clone();
	LinearMovement dut(1, 2, 2, 0.1, 5.40, 0.30);
	LinearMovement dutdisc = dut, dutcont = dut;
	for (int i = 10; i < 990; i++)
	{
		int time = i-10;
		{
			if (time == 200)
			{
				dutdisc.SetTarget(dutdisc.Pos);
			}
			
			double budget = 0.01;
			dutdisc.Tick(budget);
			double red = dutdisc.Speed/dutdisc.MaxSpeed;
			double blue = -red;
			red = red < 0 ? 0 : red;
			blue = blue < 0 ? 0 : blue;
			posplotdisc.at<Vec3b>((int)(dutdisc.TargetPos*100), i) = Vec3b(255, 0, 255);
			posplotdisc.at<Vec3b>((int)(dutdisc.Pos*100), i) = Vec3b(red*255, 255, blue*255);
			imshow("pos over time (discrete)", posplotdisc);
			//cout << "Current speed : " << dutdisc.Speed << endl;
		}

		
		{
			dutcont = dut;
			bool hassecondtarget = time > 200;
			double budget1 = hassecondtarget ? 200:time;
			budget1/=100;
			double budget2 = hassecondtarget ? time-200:0;
			budget2/=100;
			dutcont.Tick(budget1);
			if (hassecondtarget)
			{
				dutcont.SetTarget(dutcont.Pos);
				dutcont.Tick(budget2);
			}
			
			double red = dutcont.Speed/dutcont.MaxSpeed;
			double blue = -red;
			red = red < 0 ? 0 : red;
			blue = blue < 0 ? 0 : blue;
			posplotcont.at<Vec3b>((int)(dutcont.TargetPos*100), i) = Vec3b(255, 0, 255);
			posplotcont.at<Vec3b>((int)(dutcont.Pos*100), i) = Vec3b(red*255, 255, blue*255);
			imshow("pos over time (continuous)", posplotcont);
			//cout << "Current speed : " << dutcont.Speed << endl;
		}
		waitKey(10);
	}*/
#endif
	while (true)
	{
		GatherData();
		Run();
		Display();
		waitKey(1000/50);
	}
}