#include "Overlord/Overlord.hpp"

#include <utility>

#include "Overlord/Objectives/GatherCherries.hpp"
#include "Overlord/Objectives/DepositCherries.hpp"
#include "Overlord/Objectives/TakeStack.hpp"
#include "Overlord/Objectives/TrayRoutine.hpp"
#include "Overlord/Objectives/RetractTray.hpp"
#include "Overlord/Objectives/RetractClaws.hpp"
#include "Overlord/RobotHandle.hpp"
#include "thirdparty/serialib.h"
#ifdef WITH_SAURON
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <map>
#include <tuple>
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
		serialib* bridge = new serialib();
		bridge->openDevice("/dev/COM", 115200);
		//RC = new RobotHandle(bridge);
		RC = new RobotHAL();
		RC->PositionLinear = LinearMovement(0.1, 0.2, 0.1, 0); //TODO: add real params 
		RC->Rotation = LinearMovement(1, 1, 1, 0.1);
		RC->ClawExtension = LinearMovement(1, 3, 3, 0);
		RC->ClawHeight = LinearMovement(.1, .3, 1, 0);
		for (int j = 0; j < 3; j++)
		{
			RC->Trays[j] = LinearMovement(1, 3, 3, 0);
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

	PhysicalBoardState.clear();
	static const vector<Object> greendropzones = {
		Object(ObjectType::GreenDropZone, {-0.375,-0.775}),
		Object(ObjectType::GreenDropZone, {0.375,0.775}),
		Object(ObjectType::GreenDropZone, {1.275,-0.775}),
		Object(ObjectType::GreenDropZone, {1.275,0.275}),
		Object(ObjectType::GreenDropZone, {-1.275,0.775})
	};

	for (int i = 0; i < greendropzones.size(); i++)
	{
		Object dropzone = greendropzones[i];
		PhysicalBoardState.push_back(dropzone);
		dropzone.Type = ObjectType::BlueDropZone;
		dropzone.position.y *=-1;
		PhysicalBoardState.push_back(dropzone);
	}
	


	static const vector<Object> defaultcakes = {Object(ObjectType::CakeYellow, {0.725, 0.775}),
	Object(ObjectType::CakePink, {0.925, 0.775}),
	Object(ObjectType::CakeBrown, {0.375, 0.275})};

	for (int i = 0; i < 4; i++) //mirrors
	{
		double mirrorx = i & 1 ? -1:1;
		double mirrory = i & 2 ? -1:1;
		for (int j = 0; j < defaultcakes.size(); j++)
		{
			Object cake = defaultcakes[j];
			cake.position*=Vector2dd(mirrorx, mirrory);
			for (int k = 0; k < 3; k++)
			{
				PhysicalBoardState.push_back(cake);
			}
		}
	}

	for (int i = 0; i < 2; i++)
	{
		double mirror = i&1 ? -1:1;
		for (int j = 0; j < 10; j++)
		{
			Object cherryx(ObjectType::Cherry, {1.215+j*0.03, 0});
			Object cherryy(ObjectType::Cherry, {-0.135+j*0.03, 0.985});
			cherryx.position.x *= mirror;
			cherryy.position.y *= mirror;
			PhysicalBoardState.push_back(cherryx);
			PhysicalBoardState.push_back(cherryy);
		}
	}

	Objectives.clear();
	//Bigger objectives should be at the back of the list
	for (int i = 0; i < 3; i++)
	{
		Objectives.push_back(make_unique<RetractTrayObjective>(i));
	}
	Objectives.push_back(make_unique<RetractClawsObjective>());
	Objectives.push_back(make_unique<GatherCherriesObjective>());
	Objectives.push_back(make_unique<DepositCherriesObjective>());
	Objectives.push_back(make_unique<TrayRoutine>());
	Objectives.push_back(make_unique<TakeStackObjective>());
}

void Manager::GatherData()
{
	for (int i = 0; i < RobotControllers.size(); i++)
	{
		RobotControllers[i]->Tick();
	}
	
}

void Manager::Run(double delta)
{
	int numrobots = RobotControllers.size();
	assert(numrobots ==1); //todo: make the following code work for multiple robots. Should be numobjective^robots in complexity tho if we try everything :/
	vector<map<ActuatorType, std::pair<BaseObjective*, double>>> BestObjective;
	vector<map<BaseObjective*, vector<ActuatorType>>> ObjectiveActuators;
	BestObjective.resize(numrobots);
	ObjectiveActuators.resize(numrobots);

	//Copy the state of the board, and simulate every objective to pick the one that makes the most points
	std::vector<Object> SimulationBoardState;
	std::vector<RobotMemory> SimulationRobotStates;
	std::vector<RobotHAL> SimulatedControllers;
	SimulatedControllers.resize(numrobots);
	SimulationRobotStates.resize(numrobots);
	int robotidx = 0;
	auto& robjectives = BestObjective[robotidx];//Best objectives for the current robot
	auto& ractuators = ObjectiveActuators[robotidx]; //Actuators used for each objective
	for (auto &objective : Objectives)
	{
		SimulationBoardState = PhysicalBoardState;
		for (int copyrobotidx = 0; copyrobotidx < RobotControllers.size(); copyrobotidx++)
		{
			SimulationRobotStates[copyrobotidx].CopyFrom(PhysicalRobotStates[copyrobotidx]);
			SimulatedControllers[copyrobotidx] = RobotHAL(*RobotControllers[copyrobotidx]);
		}
		double timebudget = TimeLeft;
		auto ExecutionPair = objective->ExecuteObjective(timebudget, &SimulatedControllers[robotidx], SimulationBoardState, &SimulationRobotStates[robotidx]);
		double Score = ExecutionPair.first;
		auto& categories = ExecutionPair.second;
		//cout << "Objective " << objective->GetName() << " gave score " << Score << endl;
		ractuators[objective.get()] = categories;
		double ScoreThreshold = 0;
		vector<BaseObjective*> seenObj;
		for (auto category : categories)
		{
			auto found = robjectives.find(category);
			auto& thepair = found->second;
			if (found != robjectives.end()) //if there's an objective using that actuator
			{
				auto seen = find(seenObj.begin(), seenObj.end(), thepair.first);
				if (seen != seenObj.end())
				{
					continue; //already seen this objective, don't add it's score
				}
				
				ScoreThreshold += thepair.second;
				seenObj.push_back(thepair.first);
			}
		}
		if (Score > ScoreThreshold)
		{
			//remove all objectives that have been seen from the best objectives, as we're now using those actuators
			for (auto & obj : seenObj)
			{
				auto cats = ractuators[obj];
				for (auto& cat : cats)
				{
					auto found = robjectives.find(cat);
					if (found == robjectives.end())
					{
						continue;
					}
					robjectives.erase(found);
					//cout << "Objective " << objective->GetName() << " is displacing objective " << obj->GetName() << " at " << (int)cat <<endl;
				}
			}
			//register that objective as the objective for those actuators
			for (auto category : categories)
			{
				robjectives[category] = make_pair(objective.get(), Score);
			}
		}
		
	}

	//BestObjective[0] = Objectives[1].get();

	//Objectives selected : execute them for real
	for (int robotidx = 0; robotidx < numrobots; robotidx++)
	{
		vector<BaseObjective*> seenObj;
		string ObjectivesChosen = "";
		for (auto& entry : BestObjective[robotidx])
		{
			auto& pair = entry.second;
			auto seen = find(seenObj.begin(), seenObj.end(), pair.first);
			if (seen != seenObj.end())
			{
				continue;
			}
			double timebudget = delta;
			seenObj.push_back(pair.first);
			pair.first->ExecuteObjective(timebudget, RobotControllers[robotidx], PhysicalBoardState, &PhysicalRobotStates[robotidx]);
			ObjectivesChosen += " " + pair.first->GetName(); 
		}
		cout << "Robot " << robotidx << " chose objectives" << ObjectivesChosen <<endl;
	}
}

bool Manager::Display()
{
#ifdef WITH_SAURON
	vector<GLObject> dataconcat;
	dataconcat.emplace_back(MeshNames::arena);
	vector<pair<Vector2dd, double>> CakeHeights;
	static const map<ObjectType, MeshNames> boardgltypemap = {
		{ObjectType::Robot, 		MeshNames::robot},
		{ObjectType::Cherry, 		MeshNames::cherry},
		{ObjectType::CakeBrown, 	MeshNames::browncake},
		{ObjectType::CakeYellow, 	MeshNames::yellowcake},
		{ObjectType::CakePink, 		MeshNames::pinkcake},
		{ObjectType::GreenDropZone, MeshNames::axis},
		{ObjectType::BlueDropZone, 	MeshNames::axis}
	};
	for (const auto &object : PhysicalBoardState)
	{
		MeshNames type = boardgltypemap.at(object.Type);
		double posZ = 0.0;
		switch (object.Type)
		{
		case ObjectType::Cherry:
			posZ = 0.035;
			break;
		case ObjectType::CakeBrown:
		case ObjectType::CakePink:
		case ObjectType::CakeYellow:
			{
				for (int i = 0; i < CakeHeights.size(); i++)
				{
					auto cake = CakeHeights[i];
					auto pos = cake.first;
					auto height = cake.second;
					if ((pos-object.position).length() < 0.12) //0.12 is the cake diameter
					{
						posZ = max(posZ, height+CakeHeight);
					}
					
				}
				CakeHeights.push_back({object.position, posZ});
			}
			break;
		default:
			break;
		}
		dataconcat.emplace_back(type, object.position.x, object.position.y, posZ);
	}
	for (int robotidx = 0; robotidx < RobotControllers.size(); robotidx++)
	{
		const auto& robothandle = RobotControllers[robotidx];
		const auto& robotmem = PhysicalRobotStates[robotidx];
		Affine3d robotloc;
		//cout << "Robot " << robotidx << " speed is " << robothandle->PositionLinear.Speed << endl;
		robotloc.translation(Vec3d(robothandle->position.x, robothandle->position.y, 0.0));
		auto forward  = robothandle->GetForwardVector();
		Vec3d xvec(forward.x,forward.y,0), yvec(-forward.y,forward.x,0);
		robotloc.rotation(MakeRotationFromXY(xvec, yvec));
		dataconcat.emplace_back(MeshNames::robot, Affine3DToGLM(robotloc));
		Affine3d cherrypos = robotloc;
		auto cherrypos2d = robothandle->CherryPickupPosition.rotate(robothandle->Rotation.Pos);
		cherrypos.translation(cherrypos.translation() + Vec3d(cherrypos2d.x, cherrypos2d.y, 0));
		dataconcat.emplace_back(MeshNames::axis, Affine3DToGLM(cherrypos));
		for (int trayidx = 0; trayidx < sizeof(robotmem.CakeTrays) / sizeof(robotmem.CakeTrays[0]); trayidx++)
		{
			const auto& tray = robotmem.CakeTrays[trayidx];
			Vec3d traypos;
			MeshNames meshtype;
			if (trayidx == 0)
			{
				traypos = robotloc.translation() 
					+ xvec * (robothandle->ClawPickupPosition.x * robothandle->ClawExtension.Pos) 
					+ yvec * robothandle->ClawPickupPosition.y
					+ Vec3d(0,0,robothandle->ClawHeight.Pos);
				meshtype = MeshNames::robot_claw;
			}
			else
			{
				traypos = robotloc.translation() 
					+ xvec * robothandle->Trays[trayidx-1].Pos * robothandle->ClawPickupPosition.x
					+ Vec3d(0,0,robothandle->TrayHeights[trayidx]);
				meshtype = MeshNames::robot_tray;
			}
			Affine3d traytransform(robotloc.rotation(), traypos);
			dataconcat.emplace_back(meshtype, Affine3DToGLM(traytransform));
			for (int cakeidx = 0; cakeidx < tray.size(); cakeidx++)
			{
				const Object& object = tray[cakeidx];
				MeshNames type = boardgltypemap.at(object.Type);
				Affine3d caketransform(traytransform.rotation(), traytransform.translation() + Vec3d(0,0,0.02*cakeidx));
				dataconcat.emplace_back(type, Affine3DToGLM(caketransform));
			}
		}
	}
	
	
	return visualiser.Tick(dataconcat);
#endif
	return true;
}

void Manager::Thread()
{
	Init();
#ifdef WITH_SAURON
	visualiser.Start("Overlord");
#if false
	Mat posplotdisc = Mat::zeros(Size(1000,800), CV_8UC3);
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
	}
#endif
#endif
	bool killed = false;
	//usleep(2*1000000);
	lastTick = chrono::steady_clock::now();
	while (!killed)
	{
		auto now = chrono::steady_clock::now();
		std::chrono::duration<double> delta = now - lastTick;
		GatherData();
		//Run(delta.count()*2);
		Run(1.0/200);
		killed = !Display();
		//waitKey(1000/50);
		lastTick = now;
	}
}