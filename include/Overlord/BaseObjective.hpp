#pragma once

class BaseObjective
{
private:
	/* data */
public:
	BaseObjective();
	~BaseObjective();

	//Estimate the amount of points per second that executing this objective will give
	//Return value is points gained / time estimated 
	virtual float EvaluateObjective() {return -1;};

	//Carry out the objective
	virtual void ExecuteObjective() {};

	//Get the total number of points that this objective have given us
	virtual float GetPoints() {return 0;};
};

