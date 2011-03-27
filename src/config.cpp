#include "config.h"

GA_VEHICLE::Config::Config()
{
	mutationChance = 3;
	mutationFactor = 1.0;

	tournamentSize = 2;
	toCrossOverSize = 10;

	populationSize = 20;

	verticeMinLength = 0.5; 
	verticeMaxLength = 10;
	verticeCount = 8;

	wheelMinSize = 1; 
	wheelMaxSize = 3;

	velocityIterations = 10;
	positionIterations = 10;

	wheelTorqueMin = 50;
	wheelTorqueMax = 1000;

	wheelSpeedMin = 2;
	wheelSpeedMax = 10;

	allowedStandStillSteps = 500; // 60/sec
	minMove = 5;
}
GA_VEHICLE::Config* GA_VEHICLE::Config::get()
{
	static Config* instance;
	if(!instance)
	{
		instance = new Config();
	}
	return instance;
}

void GA_VEHICLE::Config::LoadFromFile(std::string filename)
{

}