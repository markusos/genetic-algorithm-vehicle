#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>

namespace GA_VEHICLE
{
	//class Config
	//{
	//public:
		//static void LoadFromFile(std::string filename);
		//static void standardConfig();

		static int mutationChance = 5; // %
		static float mutationFactor = 1.0;

		static int tournamentSize = 2;
		static int toCrossOverSize = 10;

		static int populationSize = 20;

		static float verticeMinLength = 0.5; 
		static float verticeMaxLength = 10;
		static int verticeCount = 8;

		static float wheelMinSize = 1; 
		static float wheelMaxSize = 3;

		static int velocityIterations = 10;
		static int positionIterations = 10;

		static int wheelTorqueMin = 50;
		static int wheelTorqueMax = 1000;

		static int wheelSpeedMin = 2;
		static int wheelSpeedMax = 10;
	//};
}

#endif