#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>

namespace GA_VEHICLE
{
	class Config
	{
	
	public:
		static Config* get();
		void LoadFromFile(std::string filename);

		int mutationChance; // %
		float mutationFactor;

		int tournamentSize;
		int toCrossOverSize;

		int populationSize;

		float verticeMinLength; 
		float verticeMaxLength;
		int verticeCount;

		float wheelMinSize; 
		float wheelMaxSize;

		int velocityIterations;
		int positionIterations;

		int wheelTorqueMin;
		int wheelTorqueMax;

		int wheelSpeedMin;
		int wheelSpeedMax;
		long allowedStandStillSteps; // 60/sec
		float minMove;
		
	private:
		Config();
	};
}

#endif