#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>

namespace GA_VEHICLE
{
	class Config
	{
	private:
		static Config* instance;
		Config()
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
	public:
		
		static Config* get()
		{
			if(!instance)
			{
				instance = new Config();
			}
			return instance;
		}
		void LoadFromFile(std::string filename)
		{

		}
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
		
	
	};
}

#endif