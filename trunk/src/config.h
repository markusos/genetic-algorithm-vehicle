#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_01.hpp>

namespace GA_VEHICLE
{
	class Config
	{
	
	public:
		static Config* get();
		void LoadFromFile(std::string filename);
		int randomInInterval(int lowerBound, int upperBound);
		double random01();

		float mutationChance;
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
		boost::mt19937 gen;
	};
}

#endif