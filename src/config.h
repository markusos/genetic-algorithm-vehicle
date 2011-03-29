#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <string>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_01.hpp>
#include <iostream>
#include <fstream>

namespace GA_VEHICLE
{
	class Config
	{
	
	public:
		static Config* get();
		int randomInInterval(int lowerBound, int upperBound);
		double random01();

		int splitPoints;

		float mutationChance;

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

		std::ofstream m_log;
	private:
		Config();
		boost::mt19937 gen;
	};
}

#endif