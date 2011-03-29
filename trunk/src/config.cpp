#include "config.h"
#include <time.h>

GA_VEHICLE::Config::Config()
{
	unsigned int seed = time(0);
	//unsigned int seed = 0;
	gen.seed(seed);

	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	strftime (buffer,80,"log_%H.%M_%d-%m-%y.log",timeinfo);
	m_log.open(buffer);

	m_log << "seed = " << seed << std::endl;
	m_log << "Generation Mean_value Max" << std::endl;

	mutationChance = 0.03;
	mutationFactor = 1.0;

	tournamentSize = 2;
	toCrossOverSize = 10;

	populationSize = 20;

	verticeMinLength = 0.5; 
	verticeMaxLength = 7;
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

int GA_VEHICLE::Config::randomInInterval(int lowerBound, int upperBound)
{
	boost::uniform_int<> dist(lowerBound, upperBound);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(gen, dist);
    return die();
}

double GA_VEHICLE::Config::random01()
{
	static boost::uniform_01<boost::mt19937> zeroone(gen);
	return zeroone();
}