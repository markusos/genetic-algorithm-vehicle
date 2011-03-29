#include "config.h"
#include <time.h>

GA_VEHICLE::Config::Config()
{
	std::ifstream in;

	in.open ("config.cfg", std::ifstream::in);
	
	unsigned int seed;
	in >> seed;
	if(seed == 0) seed = time(0);

	gen.seed(seed);

	in >> display >> splitPoints >> mutationChance >> tournamentSize >> toCrossOverSize >> populationSize >> verticeMinLength 
	   >> verticeMaxLength >> verticeCount >> wheelMinSize >> wheelMaxSize >> velocityIterations >> positionIterations 
	   >> wheelTorqueMin >> wheelTorqueMax >> wheelSpeedMin >> wheelSpeedMax >> allowedStandStillSteps >> minMove;

	in.close();

	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	strftime (buffer,80,"log_%H.%M.%S_%d-%m-%y.log",timeinfo);
	m_log.open(buffer);

	/*
	splitPoints = 2;

	mutationChance = 0.03;

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
	*/
	m_log << "seed = " << seed << std::endl;
	m_log << "splitPoints =" << splitPoints << std::endl;

	m_log << "mutationChance =" << mutationChance << std::endl;

	m_log << "tournamentSize =" << tournamentSize << std::endl;
	m_log << "toCrossOverSize =" << toCrossOverSize << std::endl;

	m_log << "populationSize =" << populationSize << std::endl;

	m_log << "verticeMinLength =" << verticeMinLength << std::endl;
	m_log << "verticeMaxLength =" << verticeMaxLength << std::endl;
	m_log << "verticeCount =" << verticeCount << std::endl;

	m_log << "wheelMinSize =" << wheelMinSize << std::endl;
	m_log << "wheelMaxSize =" << wheelMaxSize << std::endl;

	m_log << "velocityIterations =" << velocityIterations << std::endl;
	m_log << "positionIterations =" << positionIterations << std::endl;

	m_log << "wheelTorqueMin =" << wheelTorqueMin << std::endl;
	m_log << "wheelTorqueMax =" << wheelTorqueMax << std::endl;

	m_log << "wheelSpeedMin =" << wheelSpeedMin << std::endl;
	m_log << "wheelSpeedMax =" << wheelSpeedMax << std::endl;

	m_log << "allowedStandStillSteps =" << allowedStandStillSteps << std::endl;
	m_log << "minMove =" << minMove << std::endl;

	m_log << "Generation Mean_value Max" << std::endl;
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