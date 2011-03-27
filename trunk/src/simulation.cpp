#include "simulation.h"
#include "config.h"
#include <GL/glfw.h>
#include <GL/glut.h>

#include <iostream>
#include <Box2D\Common\b2Settings.h>
#include <time.h>

GA_VEHICLE::Simulation::Simulation() : m_time(0), m_timeStep(1.0/60.0), m_render(true), m_stepsPerRenderFrame(6)
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [80];

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	strftime (buffer,80,"log_%H.%M_%d-%m-%y.log",timeinfo);
	m_log.open(buffer);

	
	m_log << "Generation Median Max" << std::endl;
	
	b2Vec2 gravity(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(gravity, doSleep);
	m_currentVehicle = 0;
	m_generationCounter = 1;
	m_stepsStillForThisVehicle = 0;
	addTests();

	if(m_render)
	{
		m_renderer = new Renderer(m_world);
		uint32 flags = 0;
		flags += b2DebugDraw::e_shapeBit;
		flags += b2DebugDraw::e_jointBit;
		//flags += b2DebugDraw::e_aabbBit;
		//flags += b2DebugDraw::e_pairBit;
		//flags += b2DebugDraw::e_centerOfMassBit;
		m_renderer->SetFlags(flags);
		m_world->SetDebugDraw(m_renderer);

	}

	initRandomPopulation();

	glfwSetTime(0);
	mainLoop();
	m_log.close();
}

GA_VEHICLE::Simulation::~Simulation()
{
}

bool GA_VEHICLE::Simulation::stepPhysics()
{
	if(m_render)
	{
		double currentTime = glfwGetTime();
		double dt = currentTime - m_time;
		if(dt > m_timeStep)
		{
			m_time = currentTime;
			
			for(int i=0;i<m_stepsPerRenderFrame;i++)
			{
				m_world->Step(m_timeStep, Config::get()->velocityIterations, Config::get()->positionIterations);
				m_world->ClearForces();
			}
			
			return true;
		}
		else
		{
			glfwSleep(0.001);
			return false;	
		}
	}
	else
	{
		for(int i=0;i<m_stepsPerRenderFrame;i++)
		{
			m_world->Step(m_timeStep, Config::get()->velocityIterations, Config::get()->positionIterations);
			m_world->ClearForces();
		}
		return true;
	}
	
}

void GA_VEHICLE::Simulation::mainLoop()
{
	bool running = true;
	while (running)
	{
		if (glfwGetKey(GLFW_KEY_ESC) || (m_render &&!glfwGetWindowParam(GLFW_OPENED)) )
		{
			running = false;
		}

		if (glfwGetKey('R'))
		{
			m_render = true;
		}
		else if (glfwGetKey('T'))
		{
			m_render = false;
		}

		if(m_currentVehicle >= m_population.size())
		{
			float medianFitness = 0;
			float maxFitness = 0;
			for(int i = 0; i < m_population.size(); i++)
			{
				medianFitness += m_population[i].m_fitness;
				if(m_population[i].m_fitness > maxFitness) maxFitness = m_population[i].m_fitness;
			}
			medianFitness = medianFitness/m_population.size();
			m_log << m_generationCounter << ";" << medianFitness << ";" << maxFitness << std::endl;
			

			m_currentVehicle = 0;
			m_generationCounter++;
			m_population = mutation( crossOver( selection(m_population)));
			m_population[m_currentVehicle].addToWorld();
			//Init evaluateVehicleAbortCondition variables
			oldPosX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
			//oldTime = glfwGetTime();
			m_stepsStillForThisVehicle = 0;
		}

		if(stepPhysics()) // physics "stepped"
		{
			if(m_render)
			{
				render();
			}
			if(evaluateVehicleAbortCondition(m_population[m_currentVehicle]))
			{
				
				m_population[m_currentVehicle].m_fitness = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
				std::cout << "abort == true, generation == "<< m_generationCounter <<" fitness == "<< m_population[m_currentVehicle].m_fitness<< std::endl;
				////////////////////////////////////////////////////////
				//Testkod
				//if(m_population[m_currentVehicle].m_fitness > 1200) m_currentVehicle--;
				/////////////////////////////////////////////////////////
				m_population[m_currentVehicle].removeFromWorld();
				m_currentVehicle++;
				if(m_currentVehicle < m_population.size())
				{
					m_population[m_currentVehicle].addToWorld();
					//Init evaluateVehicleAbortCondition variables
					oldPosX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
					//oldTime = glfwGetTime();
					m_stepsStillForThisVehicle = 0;
				}
			}
			glfwSwapBuffers();// otherwise keyboard etc wont work..
		}
	}
}

void GA_VEHICLE::Simulation::render()
{
	m_renderer->display(m_population[m_currentVehicle].m_vehicleBody);
}

void GA_VEHICLE::Simulation::addTests()
{
	b2BodyDef groundBodyDef;
	b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
	b2PolygonShape groundBox;
	std::vector<float> groundPoints;

	//Start
	groundPoints.push_back(100);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);

	//smal bumps
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(20);

	//smal hill
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(26);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(28);
	groundPoints.push_back(26);
	groundPoints.push_back(24);
	groundPoints.push_back(22);
	groundPoints.push_back(20);

	//medium bumps
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(24);
	groundPoints.push_back(20);
	groundPoints.push_back(24);
	groundPoints.push_back(20);
	groundPoints.push_back(24);
	groundPoints.push_back(20);

	//Large jump
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(32);
	groundPoints.push_back(38);
	groundPoints.push_back(42);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);

	//U pit
	groundPoints.push_back(20);
	groundPoints.push_back(8);
	groundPoints.push_back(4);
	groundPoints.push_back(2);
	groundPoints.push_back(4);
	groundPoints.push_back(8);
	groundPoints.push_back(20);

	//two medium jumps
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(32);
	groundPoints.push_back(34);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(10);
	groundPoints.push_back(10);

	//Hill
	groundPoints.push_back(12);
	groundPoints.push_back(18);
	groundPoints.push_back(22);

	//Large bumbs
	groundPoints.push_back(22);
	groundPoints.push_back(28);
	groundPoints.push_back(22);
	groundPoints.push_back(28);
	groundPoints.push_back(22);
	groundPoints.push_back(28);
	groundPoints.push_back(22);
	groundPoints.push_back(28);
	groundPoints.push_back(22);

	//Larger bumps
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);

	//jump
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(42);
	groundPoints.push_back(10);
	groundPoints.push_back(20);

	//Hill
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(26);
	groundPoints.push_back(28);
	groundPoints.push_back(32);
	groundPoints.push_back(38);
	groundPoints.push_back(32);
	groundPoints.push_back(38);
	groundPoints.push_back(32);
	groundPoints.push_back(26);
	groundPoints.push_back(20);

	//Double U pit
	groundPoints.push_back(20);
	groundPoints.push_back(8);
	groundPoints.push_back(4);
	groundPoints.push_back(2);
	groundPoints.push_back(4);
	groundPoints.push_back(8);
	groundPoints.push_back(20);
	groundPoints.push_back(8);
	groundPoints.push_back(4);
	groundPoints.push_back(2);
	groundPoints.push_back(4);
	groundPoints.push_back(8);
	groundPoints.push_back(20);
	
	//tree jumps
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(10);
	groundPoints.push_back(12);
	groundPoints.push_back(14);
	groundPoints.push_back(18);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(10);
	groundPoints.push_back(12);
	groundPoints.push_back(14);
	groundPoints.push_back(18);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(10);
	groundPoints.push_back(14);
	groundPoints.push_back(18);
	groundPoints.push_back(22);
	groundPoints.push_back(20);

	//End
	groundPoints.push_back(100);


	for(int i=0; i< groundPoints.size() - 1 ;i++)
	{
		b2Vec2 box[] = {b2Vec2(i*10+30,groundPoints[i]), b2Vec2(i*10+30,groundPoints[i]-2), b2Vec2(i*10+30+10,groundPoints[i+1]-2), b2Vec2(i*10+30+10,groundPoints[i+1])};
		groundBox.Set(box,4);
		groundBody->CreateFixture(&groundBox, 0.0f);
	}

}

void GA_VEHICLE::Simulation::initRandomPopulation()
{
	for(int i=0;i<Config::get()->populationSize;i++)
	{
		m_population.push_back(Vehicle(m_world, 0, 2));
	}
	
	/*
	std::vector<VehicleVertex> vertices;
	vertices.push_back(VehicleVertex(0,5,0));
	vertices.push_back(VehicleVertex(0,7,pi/4));
	vertices.push_back(VehicleVertex(0,3,pi/2));
	vertices.push_back(VehicleVertex(0,7,3*pi/4));
	vertices.push_back(VehicleVertex(0,5,pi));
	vertices.push_back(VehicleVertex(0,1,-pi/2));
	std::vector<Wheel> wheels;
	wheels.push_back(Wheel(-pi/4,-5,300,0,2));
	wheels.push_back(Wheel(-3*pi/4,-5,300,4,2));
	wheels.push_back(Wheel(-pi/4,-5,300,5,1.3));
	wheels.push_back(Wheel(-3*pi/4,-5,300,5,1.3));

	Vehicle vehicle = Vehicle(m_world, 0,vertices,wheels);
	m_population.push_back(vehicle);
	
	std::vector<VehicleVertex> vertices2;
	vertices2.push_back(VehicleVertex(0,5,0));
	vertices2.push_back(VehicleVertex(0,3,pi/4));
	vertices2.push_back(VehicleVertex(0,5,pi/2));
	vertices2.push_back(VehicleVertex(0,3,3*pi/4));
	vertices2.push_back(VehicleVertex(0,5,pi));
	vertices2.push_back(VehicleVertex(0,1,-pi/2));
	std::vector<Wheel> wheels2;
	wheels2.push_back(Wheel(-pi/4,-5,300,0,2));
	wheels2.push_back(Wheel(-3*pi/4,-5,300,4,2));
	//wheels2.push_back(Wheel(-pi/4,-5,300,5,1.3));
	//wheels2.push_back(Wheel(-3*pi/4,-5,300,5,1.3));

	Vehicle vehicle2 = Vehicle(m_world, 0,vertices2,wheels2);
	m_population.push_back(vehicle2);

	
	*/
	m_population[0].addToWorld();
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::selection(std::vector<Vehicle> vehicles)
{
	//binary tournament selection

	std::vector<Vehicle> toCrossOver;

	for(int i=0;i<Config::get()->toCrossOverSize;i++)
	{
		std::vector<Vehicle> tournament;
		for (int j=0;j < std::min<int>(Config::get()->tournamentSize, vehicles.size()); j++)
		{
			int pos = rand() % vehicles.size();
			tournament.push_back(vehicles[pos]);
			vehicles.erase(vehicles.begin()+ pos);
		}
		int best = 0;
		for(int j=0;j<tournament.size();j++)
		{
			if(tournament[j].m_fitness > tournament[best].m_fitness)
			{
				best = j;
			}
		}
		toCrossOver.push_back(tournament[best]);
		tournament.erase(tournament.begin()+ best);

		for(int j=0;j<tournament.size();j++)
		{
			vehicles.push_back(tournament[j]);
		}
	}
	return toCrossOver;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::crossOver(std::vector<Vehicle> vehicles)
{

	std::vector<Vehicle> newVehicles;

	for (int i = 0; i < vehicles.size(); i++)
	{
		std::vector<Chromosome> a = vehicles[i].getGenome();
		std::vector<Chromosome> b = vehicles[(i+2)%vehicles.size()].getGenome();

		if (a.size() == b.size())
		{
			int split = rand()%(a.size()-1)+1;
			for (int j = 0; j < a.size(); j++)
			{
				if (j >= split)
				{
					Chromosome temp = a[j];
					a[j] = b[j];
					b[j] = temp;
				}
			}
			newVehicles.push_back(Vehicle(m_world,a));
			newVehicles.push_back(Vehicle(m_world,b));
		}
		else std::cout << "ERROR: Genomes of different size" << std::endl;
	}

	return newVehicles;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::mutation(std::vector<Vehicle> vehicles)
{
	std::vector<Vehicle> newVehicles;
	for(int i=0;i<vehicles.size();i++)
	{
		std::vector<Chromosome> genome = vehicles[i].getGenome();
		for(int j=0;j<genome.size();j++)
		{
			if(rand()%100 <=Config::get()->mutationChance)
			{
				if(genome[j].type == Chromosome::POINTDISTANCE)
				{
					genome[j].value = (Config::get()->verticeMaxLength-Config::get()->verticeMinLength)*(rand()/(float)(RAND_MAX+1))+Config::get()->verticeMinLength;
				}
				else if(genome[j].type == Chromosome::WHEELSIZE)
				{
					genome[j].value = (Config::get()->wheelMaxSize-Config::get()->wheelMinSize)*(rand()/(float)(RAND_MAX+1))+Config::get()->wheelMinSize;
				}
				else if(genome[j].type == Chromosome::WHEELANGLE)
				{
					genome[j].value = (2*b2_pi)*(rand()/(float)(RAND_MAX+1));
				}
				else if(genome[j].type == Chromosome::WHEELPOS)
				{
					genome[j].value = (Config::get()->verticeCount)*(rand()/(float)(RAND_MAX+1));
				}
				else if(genome[j].type == Chromosome::WHEELSPEED)
				{
					genome[j].value = (Config::get()->wheelSpeedMax-Config::get()->wheelSpeedMin)*(rand()/(float)(RAND_MAX+1))+Config::get()->wheelSpeedMin;
				}
				else if(genome[j].type == Chromosome::WHEELTORQUE)
				{
					genome[j].value = (Config::get()->wheelTorqueMax-Config::get()->wheelTorqueMin)*(rand()/(float)(RAND_MAX+1))+Config::get()->wheelTorqueMin;
				}
				/*
				if(rand()%2 == 1)
				{
					
					if(genome[j].type == Chromosome::POINTDISTANCE)
					{
						genome[j].value += mutationFactor*(verticeMaxLength-verticeMinLength)*(rand()%100+1)/100.0;
						if(genome[j].value > verticeMaxLength) genome[j].value = verticeMaxLength;
					}
					else if(genome[j].type == Chromosome::WHEELSIZE)
					{
						genome[j].value += mutationFactor*(wheelMaxSize-wheelMinSize)*(rand()%100+1)/100.0;
						if(genome[j].value > wheelMaxSize) genome[j].value = wheelMaxSize;
					}
					else if(genome[j].type == Chromosome::WHEELANGLE)
					{
						genome[j].value += mutationFactor*(2*b2_pi)*(rand()%100+1)/100.0;
					}
					else if(genome[j].type == Chromosome::WHEELPOS)
					{
						genome[j].value += std::min(mutationFactor*(verticeCount)*(rand()%100+1)/100.0,1.0);
						if(genome[j].value >= verticeCount) genome[j].value = verticeCount-1;
					}
					else if(genome[j].type == Chromosome::WHEELSPEED)
					{
						genome[j].value += mutationFactor*(wheelSpeedMax-wheelSpeedMin)*(rand()%100+1)/100.0;
						if(genome[j].value > wheelSpeedMax) genome[j].value = wheelSpeedMin;
					}
					else if(genome[j].type == Chromosome::WHEELTORQUE)
					{
						genome[j].value += mutationFactor*(wheelTorqueMax-wheelTorqueMin)*(rand()%100+1)/100.0;
						if(genome[j].value > wheelTorqueMax) genome[j].value = wheelTorqueMax;
					}
					
				}
				else
				{
					if(genome[j].type == Chromosome::POINTDISTANCE)
					{
						genome[j].value -= mutationFactor*(verticeMaxLength-verticeMinLength)*(rand()%100+1)/100.0;
						if(genome[j].value < verticeMinLength) genome[j].value = verticeMinLength;
					}
					else if(genome[j].type == Chromosome::WHEELSIZE)
					{
						genome[j].value -= mutationFactor*(wheelMaxSize-wheelMinSize)*(rand()%100+1)/100.0;
						if(genome[j].value < wheelMinSize) genome[j].value = wheelMinSize;
					}
					else if(genome[j].type == Chromosome::WHEELANGLE)
					{
						genome[j].value -= mutationFactor*(2*b2_pi)*(rand()%100+1)/100.0;
					}
					else if(genome[j].type == Chromosome::WHEELPOS)
					{
						genome[j].value -= std::min(mutationFactor*(verticeCount)*(rand()%100+1)/100.0,1.0);
						if(genome[j].value < 0) genome[j].value = 0;
					}
					else if(genome[j].type == Chromosome::WHEELSPEED)
					{
						genome[j].value -= mutationFactor*(wheelSpeedMax-wheelSpeedMin)*(rand()%100+1)/100.0;
						if(genome[j].value < wheelSpeedMin) genome[j].value = wheelSpeedMin;
					}
					else if(genome[j].type == Chromosome::WHEELTORQUE)
					{
						genome[j].value -= mutationFactor*(wheelTorqueMax-wheelTorqueMin)*(rand()%100+1)/100.0;
						if(genome[j].value < wheelTorqueMin) genome[j].value = wheelTorqueMin;
					}
				}*/
			}
		}
		newVehicles.push_back(Vehicle(m_world,genome));
	}
	return newVehicles;
}

bool GA_VEHICLE::Simulation::evaluateVehicleAbortCondition(Vehicle& vehicle)
{
	float32 posX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
	m_stepsStillForThisVehicle += m_stepsPerRenderFrame;

	//double timeNow = glfwGetTime();

	//double allowedStandStillTime = 1.5;


	if (m_stepsStillForThisVehicle > Config::get()->allowedStandStillSteps){
	//if ((timeNow - oldTime) > allowedStandStillTime){
		if (abs(posX - oldPosX) < Config::get()->minMove)
		{ 
			return true;
		}
		else{
			//oldTime = timeNow;
			m_stepsStillForThisVehicle = 0;
			oldPosX = posX;
			return false;
		}
	}
	else{
		return false;
	}
}