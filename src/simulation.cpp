#include "simulation.h"
#include "config.h"
#include <GL/glfw.h>
#include <GL/glut.h>

#include <Box2D\Common\b2Settings.h>
#include <time.h>

GA_VEHICLE::Simulation::Simulation() : m_time(0), m_timeStep(1.0/60.0), m_render(false), m_stepsPerRenderFrame(6)
{
	
	b2Vec2 gravity(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(gravity, doSleep);
	m_currentVehicle = 0;
	m_generationCounter = 1;
	m_stepsStillForThisVehicle = 0;
	addTests();

	if(Config::get()->display)
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
	Config::get()->m_log.close();
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
		if(Config::get()->display)
		{
			if (glfwGetKey(GLFW_KEY_ESC) || !glfwGetWindowParam(GLFW_OPENED) )
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
		}

		if(m_currentVehicle >= m_population.size())
		{
			float meanValueFitness = 0;
			float maxFitness = 0;
			for(int i = 0; i < m_population.size(); i++)
			{
				meanValueFitness += m_population[i].m_fitness;
				if(m_population[i].m_fitness > maxFitness) maxFitness = m_population[i].m_fitness;
			}
			meanValueFitness = meanValueFitness/m_population.size();
			Config::get()->m_log << m_generationCounter << ";" << meanValueFitness << ";" << maxFitness << std::endl;
			////////////////////////////////////////////////////////
				//End Test
				if(maxFitness > 2200){
					running = false;
					return;
				}
			/////////////////////////////////////////////////////////

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
			if(m_render && Config::get()->display)
			{
				render();
			}
			if(evaluateVehicleAbortCondition(m_population[m_currentVehicle]))
			{
				
				m_population[m_currentVehicle].m_fitness = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
				std::cout << "abort == true, generation == "<< m_generationCounter <<" fitness == "<< m_population[m_currentVehicle].m_fitness<< std::endl;
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
			if(Config::get()->display)
			{
				glfwSwapBuffers();// otherwise keyboard etc wont work..
			}
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
	groundPoints.push_back(24);
	groundPoints.push_back(20);

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);

	//Large jump
	groundPoints.push_back(24);
	groundPoints.push_back(28);
	groundPoints.push_back(32);
	groundPoints.push_back(36);
	groundPoints.push_back(40);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);

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

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
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
	groundPoints.push_back(10);
	groundPoints.push_back(10);
	groundPoints.push_back(10);

	//Hill
	groundPoints.push_back(14);
	groundPoints.push_back(18);
	groundPoints.push_back(22);

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);

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
	groundPoints.push_back(28);
	groundPoints.push_back(22);

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(22);
	groundPoints.push_back(24);

	//Larger bumps
	groundPoints.push_back(28);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(30);

	groundPoints.push_back(30);
	groundPoints.push_back(30);
	groundPoints.push_back(30);
	groundPoints.push_back(30);

	//jump
	groundPoints.push_back(30);
	groundPoints.push_back(36);
	groundPoints.push_back(42);
	groundPoints.push_back(10);
	groundPoints.push_back(20);

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
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
	groundPoints.push_back(20);

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
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

	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	
	//tree jumps
	groundPoints.push_back(20);
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
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
	groundPoints.push_back(20);
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
			int pos = Config::get()->randomInInterval(0,vehicles.size()-1);
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
	if(Config::get()->splitPoints == 1) return OnePointCrossOver(vehicles);
	else if (Config::get()->splitPoints == 2) return TwoPointCrossOver(vehicles);
	else std::cout << "ERROR: Not correct number of splitpoints" << std::endl;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::mutation(std::vector<Vehicle> vehicles)
{
	std::vector<Vehicle> newVehicles;
	for(int i=0;i<vehicles.size();i++)
	{
		std::vector<Chromosome> genome = vehicles[i].getGenome();
		for(int j=0;j<genome.size();j++)
		{
			if(Config::get()->random01() <= Config::get()->mutationChance)
			{
				if(genome[j].type == Chromosome::POINTDISTANCE)
				{
					genome[j].value = (Config::get()->verticeMaxLength-Config::get()->verticeMinLength)*Config::get()->random01()+Config::get()->verticeMinLength;
				}
				else if(genome[j].type == Chromosome::WHEELSIZE)
				{
					genome[j].value = (Config::get()->wheelMaxSize-Config::get()->wheelMinSize)*Config::get()->random01()+Config::get()->wheelMinSize;
				}
				else if(genome[j].type == Chromosome::WHEELANGLE)
				{
					genome[j].value = (2*b2_pi)*Config::get()->random01();
				}
				else if(genome[j].type == Chromosome::WHEELPOS)
				{
					genome[j].value = (Config::get()->verticeCount)*Config::get()->random01();
				}
				else if(genome[j].type == Chromosome::WHEELSPEED)
				{
					genome[j].value = (Config::get()->wheelSpeedMax-Config::get()->wheelSpeedMin)*Config::get()->random01()+Config::get()->wheelSpeedMin;
				}
				else if(genome[j].type == Chromosome::WHEELTORQUE)
				{
					genome[j].value = (Config::get()->wheelTorqueMax-Config::get()->wheelTorqueMin)*Config::get()->random01()+Config::get()->wheelTorqueMin;
				}
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



std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::OnePointCrossOver(std::vector<Vehicle> vehicles)
{
	std::vector<Vehicle> newVehicles;

	for (int i = 0; i < vehicles.size(); i++)
	{
		std::vector<Chromosome> a = vehicles[i].getGenome();
		std::vector<Chromosome> b = vehicles[(i+2)%vehicles.size()].getGenome();

		if (a.size() == b.size())
		{
			int split = Config::get()->randomInInterval(1,a.size()-1);
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


std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::TwoPointCrossOver(std::vector<Vehicle> vehicles){

	std::vector<Vehicle> newVehicles;

	for (int i = 0; i < vehicles.size(); i++)
	{
		std::vector<Chromosome> a = vehicles[i].getGenome();
		std::vector<Chromosome> b = vehicles[(i+2)%vehicles.size()].getGenome();

		if (a.size() == b.size())
		{
			int split1 = Config::get()->randomInInterval(1,a.size()-1);
			int split2 = Config::get()->randomInInterval(1,a.size()-1);

			if (split1 > split2){
				int tmp = split1;
				split1 = split2;
				split2 = tmp;
			}		

			for (int j = 0; j < a.size(); j++)
			{
				if (j >= split1 && j <= split2)
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