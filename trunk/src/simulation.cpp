#include "simulation.h"
#include "config.h"
#include <GL/glfw.h>
#include <GL/glut.h>

#include <iostream>

GA_VEHICLE::Simulation::Simulation() : m_time(0), m_timeStep(1.0/60.0), m_render(true), m_stepsPerRenderFrame(3)
{
	srand(glfwGetTime());
	b2Vec2 gravity(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(gravity, doSleep);
	m_currentVehicle = 0;
	m_generationCounter = 1;

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
				m_world->Step(m_timeStep, velocityIterations, positionIterations);
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
		m_world->Step(m_timeStep, velocityIterations, positionIterations);
		m_world->ClearForces();
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
			m_currentVehicle = 0;
			m_generationCounter++;
			m_population = mutation( crossOver( selection(m_population)));
			m_population[m_currentVehicle].addToWorld();
			//Init evaluateVehicleAbortCondition variables
			oldPosX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
			oldTime = glfwGetTime();
		}

		if(stepPhysics()) // physics "stepped"
		{
			if(m_render)
			{
				render();
			}
			if(evaluateVehicleAbortCondition(m_population[m_currentVehicle]))
			{
				std::cout << "abort == true, generation == "<< m_generationCounter << std::endl;
				m_population[m_currentVehicle].removeFromWorld();
				m_population[m_currentVehicle].m_fitness = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
				m_currentVehicle++;
				if(m_currentVehicle < m_population.size())
				{
					m_population[m_currentVehicle].addToWorld();
					//Init evaluateVehicleAbortCondition variables
					oldPosX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
					oldTime = glfwGetTime();
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
	float groundPoints[] = {200, 20, 22, 24, 23, 24, 26, 22, 24, 25, 28, 30, 30, 28, 24, 28, 30, 30, 100};
	for(int i=0; i< 19-1;i++)
	{
		b2Vec2 box[] = {b2Vec2(i*10+30,groundPoints[i]), b2Vec2(i*10+30,groundPoints[i]-2), b2Vec2(i*10+30+10,groundPoints[i+1]-2), b2Vec2(i*10+30+10,groundPoints[i+1])};
		groundBox.Set(box,4);
		groundBody->CreateFixture(&groundBox, 0.0f);
	}

}

void GA_VEHICLE::Simulation::initRandomPopulation()
{
	float pi = 3.1415;
	for(int i=0;i<populationSize;i++)
	{
		m_population.push_back(Vehicle(m_world, 0, 4));
	}
	/*
	Vehicle vehicle3 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle3);

	Vehicle vehicle4 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle4);

	Vehicle vehicle5 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle5);

	Vehicle vehicle6 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle6);

	Vehicle vehicle7 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle7);

	Vehicle vehicle8 = Vehicle(m_world, 0, 4);
	m_population.push_back(vehicle8);*/
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

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::selection(std::vector<Vehicle>& vehicles)
{
	//binary tournament selection

	std::vector<Vehicle> toCrossOver;

	for(int i=0;i<toCrossOverSize;i++)
	{
		int aPos = rand() % vehicles.size();
		Vehicle a = vehicles[aPos];
		vehicles.erase(vehicles.begin()+ aPos);

		int bPos = rand() % vehicles.size();
		Vehicle b = vehicles[bPos];
		vehicles.erase(vehicles.begin()+ bPos);

		if(a.m_fitness >= b.m_fitness)
		{
			toCrossOver.push_back(a);
			vehicles.push_back(b);
		}
		else
		{
			toCrossOver.push_back(b);
			vehicles.push_back(a);
		}
	}
	return toCrossOver;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::crossOver(std::vector<Vehicle>& vehicles)
{
	std::vector<Vehicle> newVehicles;
	
	for (int i = 0; i < vehicles.size(); i++)
	{
		std::vector<GA_VEHICLE::Chromosome> a = vehicles[i].getGenome();
		std::vector<GA_VEHICLE::Chromosome> b = vehicles[(i+1)%vehicles.size()].getGenome();
		if (a.size() == b.size())
		{
			int split = rand()%a.size();
			for (int j = 0; j < a.size(); j++)
			{
				if (j >= split) std::swap(a[j], b[j]);
			}
			newVehicles.push_back(Vehicle(a));
			newVehicles.push_back(Vehicle(b));
		}
		else std::cout << "ERROR: Genomes of different size" << std::endl;
	}

	return newVehicles;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::mutation(std::vector<Vehicle>& vehicles)
{
	std::vector<Vehicle> newVehicles;
	for(int i=0;i<vehicles.size();i++)
	{
		std::vector<Chromosome> genome = vehicles[i].getGenome();
		for(int j=0;j<genome.size();j++)
		{
			if(rand() <=mutationChance)
			{
				if(rand() <= 0.5)
					genome[i].value += mutationFactor*2;//tmp
				else
					genome[i].value -= mutationFactor*2;//tmp
			}
		}
		newVehicles.push_back(Vehicle(genome));
	}
	return vehicles;
}

bool GA_VEHICLE::Simulation::evaluateVehicleAbortCondition(Vehicle& vehicle)
{
	float32 posX = m_population[m_currentVehicle].m_vehicleBody->GetPosition().x;
	double timeNow = glfwGetTime();

	double allowedStandStillTime = 3;
	float32 minMove = 5;

	if ((timeNow - oldTime) > allowedStandStillTime){
		if (abs(posX - oldPosX) < minMove) return true;
		else{
			oldTime = timeNow;
			oldPosX = posX;
			return false;
		}
	}
	else{
		return false;
	}
}