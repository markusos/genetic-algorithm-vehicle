#include "simulation.h"

#include <GL/glfw.h>
#include <GL/glut.h>

#include <iostream>

GA_VEHICLE::Simulation::Simulation() : m_time(0), m_timeStep(1.0/60.0), m_render(true), m_stepsPerRenderFrame(3)
{
	b2Vec2 gravity(0.0f, -10.0f);
	bool doSleep = true;
	m_world = new b2World(gravity, doSleep);
	m_currentVehicle = 0;
	m_generationCounter = 1;

	addTests();

	if(m_render)
	{
		m_renderer = new Renderer(m_world, m_body);
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
	const int velocityIterations = 10;
	const int positionIterations = 10;

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
		}

		if(stepPhysics()) // physics "stepped"
		{
			if(m_render)
			{
				render();
			}
			//if(evaluateVehicleAbortCondition(m_population[m_currentVehicle]))
			//{
			//	m_currentVehicle++;
			//}
			glfwSwapBuffers();// otherwise keyboard etc wont work..
		}
	}
}

void GA_VEHICLE::Simulation::render()
{
	m_renderer->display();
}

void GA_VEHICLE::Simulation::addTests()
{
	b2BodyDef groundBodyDef;
	b2Body* groundBody = m_world->CreateBody(&groundBodyDef);
	b2PolygonShape groundBox;
	float groundPoints[] = {20, 20, 22, 24, 23, 24, 26, 22, 24, 25, 28, 30, 30, 28, 24, 30};
	for(int i=0; i< 16-1;i++)
	{
		b2Vec2 box[] = {b2Vec2(i*10+30,groundPoints[i]), b2Vec2(i*10+30,groundPoints[i]-2), b2Vec2(i*10+30+10,groundPoints[i+1]-2), b2Vec2(i*10+30+10,groundPoints[i+1])};
		groundBox.Set(box,4);
		groundBody->CreateFixture(&groundBox, 0.0f);
	}

	float pi = 3.1415;
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
	Vehicle* vehicle = new Vehicle(m_world, 0,vertices,wheels);

	m_body = vehicle->m_vehicleBody;
}

void GA_VEHICLE::Simulation::initRandomPopulation()
{

}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::selection(std::vector<Vehicle>& vehicles)
{
	return vehicles;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::crossOver(std::vector<Vehicle>& vehicles)
{
	return vehicles;
}

std::vector<GA_VEHICLE::Vehicle> GA_VEHICLE::Simulation::mutation(std::vector<Vehicle>& vehicles)
{
	return vehicles;
}

bool GA_VEHICLE::Simulation::evaluateVehicleAbortCondition(Vehicle& vehicle)
{
	return false;
}