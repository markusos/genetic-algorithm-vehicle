#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <Box2D/Box2D.h>
#include "Renderer.h"
#include "vehicle.h"
#include <vector>

namespace GA_VEHICLE
{
	class Simulation
	{
	public:
		Simulation();
		~Simulation();
		bool stepPhysics();
		void mainLoop();
		void render();
	protected:
		void addTests();
		void initRandomPopulation();
		std::vector<Vehicle> selection(std::vector<Vehicle>& vehicles);
		std::vector<Vehicle> crossOver(std::vector<Vehicle>& vehicles);
		std::vector<Vehicle> mutation(std::vector<Vehicle>& vehicles);
		bool evaluateVehicleAbortCondition(Vehicle& vehicle);

		Renderer* m_renderer;
		b2World* m_world;
		double m_time;
		double m_timeStep;
		bool m_render;
		int m_stepsPerRenderFrame;
		b2Body* m_body;
		std::vector<Vehicle> m_population;
		int m_currentVehicle;
		int m_generationCounter;
	};
}

#endif