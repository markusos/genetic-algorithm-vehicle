#ifndef __VEHICLE_H__
#define __VEHICLE_H__

#include <vector>
#include <Box2D/Box2D.h>

namespace GA_VEHICLE
{

struct Wheel
{
	Wheel(float wheelAngle, float wheelSpeed, float wheelTorque, int wheelPos, float wheelSize) : m_wheelAngle(wheelAngle), m_wheelSpeed(wheelSpeed), m_wheelTorque(wheelTorque), m_wheelPos(wheelPos), m_wheelSize(wheelSize){}
	float m_wheelAngle;
	float m_wheelSpeed;
	float m_wheelTorque;
	int m_wheelPos;
	float m_wheelSize;
};

struct VehicleVertex
{
	VehicleVertex(int mainPointIndex, float pointDistance, float pointAngle) : m_mainPointIndex(mainPointIndex), m_pointDistance(pointDistance), m_pointAngle(pointAngle) {}
	int m_mainPointIndex;
	float m_pointDistance;
	float m_pointAngle;
};

struct Chromosome
{
	float value;
	enum Type{POINTDISTANCE, POINTANGLE, WHEELANGLE, WHEELSPEED, WHEELTORQUE, WHEELPOS, WHEELSIZE};
	Type type;
};

class Vehicle
{
public:
	float m_mainPointsDistance;
	std::vector<VehicleVertex> m_vertices;
	std::vector<Wheel> m_wheels;
	b2World* m_world;
	b2Body* m_vehicleBody;

	float m_fitness;

	Vehicle(b2World* world, float mainPointsDistance, const std::vector<VehicleVertex>& vertices, const std::vector<Wheel>& wheels);
	Vehicle(b2World* world, float mainPointsDistance, int nrOfWheels);
	Vehicle(std::vector<GA_VEHICLE::Chromosome> genome);
	void addToWorld();
	void removeFromWorld();
	std::vector<Chromosome> getGenome();
	~Vehicle();
private:
	void createWheel(Wheel& wheel);
	std::vector<b2Body*> m_bodys;
};


}
#endif