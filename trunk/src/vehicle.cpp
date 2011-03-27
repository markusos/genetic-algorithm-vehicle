#include "vehicle.h"
#include "config.h"
#include <iostream>
#include <Box2D\Common\b2Settings.h>

GA_VEHICLE::Vehicle::Vehicle(b2World* world, float mainPointsDistance, const std::vector<VehicleVertex>& vertices, const std::vector<Wheel>& wheels) : m_world(world), m_mainPointsDistance(mainPointsDistance), 
	m_vertices(vertices), m_wheels(wheels), m_bodys(std::vector<b2Body*>())
{	
}

GA_VEHICLE::Vehicle::Vehicle(b2World* world, float mainPointsDistance, int nrOfWheels) : m_world(world), m_mainPointsDistance(mainPointsDistance)
{	

	for(double i = 0; i < 2*b2_pi; i = i + b2_pi/4)
	{
		float sideLength = rand()/(double(RAND_MAX) + 1) * 4 + 1;
		m_vertices.push_back(VehicleVertex(0,sideLength,i));
	}	

	for(int i = 0; i < nrOfWheels; i++)
	{

		float wheelAngle = rand()/(double(RAND_MAX) + 1) * b2_pi * 2;
		int wheelPos = rand()%8;
		float wheelSize = rand()/(double(RAND_MAX) + 1) * (Config::get()->wheelMaxSize - Config::get()->wheelMinSize)  + Config::get()->wheelMinSize;
		float torque = rand()%(Config::get()->wheelTorqueMax - Config::get()->wheelTorqueMin) + Config::get()->wheelTorqueMin;
		float speed = rand()%(Config::get()->wheelSpeedMax - Config::get()->wheelSpeedMin)  + Config::get()->wheelSpeedMin;
		
		m_wheels.push_back(Wheel(wheelAngle,-speed,torque,wheelPos,wheelSize));
	}
}

GA_VEHICLE::Vehicle::Vehicle(b2World* world , std::vector<GA_VEHICLE::Chromosome> genome) : m_world(world)
{
	int nrOfvertices = 0;
	int nrOfWheels = 0;
	for (int i = 0; i < genome.size(); i++){
		if (genome[i].type == Chromosome::POINTDISTANCE) nrOfvertices++;
		if (genome[i].type == Chromosome::WHEELANGLE) nrOfWheels++;
	}

	if (nrOfvertices == 8){
		int j = 0;
		for(double i = 0; i < 2*b2_pi; i = i + b2_pi/4)
		{
			if(genome[j].type == Chromosome::POINTDISTANCE){
				m_vertices.push_back(VehicleVertex(0,genome[j].value,i));
			}
			j++;
		}
		for(int i = 0; i < nrOfWheels; i++)
		{
			float wheelAngle;
			int wheelPos;
			float wheelSize;
			float wheelTorque;
			float wheelSpeed;

			bool angle = false;
			bool pos = false;
			bool size = false;
			bool torque = false;
			bool speed = false;

			for(int a = 0; a < 5; a++)
			{
				if (genome[j].type == Chromosome::WHEELANGLE){
					wheelAngle = genome[j].value;
					angle = true;
				}
				if (genome[j].type == Chromosome::WHEELPOS){
					wheelPos = (int)(genome[j].value + 0.0001);
					pos = true;
				}
				if (genome[j].type == Chromosome::WHEELSIZE){
					wheelSize = genome[j].value;
					size = true;
				}
				if (genome[j].type == Chromosome::WHEELTORQUE){
					wheelTorque = genome[j].value;
					torque = true;
				}
				if (genome[j].type == Chromosome::WHEELSPEED){
					wheelSpeed = genome[j].value;
					speed = true;
				}
				j++;
			}
			if (angle && pos && size && torque && speed){
				m_wheels.push_back(Wheel(wheelAngle,-wheelSpeed,wheelTorque,wheelPos,wheelSize));
			}
			else std::cout << "ERROR: Wheels not correctly defined" << std::endl;
		}
	}
	else std::cout << "ERROR: To many sides on the vehicle" << std::endl;

	
	
}


GA_VEHICLE::Vehicle::~Vehicle()
{
}

void GA_VEHICLE::Vehicle::createWheel(Wheel& wheel)
{
	
	//wheel
	b2BodyDef wheelDef;
	wheelDef.type = b2_dynamicBody;
	float x = m_vehicleBody->GetPosition().x + (m_vertices[wheel.m_wheelPos].m_pointDistance) * cos(m_vertices[wheel.m_wheelPos].m_pointAngle) + ((wheel.m_wheelSize * 1.0f+1.0f) * cos(wheel.m_wheelAngle));
	float y = m_vehicleBody->GetPosition().y + (m_vertices[wheel.m_wheelPos].m_pointDistance) * sin(m_vertices[wheel.m_wheelPos].m_pointAngle) + ((wheel.m_wheelSize * 1.0f+1.0f) * sin(wheel.m_wheelAngle));
	wheelDef.position.Set(x, y);
	b2Body* wheelBody = m_world->CreateBody(&wheelDef);
	b2CircleShape wheelShape;
	wheelShape.m_radius = wheel.m_wheelSize;
	b2FixtureDef wheelfixtureDef;
	wheelfixtureDef.shape = &wheelShape;
	wheelfixtureDef.density = 0.5f;
	wheelfixtureDef.friction = 8.0f;
	wheelfixtureDef.restitution = 0.5f;
	wheelfixtureDef.filter.groupIndex = -1;
	wheelBody->CreateFixture(&wheelfixtureDef);
	m_bodys.push_back(wheelBody);

	//mount point
	b2BodyDef wheelMountBodyDef;
	wheelMountBodyDef.type = b2_dynamicBody;
	wheelMountBodyDef.position.Set(x, y);
	b2Body* wheelMountBody = m_world->CreateBody(&wheelMountBodyDef);
	b2CircleShape wheelMountShape;
	wheelMountShape.m_radius = wheel.m_wheelSize*0.7;
	b2FixtureDef wheelMountFixtureDef;
	wheelMountFixtureDef.shape = &wheelMountShape;
	wheelMountFixtureDef.density = 0.5f;
	wheelMountFixtureDef.filter.groupIndex = -1;
	wheelMountBody->CreateFixture(&wheelMountFixtureDef);
	m_bodys.push_back(wheelMountBody);

	//spring and damper
	float mountPointX = m_vehicleBody->GetPosition().x + m_vertices[wheel.m_wheelPos].m_pointDistance * cos(m_vertices[wheel.m_wheelPos].m_pointAngle);
	float mountPointY = m_vehicleBody->GetPosition().y + m_vertices[wheel.m_wheelPos].m_pointDistance * sin(m_vertices[wheel.m_wheelPos].m_pointAngle);
	b2Vec2 wheelSpringMountPoint = b2Vec2(mountPointX,mountPointY);

	b2DistanceJointDef springAndDamperJointDef;
	springAndDamperJointDef.Initialize(m_vehicleBody, wheelMountBody, wheelSpringMountPoint, wheelMountBody->GetWorldCenter());
	//springAndDamperJointDef.collideConnected = true;
	springAndDamperJointDef.frequencyHz = 5.0f;
	springAndDamperJointDef.dampingRatio = 0.5f;
	b2Joint* springAndDamperJoint = m_world->CreateJoint(&springAndDamperJointDef);

	//prismatic joint
	b2PrismaticJointDef wheelPrismaticJointDef;
	wheelPrismaticJointDef.Initialize(m_vehicleBody, wheelMountBody, wheelSpringMountPoint, b2Vec2( cos(wheel.m_wheelAngle), sin(wheel.m_wheelAngle)));
	wheelPrismaticJointDef.lowerTranslation = -0.4f;
	wheelPrismaticJointDef.upperTranslation = 0.4f;
	wheelPrismaticJointDef.enableLimit = true;
	wheelPrismaticJointDef.enableMotor = true;
	wheelPrismaticJointDef.maxMotorForce = 10.0f;
	wheelPrismaticJointDef.motorSpeed = 0.0f;
	b2Joint* wheelPrismaticJoint = m_world->CreateJoint(&wheelPrismaticJointDef);

	//wheel motor
	b2RevoluteJointDef wheelJointDef;
	wheelJointDef.Initialize(wheelMountBody, wheelBody, wheelBody->GetWorldCenter());
	wheelJointDef.maxMotorTorque = wheel.m_wheelTorque;
	wheelJointDef.motorSpeed = wheel.m_wheelSpeed;
	wheelJointDef.enableMotor = true;
	b2Joint* wheelJoint = m_world->CreateJoint(&wheelJointDef);
}

void GA_VEHICLE::Vehicle::addToWorld()
{
	//mainbody
	b2BodyDef vehicleBodyDef;
	vehicleBodyDef.type = b2_dynamicBody;
	vehicleBodyDef.position.Set(50.0f, 35.0f);
	m_vehicleBody = m_world->CreateBody(&vehicleBodyDef);

	b2PolygonShape vehicleShape;
	b2FixtureDef vehicleFixtureDef;
	vehicleFixtureDef.friction = 10.0f;
	vehicleFixtureDef.density = 1.0f;
	for(int i=0;i<m_vertices.size();i++)
	{
		float x0,x1,y0,y1;
		x0 = m_vertices[i].m_pointDistance * cos(m_vertices[i].m_pointAngle);
		x1 = m_vertices[(i+1)%m_vertices.size()].m_pointDistance * cos(m_vertices[(i+1)%m_vertices.size()].m_pointAngle);
		y0 = m_vertices[i].m_pointDistance * sin(m_vertices[i].m_pointAngle);
		y1 = m_vertices[(i+1)%m_vertices.size()].m_pointDistance * sin(m_vertices[(i+1)%m_vertices.size()].m_pointAngle);

		if(x0 != x1 && y0!= y1)
		{
			b2Vec2 triangle[] = {b2Vec2(0,0), b2Vec2(x0,y0), b2Vec2(x1,y1)};
			vehicleShape.Set(triangle,3);
			vehicleFixtureDef.shape = &vehicleShape;
			vehicleFixtureDef.filter.groupIndex = -1;
			m_vehicleBody->CreateFixture(&vehicleFixtureDef);
		}
	}
	m_bodys.push_back(m_vehicleBody);
	//wheels
	for(int i=0; i< m_wheels.size();i++)
	{
		createWheel(m_wheels[i]);
	}

	
}

void GA_VEHICLE::Vehicle::removeFromWorld()
{
	for(int i=0;i<m_bodys.size();i++)
	{
		m_world->DestroyBody(m_bodys[i]);
	}
	m_bodys.clear();
}

std::vector<GA_VEHICLE::Chromosome> GA_VEHICLE::Vehicle::getGenome(){
	std::vector<Chromosome> genome;
	for (int i = 0; i < m_vertices.size(); i++){
		Chromosome c;
		c.value = m_vertices[i].m_pointDistance;
		c.type = Chromosome::POINTDISTANCE;
		genome.push_back(c);
	}
	for (int i = 0; i < m_wheels.size(); i++)
	{
		Chromosome c;
		c.value = m_wheels[i].m_wheelAngle;
		c.type = Chromosome::WHEELANGLE;
		genome.push_back(c);

		c.value = m_wheels[i].m_wheelPos;
		c.type = Chromosome::WHEELPOS;
		genome.push_back(c);
		
		c.value = m_wheels[i].m_wheelSize;
		c.type = Chromosome::WHEELSIZE;
		genome.push_back(c);

		c.value = m_wheels[i].m_wheelTorque;
		c.type = Chromosome::WHEELTORQUE;
		genome.push_back(c);

		c.value = m_wheels[i].m_wheelSpeed;
		c.type = Chromosome::WHEELSPEED;
		genome.push_back(c);

	}
	return genome;
}