#include "vehicle.h"
#include <iostream>

GA_VEHICLE::Vehicle::Vehicle(b2World* world, float mainPointsDistance, const std::vector<VehicleVertex>& vertices, const std::vector<Wheel>& wheels) : m_world(world), m_mainPointsDistance(mainPointsDistance), 
	m_vertices(vertices), m_wheels(wheels), m_bodys(std::vector<b2Body*>())
{

	
}
GA_VEHICLE::Vehicle::~Vehicle()
{
}

void GA_VEHICLE::Vehicle::createWheel(Wheel& wheel)
{
	//wheel
	b2BodyDef wheelDef;
	wheelDef.type = b2_dynamicBody;
	std::cout << m_vehicleBody->GetPosition().x << " " << m_vehicleBody->GetPosition().y <<  std::endl;
	float x = m_vehicleBody->GetPosition().x + (m_vertices[wheel.m_wheelPos].m_pointDistance) * cos(m_vertices[wheel.m_wheelPos].m_pointAngle) + wheel.m_wheelSize * 1.5f * cos(wheel.m_wheelAngle);
	float y = m_vehicleBody->GetPosition().y + (m_vertices[wheel.m_wheelPos].m_pointDistance) * sin(m_vertices[wheel.m_wheelPos].m_pointAngle) + wheel.m_wheelSize * 1.5f * sin(wheel.m_wheelAngle);
	wheelDef.position.Set(x, y);
	b2Body* wheelBody = m_world->CreateBody(&wheelDef);
	b2CircleShape wheelShape;
	wheelShape.m_radius = wheel.m_wheelSize;
	b2FixtureDef wheelfixtureDef;
	wheelfixtureDef.shape = &wheelShape;
	wheelfixtureDef.density = 1.0f;
	wheelfixtureDef.friction = 5.0f;
	wheelfixtureDef.restitution = 0.5f;
	wheelBody->CreateFixture(&wheelfixtureDef);
	m_bodys.push_back(wheelBody);

	//mount point
	b2BodyDef wheelMountBodyDef;
	wheelMountBodyDef.type = b2_dynamicBody;
	wheelMountBodyDef.position.Set(x, y);
	b2Body* wheelMountBody = m_world->CreateBody(&wheelMountBodyDef);
	b2CircleShape wheelMountShape;
	wheelMountShape.m_radius = 0.2f;
	b2FixtureDef wheelMountFixtureDef;
	wheelMountFixtureDef.shape = &wheelMountShape;
	wheelMountFixtureDef.density = 1.0f;
	wheelMountBody->CreateFixture(&wheelMountFixtureDef);
	m_bodys.push_back(wheelMountBody);

	//spring and damper
	float mountPointX = m_vehicleBody->GetPosition().x + m_vertices[wheel.m_wheelPos].m_pointDistance * cos(m_vertices[wheel.m_wheelPos].m_pointAngle);
	float mountPointY = m_vehicleBody->GetPosition().y + m_vertices[wheel.m_wheelPos].m_pointDistance * sin(m_vertices[wheel.m_wheelPos].m_pointAngle);
	b2Vec2 wheelSpringMountPoint = b2Vec2(mountPointX,mountPointY);
	//b2Vec2 wheelSpringMountPoint = m_vehicleBody->GetWorldCenter();
	//wheelSpringMountPoint += b2Vec2(3,-1);

	/*b2DistanceJointDef springAndDamperJointDef;
	springAndDamperJointDef.Initialize(m_vehicleBody, wheelMountBody, wheelSpringMountPoint, wheelMountBody->GetWorldCenter());
	springAndDamperJointDef.collideConnected = true;
	springAndDamperJointDef.frequencyHz = 1.0f;
	springAndDamperJointDef.dampingRatio = 3.0f;
	b2Joint* springAndDamperJoint = m_world->CreateJoint(&springAndDamperJointDef);*/

	//prismatic joint
	b2PrismaticJointDef wheelPrismaticJointDef;
	wheelPrismaticJointDef.Initialize(m_vehicleBody, wheelMountBody, wheelSpringMountPoint, b2Vec2( cos(wheel.m_wheelAngle), sin(wheel.m_wheelAngle)));
	wheelPrismaticJointDef.lowerTranslation = -0.2f;
	wheelPrismaticJointDef.upperTranslation = 0.2f;
	wheelPrismaticJointDef.enableLimit = true;
	wheelPrismaticJointDef.enableMotor = true;
	wheelPrismaticJointDef.maxMotorForce = 1000.0f;
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

	vehicleFixtureDef.density = 1.0f;
	for(int i=0;i<m_vertices.size();i++)
	{
		float x0,x1,y0,y1;
		x0 = m_vertices[i].m_pointDistance * cos(m_vertices[i].m_pointAngle);
		x1 = m_vertices[(i+1)%m_vertices.size()].m_pointDistance * cos(m_vertices[(i+1)%m_vertices.size()].m_pointAngle);
		y0 = m_vertices[i].m_pointDistance * sin(m_vertices[i].m_pointAngle);
		y1 = m_vertices[(i+1)%m_vertices.size()].m_pointDistance * sin(m_vertices[(i+1)%m_vertices.size()].m_pointAngle);

		std::cout << m_vehicleBody->GetWorldCenter().x <<" "<< x0 <<" " << x1<< std::endl;
		if(x0 != x1 && y0!= y1)
		{
			b2Vec2 triangle[] = {b2Vec2(0,0), b2Vec2(x0,y0), b2Vec2(x1,y1)};
			vehicleShape.Set(triangle,3);
			vehicleFixtureDef.shape = &vehicleShape;
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