#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include <array>
#include "d3d11.h"
#include "SimpleMath.h"
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator();
	
	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	void addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i,Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);


private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	
	struct MassPoint
	{
		MassPoint(Vec3 pos) :
			position(pos) {}

		Vec3	position;
	};
	
	struct RigidBody
	{
		static std::array<MassPoint, 8> calc_points(Vec3 box_center, Vec3 size);
		
		RigidBody(Vec3 initialOrientation, Vec3 box_center, Vec3 size, float mass) :
			points(),
			orientation(SimpleMath::Quaternion<float>(initialOrientation.x, initialOrientation.y, initialOrientation.z, 0.0)),
			box_center(box_center),
			size(size),
			mass(mass) { points = calc_points(box_center, size); }

		std::array<MassPoint, 8>	points;
		Quaternion<float>			orientation;
		Vec3						box_center;
		Vec3						size;
		float						mass;
	};
  
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	};

#endif