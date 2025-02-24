#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h

#include <array>
#include "d3d11.h"
#include "Simulator.h"
#include "collisionDetect.h"

//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 

#define TESTCASEUSEDTORUNTEST 2

class RigidBodySystemSimulator:public Simulator{
public:
	// Construtors
	RigidBodySystemSimulator() = default;
	
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
		MassPoint() { MassPoint(Vec3()); }
		MassPoint(Vec3 pos) :
			position(pos) {}

		Vec3	position;
	};
	
	struct RigidBody
	{
		std::array<MassPoint, 8> calc_points();
		Mat4d calc_inertia_tensor_0();
		
		RigidBody(Vec3 position, Vec3 size, float mass) :
			orientation(0, 0, 0, 1),
			initial_orientation(0, 0, 0, 1),
			position(position),
			intial_position(position),
			size(size),
			mass(mass),
			externalForce(),
			angular_velocity(),
			angular_momentum()
		{
			points = calc_points();

			inertia_tensor_0_inverse = calc_inertia_tensor_0().inverse();


		}

		std::array<MassPoint, 8>	points;
		GamePhysics::Quat			orientation;
		GamePhysics::Quat			initial_orientation;
		Vec3						position;
		Vec3						intial_position;
		Vec3						size;
		Vec3						torque;
		Vec3						linear_velocity;
		Vec3						angular_velocity;
		Vec3						angular_momentum;
		Vec3						externalForce;
		Mat4d						inertia_tensor_0_inverse;
		Mat4d						obj2world;
		int							mass;
	};
  
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	vector<RigidBody>      rigid_bodies_;

	void calculateCollision(RigidBody& r1, RigidBody& r2, CollisionInfo& info);
	
	};

#endif