#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include <d3d11.h>
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator(Vec3 gravity = Vec3());
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);
	void init_Demo2and3();
	void init_Demo4();

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	struct Point
	{
	public:
		Point(Vec3 pos, Vec3 velo, bool isFixed) : position{ pos }, velocity{ velo }, force{ Vec3() }, isFixed{ isFixed } {};
		Vec3 position;
		Vec3 velocity;
		Vec3 force;
		bool isFixed;
	};

	std::vector<Point> points;

	struct Spring
	{
	public:
		Spring(int p1, int p2, float inilength) : point1{ p1 }, point2{ p2 }, initialLength{ inilength } {};
		int point1;
		int point2;
		float initialLength;
		float currentLength;
	};
	std::vector<Spring> springs;

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	const Vec3 GRAVITY;
	void calculateEuler(float timestep);
	void calculateMidPoint(float timestep);
	int counter = 0;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif