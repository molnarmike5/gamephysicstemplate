#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

struct Point {
	Vec3 m_position;
	Vec3 m_velocity;
	Vec3 m_force;
	bool m_isFixed;
	Vec3 m_ptemp;
	Vec3 m_vtemp;
	Point(Vec3 position, Vec3 velocity,Vec3 force, bool isFixed);
};

struct Spring {
	int m_point1;
	int m_point2;
	float m_initialLength;
	float m_currentLength;
	Spring(int point1, int point2, float initialLength, float currentLength);
};

class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

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

	// Addtional Functions
	void calculateElasticForces(bool tmp);
	void calculate_Positions_Euler(float timeStep);
	void calculate_Velocity_Euler(float timeStep);
	void calculate_Positions_Mid(float timeStep, bool tmp);
	void calculate_Velocity_Mid(float timeStep, bool tmp);
	void init();

private:
	// Data Attributes
	float m_fMass = 0;
	float m_fStiffness = 0;
	float m_fDamping = 0;
	int m_iIntegrator = 0;
	std::vector<Point> m_massPoints;
	std::vector<Spring> m_springs;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
};
#endif