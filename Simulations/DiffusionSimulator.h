#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Point
{
public:
	Point() = default;
	Point(Vec3 pos, float temp)
	: position(pos), temperature(temp) {}
	
	Vec3 position;
	float temperature = 0;
};

//implement your own grid class for saving grid data
class Grid {
public:
	// attributes
	int width, height;
	vector<vector<Point>> points;

	// Construtors
	Grid(int w, int h);

	int getWidth();
	int getHeight();
	void setWidth(int newWidth);
	void setHeight(int newHeight);
	vector<vector<Point>>& GetPoints();
	Point getPoint(int x, int y);
	
private:
	// Attributes

	// int addPoint(Point p);
	// int addPoint(Vec3 pos, float temp);
};

class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void getHeight(void* value, void* clientData);
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();
	void diffuseTemperatureExplicit();
	void diffuseTemperatureImplicit();
	void resize();
	void fillT();

private:
	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid *T; //save results of every time step
	float m_delta = 0.05;
	int old_width, old_height, old_delta;
};

#endif