#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

GamePhysics::Vec3 mapToColor(float temp);
GamePhysics::Vec3 getColor(float temp);

Grid::Grid(int w, int h) : width(w), height(h), points(height, vector<Point>(width)) {
}

int Grid::getWidth()
{
	return width;
}

int Grid::getHeight()
{
	return height;
}

void Grid::setHeight(int newHeight)
{
	height = newHeight;
}

void Grid::setWidth(int newWidth)
{
	width = newWidth;
}

Point& Grid::getPoint(int x, int y)
{
	return points[y][x];
}

vector<vector<Point>>& Grid::getPoints()
{
	return points;
}

DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	T = new Grid(16, 16);
	fillT();
	// to be implemented
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	// to be implemented
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float timeStep) {

	float alpha = 0.005f;

	float delta = T->getPoint(1,0).position[0] - T->getPoint(0, 0).position[0];
	delta *= delta;

	for (size_t i = 0; i < T->getHeight(); i++) {
		for (size_t j = 0; j < T->getWidth(); j++) {
			Point* point = &(T->getPoint(i, j));

			// Check for boundary
			if (i == 0 || i == T->getHeight() - 1 || j == 0 || j == T->getWidth() - 1) {
				point->temperature = 1;
			}
			else {
				
				float temp_x = (T->getPoint(i + 1, j).temperature - 2 * point->temperature + T->getPoint(i - 1, j).temperature) / delta;				
				float temp_y = (T->getPoint(i, j + 1).temperature - 2 * point->temperature + T->getPoint(i, j - 1).temperature) / delta;
				float delta_t = alpha *  (temp_x + temp_y);
				point->temperature = point->temperature + timeStep * delta_t;
			}
		}
	}

	return T;
}

void setupB(std::vector<Real>& b) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < 25; i++) {
		b.at(i) = 0;
	}
}

void DiffusionSimulator::fillT() {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero

	float scale = 0.1f;

	for (int i = 0; i < T->getHeight(); ++i)
	{
		for (int j = 0; j < T->getWidth(); ++j)
		{
			T->getPoint(i, j) = Point(Vec3((i - T->getHeight() / 2) * scale, (j - T->getWidth() / 2) * scale, 0), 0.0f);
		}
	}
}

void setupA(SparseMatrix<Real>& A, double factor) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < 25; i++) {
		A.set_element(i, i, 1); // set diagonal
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit() {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = 25;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT();//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization

	for (auto vector : T->getPoints())
	{
		for (const auto& point : vector)
		{
			//Temperature Calculation
			Vec3 temp1 = mapToColor(point.temperature);
			Vec3 temp2 = mapToColor(point.temperature + 0.1f);
			auto factor = fmod(floorf(100 * point.temperature), 10);


				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, temp1);
			DUC->drawSphere(point.position, Vec3(0.02));
		}
	}

}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}


GamePhysics::Vec3 mapToColor(float temp) {
	Vec3 temp1 = getColor(temp);
	if (temp == 0.0 || temp == 1.0) return temp1;
	Vec3 temp2 = getColor(temp + 0.1f);
	Vec3 temperature{};
	float x1 = floorf(temp * 10);
	float x2 = ceilf(temp * 10);
	for (size_t i = 0; i < 3; i++) {
		temperature[i] = temp1[i] + (10*temp - x1) * (temp2[i]-temp1[i]) / (x2 - x1);
	}
	return temperature;
}

GamePhysics::Vec3 getColor(float temp) {
	float r = 0, g = 0, b = 0;
	if (temp < 0.1) {
		r = 5, g = 48, b = 97;
	}
	else if (temp < 0.2) {
		r = 33, g = 102, b = 172;
	}
	else if (temp < 0.3) {
		r = 67, g = 147, b = 195;
	}
	else if (temp < 0.4) {
		r = 146, g = 197, b = 222;
	}
	else if (temp < 0.5) {
		r = 209, g = 229, b = 240;
	}
	else if (temp < 0.6) {
		r = 253, g = 219, b = 199;
	}
	else if (temp < 0.7) {
		r = 244, g = 165, b = 130;
	}
	else if (temp < 0.8) {
		r = 214, g = 96, b = 77;
	}
	else if (temp < 0.9) {
		r = 178, g = 24, b = 43;
	}
	else {
		r = 103, g = 0, b = 31;
	}
	return Vec3(r / 256.0f, g / 256.0f, b / 256.0f);
}
