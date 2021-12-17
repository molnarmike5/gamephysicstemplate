#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(int w, int h) : width(w), height(h), points(height, vector<Point>(width))
{
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

Point Grid::getPoint(int x, int y)
{
    return points[y][x];
}

vector<vector<Point>>& Grid::GetPoints()
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

const char* DiffusionSimulator::getTestCasesStr()
{
    return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset() 
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

    diffuseTemperatureExplicit();
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
    // to be implemented

    switch (m_iTestCase)
    {
    case 0:
        TwAddVarRW(DUC->g_pTweakBar, "Height", TW_TYPE_INT32, &T->height, "min=1 step=1");
        TwAddVarRW(DUC->g_pTweakBar, "Width", TW_TYPE_INT32, &T->width, "min=1 step=1");
        TwAddVarRW(DUC->g_pTweakBar, "Space between", TW_TYPE_FLOAT, &m_delta, "min=0.01 step=0.01");

        break;
    case 1:
        break;
    }
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
        diffuseTemperatureExplicit();
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

void setupB(std::vector<Real>& b)
{
    //add your own parameters
    // to be implemented
    //set vector B[sizeX*sizeY]
    for (int i = 0; i < 25; i++)
    {
        b.at(i) = 0;
    }
}

void DiffusionSimulator::fillT()
{
    //add your own parameters
    // to be implemented
    //fill T with solved vector x
    //make sure that the temperature in boundary cells stays zero

    float currentX = 0, currentY = 0, temp;
    const auto offset = Vec3(T->getWidth() / 2 * m_delta, T->getHeight() / 2 * m_delta, 0);

    for (int i = 0; i < T->getHeight(); ++i)
    {
        for (int j = 0; j < T->getWidth(); ++j)
        {
            if (j == 0 || i == 0 || j == T->getWidth() - 1 || i == T->getHeight() - 1) temp = 0;
            else temp = 1;
            // temp = currentX * currentY * 30;

            T->points[i][j] = Point(Vec3(currentX, currentY, 0) - offset, temp);
            currentX += m_delta;
        }

        currentX = 0;
        currentY += m_delta;
    }
}

void setupA(SparseMatrix<Real>& A, double factor)
{
    //add your own parameters
    // to be implemented
    //setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
    // set with:  A.set_element( index1, index2 , value );
    // if needed, read with: A(index1, index2);
    // avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
    for (int i = 0; i < 25; i++)
    {
        A.set_element(i, i, 1); // set diagonal
    }
}

void DiffusionSimulator::diffuseTemperatureImplicit()
{
    //add your own parameters
    // solve A T = b
    // to be implemented
    const int N = 25; //N = sizeX*sizeY*sizeZ
    SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
    std::vector<Real>* b = new std::vector<Real>(N);

    setupA(*A, 0.1);
    setupB(*b);

    // perform solve
    Real pcg_target_residual = 1e-05;
    Real pcg_max_iterations = 1000;
    Real ret_pcg_residual = 1e10;
    int ret_pcg_iterations = -1;

    SparsePCGSolver<Real> solver;
    solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

    std::vector<Real> x(N);
    for (int j = 0; j < N; ++j) { x[j] = 0.; }

    // preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
    solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
    // x contains the new temperature values
    fillT(); //copy x to T
}

void DiffusionSimulator::resize()
{
}

void DiffusionSimulator::simulateTimestep(float timeStep)
{
    // to be implemented
    // update current setup for each frame
    switch (m_iTestCase)
    {
    case 0:
        if (T->width != old_width)
        {
            int diff = T->width - old_width;
            for (int i = 0; i < T->height; ++i)
            {
                for (int j = 0; j < diff; ++j)
                {
                    T->points[i].push_back(Point());
                }

                for (int j = 0; j < -diff; ++j)
                {
                    T->points[i].erase(T->points[i].end() - 1);
                }
            }

            fillT();
            old_width = T->width;
        }
        else if (T->height != old_height)
        {
            int diff = T->height - old_height;
            for (int i = 0; i < diff; ++i)
            {
                T->points.push_back(vector<Point>(T->width));
            }
            
            for (int i = 0; i < -diff; ++i)
            {
                T->points.erase(T->points.end() - 1);
            }
            
            fillT();
            old_height = T->height;
        }
        else if (m_delta != old_delta)
        {
            fillT();
            old_delta = m_delta;
        }

        break;
    case 1:
        diffuseTemperatureImplicit();
        break;
    }
}

void DiffusionSimulator::drawObjects()
{
    for (auto vector : T->GetPoints())
    {
        for (auto point : vector)
        {
            const auto color = Vec3(1, 1 - point.temperature, 1 - point.temperature);
            DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * color);
            DUC->drawSphere(point.position, Vec3(0.01));
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