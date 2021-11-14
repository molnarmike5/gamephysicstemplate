#include "MASSSPRINGSYSTEMSIMULATOR.H"

MassSpringSystemSimulator::MassSpringSystemSimulator(Vec3 gravity) : GRAVITY { gravity }
{
	  
}

void MassSpringSystemSimulator::setMass(float mass)
{
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	this->points.emplace_back(position, Velocity, isFixed);
	return points.size() - 1;
}
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	this->springs.emplace_back(masspoint1, masspoint2, initialLength);
}
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return points.size();
}
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springs.size();
}
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return points[index].position;
}
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return points[index].velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "Euler, Midpoint, Leapfrog";
}
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	default:
		break;
	}
}
void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	/*std::cout << "Resetted!";
	points.clear();
	springs.clear();
	init();*/
}
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for each (Spring spring in springs)
	{
		
		DUC->beginLine();
		DUC->drawLine(points[spring.point1].position , Vec3(1, 0, 0), points[spring.point2].position, Vec3(1, .5, .5));
		DUC->endLine();
	}

	for each (Point point in points)
	{
		//Every point gets a nice sphere
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(point.position, Vec3(0.05, 0.05, 0.05));
	}

}
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	switch (testCase)
	{
	case 0:
		cout << "Euler!\n";
		m_iIntegrator = EULER;
		break;
	case 1:
		cout << "MidPoint!\n";
		m_iIntegrator = MIDPOINT;
		break;
	case 2:
		//No time to implement bonus
		cout << "LeapFrog!\n";
		m_iIntegrator = LEAPFROG;
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	
	switch (m_iIntegrator)
	{
	case EULER:
		calculateEuler(timeStep);
		break;
	case MIDPOINT:
		calculateMidPoint(timeStep);
		break;
	case LEAPFROG:
		break;
	default:
		break;
	}

}
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//Could not implement it succesfully
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	/*Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		//float inputScale = 0.0001f;
		float inputScale = 0.5f;
		inputWorld = inputWorld * inputScale;
		std::cout << inputWorld;
		for (auto point : points)
		{
			point.position = point.position + inputWorld;
		}
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		//m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	}*/
}

void MassSpringSystemSimulator::calculateEuler(float timestep)
{
	
	for (auto& spring : springs)
	{
		//Distance vector calculation
		Vec3 vector = points[spring.point1].position - points[spring.point2].position;
		float length = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
		Vec3 normalized = vector / length;

		//Elastic forces calculation
		points[spring.point1].force = -m_fStiffness * (length - spring.initialLength) * normalized;
		points[spring.point2].force = -points[spring.point1].force;

		//isFixed check
		if (!points[spring.point1].isFixed) {
			points[spring.point1].position = points[spring.point1].position + timestep * points[spring.point1].velocity;
		}
		if (!points[spring.point2].isFixed) {
		points[spring.point2].position = points[spring.point2].position + timestep * points[spring.point2].velocity;
		}

		//Collision with z plane
		if (points[spring.point1].position.y < -0.975) { points[spring.point1].position.y = -0.975; }
		if (points[spring.point2].position.y < -0.975) { points[spring.point2].position.y = -0.975; }

		//Acceleration calculation based on elastic force and gravity
		Vec3 acc1 = (points[spring.point1].force / m_fMass) + GRAVITY;
		Vec3 acc2 = (points[spring.point2].force / m_fMass) + GRAVITY;

		//Velocity calculation
		points[spring.point1].velocity = points[spring.point1].velocity + acc1 * timestep;
		points[spring.point2].velocity = points[spring.point2].velocity + acc2 * timestep;

		//Clear elastic forces
		points[spring.point1].force = Vec3(0.0f, 0.0f, 0.0f);
		points[spring.point2].force = Vec3(0.0f, 0.0f, 0.0f);
	}
}

void MassSpringSystemSimulator::calculateMidPoint (float timestep)
{
	for (auto& spring : springs)
	{

		Vec3 pos1_tmp;
		Vec3 pos2_tmp;
		Vec3 vtemp1;
		Vec3 vtemp2;

		//isFixed check and position at timestep/2
		if (!points[spring.point1].isFixed) {
			pos1_tmp = points[spring.point1].position + timestep/2 * points[spring.point1].velocity;
		}
		if (!points[spring.point2].isFixed) {
			pos2_tmp = points[spring.point2].position + timestep/2 * points[spring.point2].velocity;
		}

		//Collision with z plane
		if (pos1_tmp.y < -0.975) { pos1_tmp.y = -0.975; }
		if (pos2_tmp.y < -0.975) { pos2_tmp.y = -0.975; }

		//First distance vector calculation
		Vec3 vector = points[spring.point1].position - points[spring.point2].position;
		float length = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
		Vec3 normalized = vector / length;

		//First elastic forces calculation
		points[spring.point1].force = -m_fStiffness * (length - spring.initialLength) * normalized;
		points[spring.point2].force = -points[spring.point1].force;

		//First acceleration calculation
		Vec3 acc1_tmp = (points[spring.point1].force / m_fMass);
		Vec3 acc2_tmp = (points[spring.point2].force / m_fMass);

		//Velocity calculation at timestep/2
		vtemp1 = points[spring.point1].velocity + acc1_tmp * timestep/2;
		vtemp2 = points[spring.point2].velocity + acc2_tmp * timestep/2;

		//isFixed check and position at timestep 
		if (!points[spring.point1].isFixed) {
			points[spring.point1].position = points[spring.point1].position + timestep * vtemp1;
		}
		if (!points[spring.point2].isFixed) {
			points[spring.point2].position = points[spring.point2].position + timestep * vtemp2;
		}

		//Collision with z plane again
		if (points[spring.point1].position.y < -0.975) { points[spring.point1].position.y = -0.975; }
		if (points[spring.point2].position.y < -0.975) { points[spring.point2].position.y = -0.975; }

		//Second distance vector calculation based on postiions at timestep/2
		vector = pos1_tmp - pos2_tmp;
		length = sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
		normalized = vector / length;

		//Second elastic forces calculation
		points[spring.point1].force = -m_fStiffness * (length - spring.initialLength) * normalized;
		points[spring.point2].force = -points[spring.point1].force;

		//Acceleration including gravity
		Vec3 acc1 = (points[spring.point1].force / m_fMass) + GRAVITY;
		Vec3 acc2 = (points[spring.point2].force / m_fMass) + GRAVITY;

		//Velocity calculation
		points[spring.point1].velocity = points[spring.point1].velocity + acc1 * timestep;
		points[spring.point2].velocity = points[spring.point2].velocity + acc2 * timestep;

		//Clear forces
		points[spring.point1].force = Vec3(0, 0, 0);
		points[spring.point2].force = Vec3(0, 0, 0);

	}




}

void MassSpringSystemSimulator::init_Demo4()
{
	//Complex simulation
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	int p1 = addMassPoint(Vec3(0, 0.5, 0), Vec3(0, -1, 0), false);
	int p2 = addMassPoint(Vec3(0.25, 1, 1), Vec3(1, -1, 1), true);
	int p3 = addMassPoint(Vec3(0.75, 0.5, 0), Vec3(0, -1, 2), false);
	int p4 = addMassPoint(Vec3(2, 0.5, 0), Vec3(0, -1, 0), false);
	int p5 = addMassPoint(Vec3(1, 1, 1), Vec3(1, 0, 1), false);
	int p6 = addMassPoint(Vec3(0, 0.5, 0.25), Vec3(-1, -1, 0), false);
	int p7 = addMassPoint(Vec3(1, 0.5, 1), Vec3(1, 1, 1), false);
	int p8 = addMassPoint(Vec3(0.5, 0.5, 0.5), Vec3(0.5, 0.5, 0.5), true);
	int p9 = addMassPoint(Vec3(0.75, 0.75, 0.75), Vec3(0.5, -1.5, 0), false);
	int p10 = addMassPoint(Vec3(0.25, 0.25, 0.25), Vec3(0.25, -1, 0.5), false);
	addSpring(p0, p1, 1);
	addSpring(p0, p5, 3);
	addSpring(p7, p1, 6);
	addSpring(p5, p3, 5);
	addSpring(p2, p4, 2);
	addSpring(p2, p3, 1);
	addSpring(p2, p8, 2);
	addSpring(p9, p8, 4);
	addSpring(p1, p2, 1);
	addSpring(p4, p7, 1);
	addSpring(p4, p9, 3);
	setStiffness(40);
	setMass(10);
}

void MassSpringSystemSimulator::init_Demo2and3()
{
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	addSpring(p0, p1, 1);
	setStiffness(40);
	setMass(10);
}







