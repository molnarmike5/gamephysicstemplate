#include "MassSpringSystemSimulator.h"

Point::Point(Vec3 position, Vec3 velocity, Vec3 force, bool isFixed) : m_position{ position }, m_velocity{ velocity }, m_force{ force }, m_isFixed{ isFixed } {
}

Spring::Spring(int point1, int point2, float initialLength, float currentLength) : m_point1(point1), m_point2(point2), m_initialLength(initialLength), m_currentLength(currentLength) {

}

MassSpringSystemSimulator::MassSpringSystemSimulator() : m_massPoints{}, m_springs{}
{
	
}

const char* MassSpringSystemSimulator::getTestCasesStr()
{
	return "MassSpringSystemSimulator";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (auto& spring : this->m_springs) {
		DUC->beginLine();
		DUC->drawLine(m_massPoints[spring.m_point1].m_position, Vec3{ 1,1,1 }, m_massPoints[spring.m_point2].m_position, Vec3{ 1,1,1 });
		DUC->endLine();
	}
	for (auto& point : this->m_massPoints) {
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(point.m_position, Vec3{ 0.05,0.05,0.05 });
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {

	switch (m_iIntegrator) {
	case EULER:
		calculateElasticForces(false);
		calculate_Positions_Euler(timeStep);
		calculate_Velocity_Euler(timeStep);
		break;
	case MIDPOINT:
		calculate_Positions_Mid(timeStep / 2, true);
		calculateElasticForces(false);
		calculate_Velocity_Mid(timeStep / 2, true);
		calculate_Positions_Mid(timeStep, false);
		calculateElasticForces(true);
		calculate_Velocity_Mid(timeStep, false);
		break;
	}
}

void MassSpringSystemSimulator::calculateElasticForces(bool tmp) {
	// Set forces to 0
	for (auto& p : m_massPoints) {
		p.m_force = 0;
	}
	for (auto& spring : m_springs) {
		Point* p1 = &m_massPoints[spring.m_point1];
		Point* p2 = &m_massPoints[spring.m_point2];
		auto vector = tmp ? m_massPoints[spring.m_point1].m_ptemp - m_massPoints[spring.m_point2].m_ptemp : m_massPoints[spring.m_point1].m_position - m_massPoints[spring.m_point2].m_position;
		spring.m_currentLength = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
		auto distance = tmp ? p1->m_ptemp - p2->m_ptemp : p1->m_position - p2->m_position;
		auto force = m_fStiffness * (spring.m_currentLength - spring.m_initialLength) * distance / spring.m_currentLength;
		p1->m_force -= force;
		p2->m_force += force;
	}
}

void MassSpringSystemSimulator::calculate_Positions_Euler(float timeStep) {
	for (auto& point : m_massPoints) {
		if (!point.m_isFixed) {
			point.m_position = point.m_position + timeStep * point.m_velocity;
		}
	}
}

void MassSpringSystemSimulator::calculate_Velocity_Euler(float timeStep) {
	for (auto& point : m_massPoints) {
		if (!point.m_isFixed) {
			point.m_velocity = point.m_velocity + timeStep * point.m_force / m_fMass;
		}
	}
}

void MassSpringSystemSimulator::calculate_Positions_Mid(float timeStep, bool tmp) {
	for (auto& point : m_massPoints) {
		if (!point.m_isFixed) {
			if (tmp) {
				point.m_ptemp = point.m_position + timeStep * point.m_velocity;
			}
			else {
				point.m_position = point.m_position + timeStep * point.m_vtemp;
			}
		}
	}
}

void MassSpringSystemSimulator::calculate_Velocity_Mid(float timeStep, bool tmp) {
	for (auto& point : m_massPoints) {
		if (!point.m_isFixed) {
			if (tmp) {
				point.m_vtemp = point.m_velocity + timeStep * point.m_force / m_fMass;
			}
			else {
				point.m_velocity = point.m_velocity + timeStep * point.m_force / m_fMass;
			}
		}
	}
}

void MassSpringSystemSimulator::init() {
	/*
	* m_massPoints.clear();
	m_springs.clear();
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	int p1 = addMassPoint(Vec3(0, 0.5, 0), Vec3(0, -1, 0), false);
	int p2 = addMassPoint(Vec3(0.25, 1, 1), Vec3(1, -1, 1), true);
	int p3 = addMassPoint(Vec3(0.75, 0.5, 0), Vec3(0, -1, 2), false);
	int p4 = addMassPoint(Vec3(2, 0.5, 0), Vec3(0, -1, 0), false);
	int p5 = addMassPoint(Vec3(1, 1, 1), Vec3(1, 0, 1), false);
	int p6 = addMassPoint(Vec3(0, 0.5, 0.25), Vec3(-1, -1, 0), false);
	int p7 = addMassPoint(Vec3(1, 0.5, 1), Vec3(1, 1, 1), false);
	int p8 = addMassPoint(Vec3(0.5, 0.5, 0.5), Vec3(0.5, 0.5, 0.5), false);
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
	*/

}


void MassSpringSystemSimulator::onClick(int x, int y)
{
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
}

void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	m_massPoints.emplace_back(position, Velocity, Vec3{ 0,0,0 }, isFixed);
	return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	m_springs.emplace_back(masspoint1, masspoint2, initialLength, initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return m_massPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return m_massPoints[index].m_position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return m_massPoints[index].m_velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	for (auto& p : m_massPoints) {
		p.m_force += force;
	}
}



