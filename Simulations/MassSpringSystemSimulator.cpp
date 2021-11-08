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

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	for (auto& p : m_massPoints) {
		p.m_force = 0;
	}
	calculateElasticForces();
	calculate_Positions_Euler(timeStep);
	calculate_Velocity_Euler(timeStep);
	for (auto& spring : m_springs) {
		auto vector = m_massPoints[spring.m_point1].m_position - m_massPoints[spring.m_point2].m_position;
		spring.m_currentLength = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	}
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

void MassSpringSystemSimulator::calculateElasticForces() {
	for (auto& spring : m_springs) {
		Point* p1 = &m_massPoints[spring.m_point1];
		Point* p2 = &m_massPoints[spring.m_point2];
		auto force = m_fStiffness * (spring.m_currentLength - spring.m_initialLength) * (p1->m_position - p2->m_position) / spring.m_currentLength;
		p1->m_force -= force;
		p2->m_force += force;
	}
}

void MassSpringSystemSimulator::calculate_Positions_Euler(float timeStep) {
	for (auto& point : m_massPoints) {
		point.m_position = point.m_position + timeStep * point.m_velocity;
	}
}

void MassSpringSystemSimulator::calculate_Velocity_Euler(float timeStep){
	for (auto& point : m_massPoints) {
		point.m_velocity = point.m_velocity + timeStep * point.m_force/m_fMass;
	}
}


