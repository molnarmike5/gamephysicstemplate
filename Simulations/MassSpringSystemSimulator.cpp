#include "MassSpringSystemSimulator.h"

Point::Point(Vec3 position, Vec3 velocity, Vec3 force, bool isFixed) : m_position{ position }, m_velocity{ velocity }, m_force{ force }, m_isFixed{ isFixed } {
}

Spring::Spring(int point1, int point2, float initialLength, float currentLength) : m_point1(point1),m_point2(point2),m_initialLength(initialLength),m_currentLength(currentLength){
	
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

}

void MassSpringSystemSimulator::reset()
{
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
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

void MassSpringSystemSimulator::applyExternalForce(Vec3 force){
	for (auto& p : m_massPoints) {
		p.m_force += force;
	}
}


