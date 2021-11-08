#include "MassSpringSystemSimulator.h"

Point::Point(Vec3 position, Vec3 velocity, bool isFixed) : m_position{ position }, m_velocity{ velocity }, m_isFixed{ isFixed } {
}

MassSpringSystemSimulator::MassSpringSystemSimulator() : m_massPoints{}
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
	m_massPoints.emplace_back(position, Velocity, isFixed);
	return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	//m_springs.emplace_back(masspoint1, masspoint2, initialLength, initialLength);
}

int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return 0;
}

int MassSpringSystemSimulator::getNumberOfSprings()
{
	return 0;
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return Vec3();
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return Vec3();
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
}

