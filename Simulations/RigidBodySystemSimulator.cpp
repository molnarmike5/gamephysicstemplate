#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    
}

std::array<RigidBodySystemSimulator::MassPoint, 8> RigidBodySystemSimulator::RigidBody::calc_points()
{
    std::array<MassPoint, 8> m_points {};

    const float x = size.x / 2, y = size.y / 2, z = size.z / 2;
    
    m_points[0] = Vec3(position.x - x, position.y + y, position.z + z);
    m_points[1] = Vec3(position.x + x, position.y + y, position.z + z);
    m_points[2] = Vec3(position.x - x, position.y - y, position.z + z);
    m_points[3] = Vec3(position.x + x, position.y - y, position.z + z);
    m_points[4] = Vec3(position.x - x, position.y + y, position.z - z);
    m_points[5] = Vec3(position.x + x, position.y + y, position.z - z);
    m_points[6] = Vec3(position.x - x, position.y - y, position.z - z);
    m_points[7] = Vec3(position.x + x, position.y - y, position.z - z);
    
    return m_points;
}

Mat4f RigidBodySystemSimulator::RigidBody::calc_inertia_tensor_0()
{
    Mat4f covariance;
    float x = 0;
    const int num_points = points.size();
    const float new_mass = mass / num_points;
    covariance.value[3][3] = 1;
    
    for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.x * new_mass;
    covariance.value[0][0] = x; x = 0;

    for (int i = 0; i < num_points; ++i) x += points[i].position.y * points[i].position.y * new_mass;
    covariance.value[1][2] = x; x = 0;

    for (int i = 0; i < num_points; ++i) x += points[i].position.z * points[i].position.z * new_mass;
    covariance.value[2][2] = x; x = 0;

    for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.y * new_mass;
    covariance.value[0][1] = covariance.value[1][0] = x; x = 0;

    for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.z * new_mass;
    covariance.value[0][2] = covariance.value[2][0] = x; x = 0;

    for (int i = 0; i < num_points; ++i) x += points[i].position.y * points[i].position.z * new_mass;
    covariance.value[1][2] = covariance.value[2][1] = x;

    
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return rigid_bodies_.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return rigid_bodies_[i].position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return rigid_bodies_[i].linear_velocity;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return rigid_bodies_[i].angular_velocity;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{


    
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    rigid_bodies_.push_back(RigidBody(Vec3(), position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    rigid_bodies_[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    rigid_bodies_[i].linear_velocity = velocity;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
    return "Euler";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
    this->DUC = DUC;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    DUC->drawRigidBody(Mat4(0.5,0,0,0,0,0.5,0,0,0,0,0.5,0,0,0,0,1));
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
    m_iTestCase = testCase;
    switch (m_iTestCase)
    {
    case 0:
        cout << "Euler\n";
        break;
    default:
        cout << "Empty Test!\n";
        break;
    }
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}