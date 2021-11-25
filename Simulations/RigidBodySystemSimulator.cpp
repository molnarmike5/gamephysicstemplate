#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{
    
}

std::array<RigidBodySystemSimulator::MassPoint, 8> RigidBodySystemSimulator::RigidBody::calc_points(Vec3 box_center, Vec3 size)
{
    std::array<MassPoint, 8> m_points {};

    const float x = size.x / 2, y = size.y / 2, z = size.z / 2;
    
    m_points[0] = Vec3(box_center.x - x, box_center.y + y, box_center.z + z);
    m_points[1] = Vec3(box_center.x + x, box_center.y + y, box_center.z + z);
    m_points[2] = Vec3(box_center.x - x, box_center.y - y, box_center.z + z);
    m_points[3] = Vec3(box_center.x + x, box_center.y - y, box_center.z + z);
    m_points[4] = Vec3(box_center.x - x, box_center.y + y, box_center.z - z);
    m_points[5] = Vec3(box_center.x + x, box_center.y + y, box_center.z - z);
    m_points[6] = Vec3(box_center.x - x, box_center.y - y, box_center.z - z);
    m_points[7] = Vec3(box_center.x + x, box_center.y - y, box_center.z - z);
    
    return m_points;
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
    return 0;
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return Vec3();
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return Vec3();
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
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