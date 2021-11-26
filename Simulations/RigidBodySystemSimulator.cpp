#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator()
{

}

std::array<RigidBodySystemSimulator::MassPoint, 8> RigidBodySystemSimulator::RigidBody::calc_points()
{
	std::array<MassPoint, 8> m_points{};

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

Mat4d RigidBodySystemSimulator::RigidBody::calc_inertia_tensor_0()
{
	Mat4d covariance;
	double x = 0;
	const int num_points = points.size();
	const double new_mass = mass / (num_points*1.0);

	for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.x * new_mass;
	covariance.value[0][0] = x; x = 0;

	for (int i = 0; i < num_points; ++i) x += points[i].position.y * points[i].position.y * new_mass;
	covariance.value[1][1] = x; x = 0;

	for (int i = 0; i < num_points; ++i) x += points[i].position.z * points[i].position.z * new_mass;
	covariance.value[2][2] = x; x = 0;

	for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.y * new_mass;
	covariance.value[0][1] = x;
	covariance.value[1][0] = x;
	x = 0;

	for (int i = 0; i < num_points; ++i) x += points[i].position.x * points[i].position.z * new_mass;
	covariance.value[0][2] = covariance.value[2][0] = x; x = 0;

	for (int i = 0; i < num_points; ++i) x += points[i].position.y * points[i].position.z * new_mass;
	covariance.value[1][2] = covariance.value[2][1] = x;

	auto trace = covariance.value[0][0] + covariance.value[1][1] + covariance.value[2][2];
	Mat4d identity;
	identity.initId();
	identity.value[3][3] = 0;
	auto inertia = identity * trace - covariance;
	inertia.value[3][3] = 1;
	return inertia;
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
	for (auto& rigidBody : rigid_bodies_) {

		//Euler Step
		rigidBody.position += timeStep * rigidBody.linear_velocity;
		rigidBody.linear_velocity += timeStep * (m_externalForce / rigidBody.mass);
		//Calculate (0 w) Quaternion
		Quat q_w(rigidBody.angular_velocity.x, rigidBody.angular_velocity.y, rigidBody.angular_velocity.z, 0);
		//Update r
		rigidBody.orientation += (timeStep / 2) * q_w * rigidBody.orientation;
		rigidBody.orientation = rigidBody.orientation.unit(); //Normalization
		rigidBody.angular_momentum += timeStep * rigidBody.torque;
		//Calculate Inertia Tensor
		auto rotMat = rigidBody.orientation.getRotMat();
		auto rotMatTrans = rigidBody.orientation.getRotMat();
		rotMatTrans.transpose();
		auto inverseTensor = rotMat * rigidBody.inertia_tensor_0_inverse * rotMatTrans;
		rigidBody.angular_velocity = inverseTensor.transformVector(rigidBody.angular_momentum);

		
		
	}	
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
	m_externalForce += force;
	rigid_bodies_[i].torque += cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigid_bodies_.push_back(RigidBody(position, size, mass));
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
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for (auto& rigidBody : rigid_bodies_) {
		Mat4 transform{};
		Mat4 scale{};
		//Trivial
		transform.initTranslation(rigidBody.position.x, rigidBody.position.y, rigidBody.position.z);
		auto scaledobjtoWorld = rigidBody.orientation.getRotMat() * transform;
		scale.initScaling(rigidBody.size.x, rigidBody.size.y, rigidBody.size.z);
		Mat4 model =  scale * scaledobjtoWorld;
		DUC->drawRigidBody(model);
	}
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