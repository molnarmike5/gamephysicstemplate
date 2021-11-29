#include "RigidBodySystemSimulator.h"

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
	const double new_mass = mass / (num_points * 1.0);

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
	for (auto& rigidBody : rigid_bodies_) {
		rigidBody.position = rigidBody.intial_position;
		rigidBody.linear_velocity = Vec3();
		rigidBody.angular_velocity = Vec3();
		rigidBody.angular_momentum = Vec3();
		rigidBody.torque = 0;
		rigidBody.orientation = rigidBody.initial_orientation;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.00005f; // works with 3000FPS lol
		inputWorld = inputWorld * inputScale;
		for (auto i = 0; i < rigid_bodies_.size(); i++) {
			applyForceOnBody(i, rigid_bodies_[i].position, inputWorld);
		}
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{

	for (auto& rigidBody : rigid_bodies_) {
		//Euler Step
		rigidBody.position += timeStep * rigidBody.linear_velocity;
		rigidBody.linear_velocity += timeStep * (rigidBody.externalForce / rigidBody.mass);
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


		Mat4 transform{};
		Mat4 scale{};
		//Trivial
		transform.initTranslation(rigidBody.position.x, rigidBody.position.y, rigidBody.position.z);
		scale.initScaling(rigidBody.size.x, rigidBody.size.y, rigidBody.size.z);
		rigidBody.obj2world = scale * rigidBody.orientation.getRotMat() * transform;
	}

	//Check collision
	for (int i = 0; i < rigid_bodies_.size(); i++) {
		for (int j = i + 1; j < rigid_bodies_.size(); j++) {
			auto info = checkCollisionSAT(rigid_bodies_[i].obj2world, rigid_bodies_[j].obj2world);
			if (info.isValid) {
				calculateCollision(rigid_bodies_[i], rigid_bodies_[j], info);
			}
		}
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
	rigid_bodies_[i].externalForce += force;
	rigid_bodies_[i].torque += cross(loc, force);
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	rigid_bodies_.push_back(RigidBody(position, size, mass));
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	rigid_bodies_[i].orientation = orientation;
	rigid_bodies_[i].initial_orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	rigid_bodies_[i].linear_velocity = velocity;
}

const char* RigidBodySystemSimulator::getTestCasesStr()
{
	return "Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

	for (auto& rigidBody : rigid_bodies_) {
		DUC->drawRigidBody(rigidBody.obj2world);
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	rigid_bodies_.clear();
	switch (m_iTestCase)
	{
	case 0:
		reset();
		cout << "Demo2!\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_2));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		break;
	case 1:
		reset();
		cout << "Demo3!\n";
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), -M_PI_4));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));
		addRigidBody(Vec3(1.2, 0, 0), Vec3(1, 0.6, 0.5), 4);
		setOrientationOf(1, Quat(Vec3(0, 1, 0), M_PI_4));
		applyForceOnBody(1, Vec3(0.3, 0.5, 0.25), Vec3(-1, 1, 0));
		break;
	case 2:
		reset();
		cout << "Demo4!\n";
		//Leider wegen Collision Detection, ist uns keine bessere Simulation eingefallen, aber es funtioniert ganz gut
		addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
		addRigidBody(Vec3(1.2, 0, 0), Vec3(1, 0.6, 0.5), 4);
		addRigidBody(Vec3(0.6, 2, 0), Vec3(5, 1, 5), 10);
		addRigidBody(Vec3(-3, 0.5, 0), Vec3(1, 1, 1), 1);
		setOrientationOf(0, Quat(Vec3(0, 0, 1), M_PI_4));
		setOrientationOf(1, Quat(Vec3(0, 1, 0), M_PI_4));
		setOrientationOf(2, Quat(Vec3(0, 0, 0), 0));
		setOrientationOf(3, Quat(Vec3(1, 0, -0.5), M_PI / 7));
		applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 3, 0));
		applyForceOnBody(1, Vec3(0.3, 0.5, 0.25), Vec3(-3, 1, 0));
		applyForceOnBody(3, Vec3(-3, 1, 0), Vec3(1, 0, 0));
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

void RigidBodySystemSimulator::calculateCollision(RigidBody& r1, RigidBody& r2, CollisionInfo& info) {
	Vec3 cPoint = info.collisionPointWorld;
	Vec3 cNormal = info.normalWorld;

	Vec3 xa = cPoint - r1.position;
	Vec3 xb = cPoint - r2.position;

	Vec3 va = r1.linear_velocity + cross(r1.angular_velocity, xa);
	Vec3 vb = r2.linear_velocity + cross(r2.angular_velocity, xb);

	Vec3 v_rel = va - vb;

	double c = 1.0;

	auto numerator = -(1 + c) * dot(v_rel, cNormal);
	auto denominator = (1.0 / r1.mass + 1.0 / r2.mass);
	auto fatVector1 = cross(r1.inertia_tensor_0_inverse * (cross(xa, cNormal)), xa);
	auto fatVector2 = cross(r2.inertia_tensor_0_inverse * (cross(xb, cNormal)), xb);
	denominator += dot(fatVector1 + fatVector2, cNormal);
	double impulse = numerator / denominator;

	r1.linear_velocity += impulse * cNormal / r1.mass;
	r2.linear_velocity -= impulse * cNormal / r2.mass;

	r1.angular_momentum += cross(xa, impulse * cNormal);
	r2.angular_momentum -= cross(xb, impulse * cNormal);
}