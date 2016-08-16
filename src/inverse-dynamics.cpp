#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include "inverse-dynamics.hpp"

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

InverseDynamics::InverseDynamics(const std::string &name) :
		TaskContext(name), _models_loaded(false), jointFB_Flow(RTT::NoData), inertia_Port(
				"inertia"), h_Port("hVector"), jointFB_Port("jointFB"), gravity_vector(
				0., 0., -9.81), robotInertia_Port("robotInertia"), robotInertia_Flow(
				RTT::NoData), useRobotInertia(false) {

	this->addOperation("loadURDFAndSRDF", &InverseDynamics::loadURDFAndSRDF,
			this, RTT::ClientThread);

	this->addOperation("getKinematicChainNames",
			&InverseDynamics::getKinematicChainNames, this, RTT::ClientThread);

	this->ports()->addPort(inertia_Port).doc("Sending inertia.");

	this->ports()->addPort(h_Port).doc(
			"Sending calculated h vector containing Coriolis plus Gravity.");

	this->ports()->addPort(jointFB_Port).doc("Receiving joint feedback.");

	this->addOperation("useRobotInertia", &InverseDynamics::useRobotInertia_func,
			this, RTT::OwnThread).doc("Use the inerati provided by the robot.").arg(
			"useRobotInertia", "use the inertia from the robot or not.");

}

void InverseDynamics::useRobotInertia_func(bool useRobotInertia) {
	this->useRobotInertia = useRobotInertia;
}

void InverseDynamics::updateHook() {

	jointFB_Flow = jointFB_Port.read(jointFB);
	if (jointFB_Flow == RTT::NewData) {
		calculateKinematics(jointFB);
	}

	if (useRobotInertia) {
		robotInertia_Flow = robotInertia_Port.read(M);
		if (robotInertia_Flow != RTT::NoData) {
			inertia_Port.write(M);
		}
	}

	if (jointFB_Flow != RTT::NoData) {
		if (!useRobotInertia) {
			inertia_Port.write(M);
		}
		h_Port.write(h.cast<float>());
	}
}

bool InverseDynamics::startHook() {
	return _models_loaded;
}

bool InverseDynamics::configureHook() {
	return true;
}

std::vector<std::string> InverseDynamics::getKinematicChainNames() {
	std::vector<std::string> names;
	for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
			++i) {
		names.push_back(_xbotcore_model.get_chain_names()[i]);
	}
	return names;
}

void InverseDynamics::selectKinematicChain(const std::string& chainName) {
// Needs to be done in non-real-time only!

	std::vector<std::string> enabled_joints_in_chain;
	_xbotcore_model.get_enabled_joints_in_chain(chainName,
			enabled_joints_in_chain);

	RTT::log(RTT::Warning) << "Size of enabled joints: "
			<< enabled_joints_in_chain.size() << RTT::endlog();

// TODO do not hardcode this!
	if (!p.initTreeAndChainFromURDFString(xml_string, "lwr_arm_base_link",
			"lwr_arm_7_link", robot_tree, activeKDLChain)) {
		log(Error) << "[ DLW " << this->getName()
				<< "] URDF could not be parsed !" << endlog();

		// TODO add proper error handling!
		return;
	}
	_models_loaded = true;

	log(Info) << "[" << this->getName() << "] " << "robot_tree joints: "
			<< robot_tree.getNrOfJoints() << ", robot_tree segments: "
			<< robot_tree.getNrOfSegments() << endlog();

	log(Info) << "[" << this->getName() << "] " << " activeKDLChain joints: "
			<< activeKDLChain.getNrOfJoints() << ", activeKDLChain segments: "
			<< activeKDLChain.getNrOfSegments() << RTT::endlog();

	id_dyn_solver.reset(new KDL::ChainDynParam(activeKDLChain, gravity_vector));

	jntPosConfigPlusJntVelConfig_q.resize(activeKDLChain.getNrOfJoints());

	M.resize(activeKDLChain.getNrOfJoints());
	inertia_Port.setDataSample(M);

	h.resize(activeKDLChain.getNrOfJoints());
	h_Port.setDataSample(h.cast<float>());

	C_.resize(activeKDLChain.getNrOfJoints());
	G_.resize(activeKDLChain.getNrOfJoints());

	jointFB = rstrt::robot::JointState(activeKDLChain.getNrOfJoints());
	jointFB.angles.fill(0);
}

bool InverseDynamics::loadURDFAndSRDF(const std::string &URDF_path,
		const std::string &SRDF_path) {
	if (!_models_loaded) {
		std::string _urdf_path = URDF_path;
		std::string _srdf_path = SRDF_path;

		RTT::log(RTT::Info) << "URDF path: " << _urdf_path << RTT::endlog();
		RTT::log(RTT::Info) << "SRDF path: " << _srdf_path << RTT::endlog();

		_models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);

		for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
				++i) {
//			RTT::log(RTT::Info) << "here: chain #" << i << " "
//					<< _xbotcore_model.get_chain_names()[i] << RTT::endlog();
			std::vector<std::string> enabled_joints_in_chain_i;
			_xbotcore_model.get_enabled_joints_in_chain(
					_xbotcore_model.get_chain_names()[i],
					enabled_joints_in_chain_i);
			for (unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
				RTT::log(RTT::Info) << "  " << enabled_joints_in_chain_i[j]
						<< RTT::endlog();
		}

		xml_string = "";
		std::fstream xml_file(URDF_path.c_str(), std::fstream::in);
		if (xml_file.is_open()) {
			while (xml_file.good()) {
				std::string line;
				std::getline(xml_file, line);
				xml_string += (line + "\n");
			}
			xml_file.close();
		}

//		log(Warning) << xml_string << endlog();
//		kdl_parser::treeFromUrdfModel(*(_xbotcore_model.get_urdf_model()), robot_tree);

		activeKinematicChain = _xbotcore_model.get_chain_names()[0];
		RTT::log(RTT::Warning) << "Set default chain: " << activeKinematicChain
				<< RTT::endlog();

		// setup kdl stuff
//		robot_tree = _xbotcore_model.get_robot_tree();

		selectKinematicChain(activeKinematicChain);
	} else
		RTT::log(RTT::Info) << "URDF and SRDF have been already loaded!"
				<< RTT::endlog();

	return _models_loaded;
}

void InverseDynamics::calculateKinematics(
		const rstrt::robot::JointState& jointState) {

	jntPosConfigPlusJntVelConfig_q.q.data = jointState.angles.cast<double>();
	jntPosConfigPlusJntVelConfig_q.qdot.data =
			jointState.velocities.cast<double>();

	/* ### execute solver for Jacobian based on velocities */
	if (!useRobotInertia) {
		id_dyn_solver->JntToMass(jntPosConfigPlusJntVelConfig_q.q, M);
	}
	id_dyn_solver->JntToGravity(jntPosConfigPlusJntVelConfig_q.q, G_);
	id_dyn_solver->JntToCoriolis(jntPosConfigPlusJntVelConfig_q.q,
			jntPosConfigPlusJntVelConfig_q.qdot, C_);

	h = C_.data + G_.data;
}

ORO_LIST_COMPONENT_TYPE(cosima::InverseDynamics)

