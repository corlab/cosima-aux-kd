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

    this->addOperation("setDOFsize", &InverseDynamics::setDOFsize, this, RTT::ClientThread).doc("set DOF size");

	this->ports()->addPort(inertia_Port).doc("Sending inertia.");

	this->ports()->addPort(h_Port).doc(
			"Sending calculated h vector containing Coriolis plus Gravity.");

	this->ports()->addPort(jointFB_Port).doc("Receiving joint feedback.");

	this->addOperation("useRobotInertia", &InverseDynamics::useRobotInertia_func,
			this, RTT::OwnThread).doc("Use the inerati provided by the robot.").arg(
			"useRobotInertia", "use the inertia from the robot or not.");

    receiveTranslationOnly = true;
    if(receiveTranslationOnly){
        TaskSpaceDimension = 3;
    }
    else{
        TaskSpaceDimension = 6;
    }
}

void InverseDynamics::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void InverseDynamics::useRobotInertia_func(bool useRobotInertia) {
	this->useRobotInertia = useRobotInertia;
}

void InverseDynamics::updateHook() {

    jointFB_Flow = jointFB_Port.read(jointFB);
    if (jointFB_Flow == RTT::NewData) {
        calculateDynamics(jointFB);
    }

    if (useRobotInertia) {
        robotInertia_Flow = robotInertia_Port.read(inertia);
        if (robotInertia_Flow != RTT::NoData) {
            inertia_Port.write(inertia);
        }
    }

    if (jointFB_Flow != RTT::NoData) {
        if (!useRobotInertia) {
            inertia_Port.write(inertia);
        }
        h_Port.write(h);
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

bool InverseDynamics::selectKinematicChain(const std::string& chainName) {
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
        return false;
	}
	_models_loaded = true;

	log(Info) << "[" << this->getName() << "] " << "robot_tree joints: "
			<< robot_tree.getNrOfJoints() << ", robot_tree segments: "
			<< robot_tree.getNrOfSegments() << endlog();

	log(Info) << "[" << this->getName() << "] " << " activeKDLChain joints: "
			<< activeKDLChain.getNrOfJoints() << ", activeKDLChain segments: "
			<< activeKDLChain.getNrOfSegments() << RTT::endlog();

    if(activeKDLChain.getNrOfJoints() != DOFsize){
        log(Info) << "DOFsize " << DOFsize << " is different from urdf model" << RTT::endlog();
        assert(false); //TODO
        return false;
    }

	id_dyn_solver.reset(new KDL::ChainDynParam(activeKDLChain, gravity_vector));

    jntPosConfigPlusJntVelConfig_q.resize(DOFsize);

    gravity = Eigen::VectorXf(DOFsize,1);
    coriolis = Eigen::VectorXf(DOFsize,1);

    inertia = Eigen::MatrixXf(DOFsize,DOFsize);
    inertia_Port.setDataSample(inertia);

    h = Eigen::VectorXf(DOFsize);
    h_Port.setDataSample(h);

    M.resize(DOFsize);
    C_.resize(DOFsize);
    G_.resize(DOFsize);

    jointFB = rstrt::robot::JointState(DOFsize);
    jointFB.angles.fill(0);
    return true;
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

void InverseDynamics::calculateDynamics(
		const rstrt::robot::JointState& jointState) {

//	jntPosConfigPlusJntVelConfig_q.q.data = jointState.angles.cast<double>();
//    jntPosConfigPlusJntVelConfig_q.qdot.data = jointState.velocities.cast<double>();
    this->castEigenVectorFtoD(jointState.angles, jntPosConfigPlusJntVelConfig_q.q.data);
    this->castEigenVectorFtoD(jointState.velocities, jntPosConfigPlusJntVelConfig_q.qdot.data);

	/* ### execute solver for Jacobian based on velocities */
	if (!useRobotInertia) {
		id_dyn_solver->JntToMass(jntPosConfigPlusJntVelConfig_q.q, M);
	}
	id_dyn_solver->JntToGravity(jntPosConfigPlusJntVelConfig_q.q, G_);
	id_dyn_solver->JntToCoriolis(jntPosConfigPlusJntVelConfig_q.q,
			jntPosConfigPlusJntVelConfig_q.qdot, C_);


    this->castEigenMatrixDtoF(M.data, inertia);
    this->castEigenVectorDtoF(G_.data, gravity);
    this->castEigenVectorDtoF(C_.data, coriolis);

    h = coriolis + gravity;
}

void InverseDynamics::castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f) {
    f = d.cast <float> ();
}

void InverseDynamics::castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d) {
    d = f.cast <double> ();
}

void InverseDynamics::castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f) {
    f = d.cast <float> ();
}

void InverseDynamics::castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d) {
    d = f.cast <double> ();
}


ORO_LIST_COMPONENT_TYPE(cosima::InverseDynamics)

