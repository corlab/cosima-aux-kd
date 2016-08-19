#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include "forward-kinematics.hpp"

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

ForwardKinematics::ForwardKinematics(const std::string &name) :
		TaskContext(name), _models_loaded(false), jointFB_Flow(RTT::NoData), jacobian_Port(
				"jacobian"), jacobianDot_Port("jacobianDot"), position_Port(
				"position"), velocity_Port("velocity"), jointFB_Port("jointFB"),
                                                    out_jointFB_Port("out_jointFB"),
jacobianOriginal_Port("jacobianOriginal"),
jacobianDotOriginal_Port("jacobianDotOriginal"){

	this->addOperation("loadURDFAndSRDF", &ForwardKinematics::loadURDFAndSRDF,
			this, RTT::ClientThread);

	this->addOperation("getKinematicChainNames",
			&ForwardKinematics::getKinematicChainNames, this,
			RTT::ClientThread);

    this->addOperation("setDOFsize", &ForwardKinematics::setDOFsize, this, RTT::ClientThread).doc("set DOF size");

	this->ports()->addPort(jacobian_Port).doc("Sending calculated jacobian.");

            this->ports()->addPort(out_jointFB_Port).doc("Forward robot joint feedback.");

	this->ports()->addPort(jacobianDot_Port).doc(
			"Sending calculated jacobian dot.");

    this->ports()->addPort(jacobianOriginal_Port).doc(
            "Sending calculated jacobian original.");

    this->ports()->addPort(jacobianDotOriginal_Port).doc(
            "Sending calculated jacobian dot original.");

	this->ports()->addPort(position_Port).doc(
			"Sending calculated cartesian position.");

	this->ports()->addPort(velocity_Port).doc(
			"Sending calculated cartesian velocity.");

	this->ports()->addPort(jointFB_Port).doc("Receiving joint feedback.");

    receiveTranslationOnly = true;
    if(receiveTranslationOnly){
        TaskSpaceDimension = 3;
    }
    else{
        TaskSpaceDimension = 6;
    }

    cartPosFloat = Eigen::VectorXf(TaskSpaceDimension);
    cartVelFloat = Eigen::VectorXf(TaskSpaceDimension);
    jacFloat_= Eigen::MatrixXf(6,DOFsize); //allways 6 rows
    jacFloat_dot_ = Eigen::MatrixXf(6,DOFsize); //allways 6 rows
}

void ForwardKinematics::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void ForwardKinematics::updateHook() {

    jointFB_Flow = jointFB_Port.read(jointFB);
    if (jointFB_Flow == RTT::NewData) {
        calculateKinematics(jointFB);
    }

    if (jointFB_Flow != RTT::NoData) {

        jacobian_Port.write(jac_current);
        jacobianDot_Port.write(jac_dot_current);
        jacobianOriginal_Port.write(jac_original_current);
        jacobianDotOriginal_Port.write(jac_dot_original_current);

        if(receiveTranslationOnly){
            cartPosFloat(0) = cartFrame.p.x();
            cartPosFloat(1) = cartFrame.p.y();
            cartPosFloat(2) = cartFrame.p.z();
            cartVelFloat(0) = velFrame.p.v.x();
            cartVelFloat(1) = velFrame.p.v.y();
            cartVelFloat(2) = velFrame.p.v.z();
        }else{
            cartPosFloat(0) = cartFrame.p.x();
            cartPosFloat(1) = cartFrame.p.y();
            cartPosFloat(2) = cartFrame.p.z();
            cartPosFloat(3) = cartFrame.M.GetRot().x();
            cartPosFloat(4) = cartFrame.M.GetRot().y();
            cartPosFloat(5) = cartFrame.M.GetRot().z();
            cartVelFloat(0) = velFrame.p.v.x();
            cartVelFloat(1) = velFrame.p.v.y();
            cartVelFloat(2) = velFrame.p.v.z();
            cartVelFloat(3) = velFrame.GetFrame().M.GetRot().x();
            cartVelFloat(4) = velFrame.GetFrame().M.GetRot().y();
            cartVelFloat(5) = velFrame.GetFrame().M.GetRot().z();
        }

        position_Port.write(cartPosFloat);
//		RTT::log(RTT::Error) << "Cart. Pos: " << cartFrame << RTT::endlog();

        velocity_Port.write(cartVelFloat);

        out_jointFB_Port.write(jointFB);

    }
}

bool ForwardKinematics::startHook() {
	return _models_loaded;
}

bool ForwardKinematics::configureHook() {
	return true;
}

std::vector<std::string> ForwardKinematics::getKinematicChainNames() {
	std::vector<std::string> names;
	for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
			++i) {
		names.push_back(_xbotcore_model.get_chain_names()[i]);
	}
	return names;
}

bool ForwardKinematics::selectKinematicChain(const std::string& chainName) {
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

//	robot_tree.getChain(enabled_joints_in_chain[0],
//			enabled_joints_in_chain[enabled_joints_in_chain.size() - 1],
//			activeKDLChain);

	log(Info) << "[" << this->getName() << "] " << " activeKDLChain joints: "
			<< activeKDLChain.getNrOfJoints() << ", activeKDLChain segments: "
			<< activeKDLChain.getNrOfSegments() << RTT::endlog();

    if(activeKDLChain.getNrOfJoints() != DOFsize){
        log(Info) << "DOFsize " << DOFsize << " is different from urdf model" << RTT::endlog();
        assert(false); //TODO
        return false;
    }

	jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(activeKDLChain));
	jnt_to_jac_dot_solver.reset(
			new KDL::ChainJntToJacDotSolver(activeKDLChain));
	jnt_to_cart_pos_solver.reset(
			new KDL::ChainFkSolverPos_recursive(activeKDLChain));
	jnt_to_cart_vel_solver.reset(
			new KDL::ChainFkSolverVel_recursive(activeKDLChain));

    jntPosConfigPlusJntVelConfig_q.resize(DOFsize);

    jac_current = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    jacobian_Port.setDataSample(jac_current);

    jac_dot_current = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    jacobianDot_Port.setDataSample(jac_dot_current);
    jac_original_current = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    jacobianOriginal_Port.setDataSample(jac_original_current);
    jac_dot_original_current = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    jacobianDotOriginal_Port.setDataSample(jac_dot_original_current);

    jac_.resize(DOFsize);
    jac_dot_.resize(DOFsize);

    position_Port.setDataSample(cartPosFloat);
    velocity_Port.setDataSample(cartVelFloat);

    jointFB = rstrt::robot::JointState(DOFsize);
    jointFB.angles.fill(0);

    out_jointFB_Port.setDataSample(jointFB);
}

bool ForwardKinematics::loadURDFAndSRDF(const std::string &URDF_path,
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

void ForwardKinematics::calculateKinematics(
		const rstrt::robot::JointState& jointState) {

//	jntPosConfigPlusJntVelConfig_q.q.data = jointState.angles.cast<double>();
//    jntPosConfigPlusJntVelConfig_q.qdot.data = jointState.velocities.cast<double>();
    this->castEigenVectorFtoD(jointState.angles, jntPosConfigPlusJntVelConfig_q.q.data);
    this->castEigenVectorFtoD(jointState.velocities, jntPosConfigPlusJntVelConfig_q.qdot.data);

	/* ### execute solver for Jacobian based on velocities */
	jnt_to_jac_solver->JntToJac(jntPosConfigPlusJntVelConfig_q.q, jac_,
			activeKDLChain.getNrOfSegments());

    RTT::log(RTT::Warning) << "Jac: " << jac_.data << RTT::endlog();

	jnt_to_jac_dot_solver->JntToJacDot(jntPosConfigPlusJntVelConfig_q, jac_dot_,
			activeKDLChain.getNrOfSegments());

	// jnt to cart pos
	jnt_to_cart_pos_solver->JntToCart(jntPosConfigPlusJntVelConfig_q.q,
			cartFrame, activeKDLChain.getNrOfSegments());
	jnt_to_cart_vel_solver->JntToCart(jntPosConfigPlusJntVelConfig_q, velFrame,
			activeKDLChain.getNrOfSegments());


    this->castEigenMatrixDtoF(jac_.data, jacFloat_);

    this->castEigenMatrixDtoF(jac_dot_.data, jacFloat_dot_);

    RTT::log(RTT::Warning) << "jacFloat_: " << jacFloat_ << RTT::endlog();


    //convert jacobian to eigen and add constraint
    if(receiveTranslationOnly){
        jac_current = jacFloat_.topRows<3>();
        jac_original_current = jacFloat_.topRows<3>();
        jac_dot_current = jacFloat_dot_.topRows<3>();
        jac_dot_original_current = jacFloat_dot_.topRows<3>();

    //    jac_current.row(0).setZero();
    //    jac_current.row(1).setZero();
        jac_current.row(2).setZero();

    //    jac_dot_current.row(0).setZero();
    //    jac_dot_current.row(1).setZero();
        jac_dot_current.row(2).setZero();

    }
    else{
        jac_current = jacFloat_;
        jac_dot_current = jacFloat_dot_;
        jac_original_current = jacFloat_;
        jac_dot_original_current = jacFloat_dot_;

    //    jac_current.row(0).setZero();
    //    jac_current.row(1).setZero();
        jac_current.row(2).setZero();
    //    jac_current.row(3).setZero();
    //    jac_current.row(4).setZero();
        jac_current.row(5).setZero();

    //    jac_dot_current.row(0).setZero();
    //    jac_dot_current.row(1).setZero();
        jac_dot_current.row(2).setZero();
    //    jac_dot_current.row(3).setZero();
    //    jac_dot_current.row(4).setZero();
        jac_dot_current.row(5).setZero();
    }


    RTT::log(RTT::Warning) << "jac_current: " << jac_current << RTT::endlog();

}

void ForwardKinematics::castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f) {
    f = d.cast <float> ();
}

void ForwardKinematics::castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d) {
    d = f.cast <double> ();
}

void ForwardKinematics::castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f) {
    f = d.cast <float> ();
}

void ForwardKinematics::castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d) {
    d = f.cast <double> ();
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cosima::ForwardKinematics)
ORO_LIST_COMPONENT_TYPE(cosima::ForwardKinematics)

