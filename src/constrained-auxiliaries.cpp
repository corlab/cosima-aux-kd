#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include "constrained-auxiliaries.hpp"

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

ConstrainedAuxiliaries::ConstrainedAuxiliaries(const std::string &name) :
		TaskContext(name), _models_loaded(false), jacobian_Flow(RTT::NoData), inertia_Port(
				"inertia"), inertia_Flow(RTT::NoData), jacobian_Port(
				"jacobian"), p_Port("pMatrix"), lambda_constraint_Port(
				"lambdaConstrained"), jac_constraint_Port("jacConstrained"), jac_constraint_mpi_Port(
				"jacConstrainedMPI"), inertia_constraint_Port(
				"inertiaConstrained"), c_constraint_Port("cConstrained") {

	this->addOperation("loadURDFAndSRDF",
			&ConstrainedAuxiliaries::loadURDFAndSRDF, this, RTT::ClientThread);

	this->addOperation("getKinematicChainNames",
			&ConstrainedAuxiliaries::getKinematicChainNames, this,
			RTT::ClientThread);

	// input ports
	this->ports()->addPort(inertia_Port).doc("Receiving inertia.");
	this->ports()->addPort(jacobian_Port).doc("Receiving jacobian.");

	// output ports
	this->ports()->addPort(p_Port).doc("Sending joint feedback.");
	this->ports()->addPort(lambda_constraint_Port).doc("Sending P matrix.");
	this->ports()->addPort(jac_constraint_Port).doc(
			"Sending constrained jacobian.");
	this->ports()->addPort(jac_constraint_mpi_Port).doc(
			"Sending constrained jacobian MPI.");
	this->ports()->addPort(inertia_constraint_Port).doc(
			"Sending constrained inertia.");
	this->ports()->addPort(c_constraint_Port).doc("Sending constrained C.");
}

void ConstrainedAuxiliaries::updateHook() {

	jacobian_Flow = jacobian_Port.read(jacobian);
	inertia_Flow = inertia_Port.read(M);
	if ((jacobian_Flow != RTT::NoData) && (inertia_Flow != RTT::NoData)) {
		if ((jacobian_Flow == RTT::NewData) || (inertia_Flow == RTT::NewData)) {
			calculateAuxiliaries(jacobian, M);

			// publish items!
			jac_constraint_Port.write(jac_cstr_.cast<float>());

			jac_constraint_mpi_Port.write(jac_cstr_MPI.cast<float>());

			p_Port.write(P.cast<float>());

			lambda_constraint_Port.write(Lamda_cstr.cast<float>());

			inertia_constraint_Port.write(M_cstr_.cast<float>());

			c_constraint_Port.write(C_cstr_.cast<float>());

		}

	}
}

bool ConstrainedAuxiliaries::startHook() {
	return _models_loaded;
}

bool ConstrainedAuxiliaries::configureHook() {
	jacobian_Flow = RTT::NoData;
	inertia_Flow = RTT::NoData;
	return true;
}

std::vector<std::string> ConstrainedAuxiliaries::getKinematicChainNames() {
	std::vector<std::string> names;
	for (unsigned int i = 0; i < _xbotcore_model.get_chain_names().size();
			++i) {
		names.push_back(_xbotcore_model.get_chain_names()[i]);
	}

	return names;
}

void ConstrainedAuxiliaries::selectKinematicChain(
		const std::string& chainName) {
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

	M.resize(activeKDLChain.getNrOfJoints());

	jacobian.resize(activeKDLChain.getNrOfJoints());

	// TODO use constraint properly...!
	jac_cstr_.resize(6, 7);
	jac_constraint_Port.setDataSample(jac_cstr_.cast<float>());

	jac_cstr_MPI.resize(7, 6);
	jac_constraint_mpi_Port.setDataSample(jac_cstr_MPI.cast<float>());

	P.resize(activeKDLChain.getNrOfJoints(), activeKDLChain.getNrOfJoints());
	p_Port.setDataSample(P.cast<float>());

	Lamda_cstr.resize(activeKDLChain.getNrOfJoints() - 1,
			activeKDLChain.getNrOfJoints() - 1);
	lambda_constraint_Port.setDataSample(Lamda_cstr.cast<float>());

	M_cstr_.resize(activeKDLChain.getNrOfJoints(),
			activeKDLChain.getNrOfJoints());
	inertia_constraint_Port.setDataSample(M_cstr_.cast<float>());

	C_cstr_.resize(activeKDLChain.getNrOfJoints(),
			activeKDLChain.getNrOfJoints());
	c_constraint_Port.setDataSample(C_cstr_.cast<float>());

	identity77.resize(activeKDLChain.getNrOfJoints(),
			activeKDLChain.getNrOfJoints());
	identity77 = Eigen::MatrixXd::Identity(activeKDLChain.getNrOfJoints(),
			activeKDLChain.getNrOfJoints());

	identity66.resize(activeKDLChain.getNrOfJoints() - 1,
			activeKDLChain.getNrOfJoints() - 1);
	identity66 = Eigen::MatrixXd::Identity(activeKDLChain.getNrOfJoints() - 1,
			activeKDLChain.getNrOfJoints() - 1);

	tmpeye77.resize(7, 7);
	tmpeye66.resize(6, 6);

	// TODO ???
//	tmpeye77 = 0.01 * identity77;
//	tmpeye66 = 0.01 * identity66;
	tmpeye77 = identity77;
	tmpeye66 = identity66;
}

bool ConstrainedAuxiliaries::loadURDFAndSRDF(const std::string &URDF_path,
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

void ConstrainedAuxiliaries::calculateAuxiliaries(const KDL::Jacobian& jac_,
		const KDL::JntSpaceInertiaMatrix& M_) {

	// TODO constraint jacobian needs to be moved and need to depend on the constraint matrix!
	jac_cstr_ = jac_.data;
	jac_cstr_.row(0).setZero();
	jac_cstr_.row(1).setZero();
	//jac_cstr_.row(2).setZero();
	jac_cstr_.row(3).setZero();
	jac_cstr_.row(4).setZero();
	jac_cstr_.row(5).setZero();

	jac_cstr_MPI = (jac_cstr_.transpose() * jac_cstr_ + tmpeye77).inverse()
			* jac_cstr_.transpose();

	P = identity77 - (jac_cstr_MPI * jac_cstr_);

	M_cstr_ = P * M_.data + identity77 - P;

	C_cstr_ = -(jac_cstr_MPI * jac_cstr_);

	Lamda_cstr = (jac_.data * M_cstr_.inverse() * P * jac_.data.transpose()
			+ tmpeye66).inverse();
}

ORO_LIST_COMPONENT_TYPE(cosima::ConstrainedAuxiliaries)

