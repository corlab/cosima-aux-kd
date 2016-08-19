#ifndef CONSTRAINED_AUXILIARIES_HPP
#define CONSTRAINED_AUXILIARIES_HPP

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>

#include <boost/shared_ptr.hpp>

#include "KDLParser.hpp"

#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <XBotCoreModel.h>

// KDL includes
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/framevel_io.hpp>
#include <kdl/jacobian.hpp>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainfksolver.hpp>

// RST-RT
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

namespace cosima {

class ConstrainedAuxiliaries: public RTT::TaskContext {
public:
	ConstrainedAuxiliaries(std::string const& name);
	bool configureHook();
	void updateHook();
	bool startHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	virtual ~ConstrainedAuxiliaries() {
	}

protected:

	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<Eigen::MatrixXf> p_Port;
	RTT::OutputPort<Eigen::VectorXf> lambda_constraint_Port;
    RTT::OutputPort<Eigen::VectorXf> jac_constraint_Port, jac_Dot_constraint_Port;
	RTT::OutputPort<Eigen::VectorXf> jac_constraint_mpi_Port;
    RTT::OutputPort<Eigen::VectorXf> jac_mpi_Port;
	RTT::OutputPort<Eigen::VectorXf> inertia_constraint_Port;
	RTT::OutputPort<Eigen::VectorXf> c_constraint_Port;

	// intermediate output
    Eigen::MatrixXf P;
    Eigen::MatrixXf Lamda_cstr;
    Eigen::MatrixXf jac_cstr_, jac_cstr_MPI;
    Eigen::MatrixXf jac_Dot_cstr_;
    Eigen::MatrixXf M_cstr_;
    Eigen::MatrixXf C_cstr_;
    Eigen::MatrixXf jac_MPI;

	// auxiliaries
    Eigen::MatrixXf identity77, identity66;
    Eigen::MatrixXf tmpeye77, tmpeye66;


	/**
	 * InputPorts read data.
	 */
    RTT::InputPort<Eigen::MatrixXf> jacobian_Port;
    RTT::InputPort<Eigen::MatrixXf> jacobianDot_Port;
	RTT::FlowStatus jacobian_Flow;
    RTT::FlowStatus jacobianDot_Flow;
    Eigen::MatrixXf jacobian;
    Eigen::MatrixXf jacobianDot;

	// inertia
    RTT::InputPort<Eigen::MatrixXf> inertia_Port;
	RTT::FlowStatus inertia_Flow;
    Eigen::MatrixXf M;


	bool loadURDFAndSRDF(const std::string& URDF_path,
			const std::string& SRDF_path);

	std::vector<std::string> getKinematicChainNames();

	void selectKinematicChain(const std::string& chainName);

    void calculateAuxiliaries(const Eigen::MatrixXf& jac_, const Eigen::MatrixXf& jac_Dot_, const Eigen::MatrixXf& M_);

	XBot::XBotCoreModel _xbotcore_model;

	std::string activeKinematicChain;

	// KDL stuff
	KDL::Tree robot_tree;

	KDL::Chain activeKDLChain;

	boost::shared_ptr<KDL::ChainDynParam> id_dyn_solver;

	// Helper tools for KDL
	KDLParser p;

private:
	bool _models_loaded;
	std::string xml_string;
    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
};

}
#endif
