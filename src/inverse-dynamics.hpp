#ifndef INVERSE_DYNAMICS_HPP
#define INVERSE_DYNAMICS_HPP

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

class InverseDynamics: public RTT::TaskContext {
public:
	InverseDynamics(std::string const& name);
	bool configureHook();
	void updateHook();
	bool startHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	void setBaseAndTip(std::string base, std::string tip);
	void setGravityVector(KDL::Vector gravity);
	virtual ~InverseDynamics() {
	}

protected:

	void useRobotInertia_func(bool useRobotInertia);

	/**
	 * OutputPorts publish data.
	 */
    RTT::OutputPort<Eigen::MatrixXf> inertia_Port;
	RTT::OutputPort<Eigen::VectorXf> h_Port;

	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rstrt::robot::JointState> jointFB_Port;
	RTT::FlowStatus jointFB_Flow;
	rstrt::robot::JointState jointFB;

	// inertia from robot
    RTT::InputPort<Eigen::MatrixXf> robotInertia_Port;
	RTT::FlowStatus robotInertia_Flow;
	bool useRobotInertia;

	bool loadURDFAndSRDF(const std::string& URDF_path,
			const std::string& SRDF_path);

	std::vector<std::string> getKinematicChainNames();

    bool selectKinematicChain(const std::string& chainName);

    void calculateDynamics(const rstrt::robot::JointState& jointState);
    void computeGravity(rstrt::robot::JointState const & jointState, Eigen::VectorXf & gravity);

    // needs to be executed as very first operation
	void setDOFsize(unsigned int DOFsize);

	// needs to be executed as second operation
	void setTranslationOnly(const bool translationOnly);

    void castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f);
    void castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d);
    void castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f);
    void castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d);
    bool exists_test(const std::string& name);

	XBot::XBotCoreModel _xbotcore_model;

	std::string activeKinematicChain;

    Eigen::MatrixXf inertia;
    Eigen::VectorXf gravity;
    Eigen::VectorXf coriolis;

	// KDL stuff
	KDL::Tree robot_tree;

	KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;

	KDL::JntSpaceInertiaMatrix M;
	KDL::JntArray C_;
	KDL::JntArray G_;
    Eigen::VectorXf h;


	KDL::Vector gravity_vector;

	KDL::Chain activeKDLChain;

	boost::shared_ptr<KDL::ChainDynParam> id_dyn_solver;

	// Helper tools for KDL
	KDLParser p;

private:
	bool _models_loaded;
	std::string xml_string,base_string,tip_string;
    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
};

}
#endif
