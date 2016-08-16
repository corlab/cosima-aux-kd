#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

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

//#include <kdl/chainidsolver_recursive_newton_euler.hpp>
//#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
//#include <kdl/chainfksolver.hpp>

// RST-RT
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>

namespace cosima {

class ForwardKinematics: public RTT::TaskContext {
public:
	ForwardKinematics(std::string const& name);
	bool configureHook();
	void updateHook();
	bool startHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	virtual ~ForwardKinematics() {
	}

protected:

	/**
	 * OutputPorts publish data.
	 */
	RTT::OutputPort<KDL::Jacobian> jacobian_Port;
	RTT::OutputPort<KDL::Jacobian> jacobianDot_Port;

	RTT::OutputPort<KDL::Frame> position_Port;
	RTT::OutputPort<KDL::FrameVel> velocity_Port;

	/**
	 * InputPorts read data.
	 */
	RTT::InputPort<rstrt::robot::JointState> jointFB_Port;
	RTT::FlowStatus jointFB_Flow;
	rstrt::robot::JointState jointFB;


	bool loadURDFAndSRDF(const std::string& URDF_path,
			const std::string& SRDF_path);

	std::vector<std::string> getKinematicChainNames();

	void selectKinematicChain(const std::string& chainName);

	void calculateKinematics(const rstrt::robot::JointState& jointState);

	XBot::XBotCoreModel _xbotcore_model;

	std::string activeKinematicChain;

	// KDL stuff
	KDL::Tree robot_tree;

	KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;

	boost::shared_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
	boost::shared_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;
	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
	boost::shared_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;

	KDL::Chain activeKDLChain;

	KDL::Jacobian jac_;
	KDL::Jacobian jac_dot_;
	KDL::Frame cartFrame;
	KDL::FrameVel velFrame;

	// Helper tools for KDL
	KDLParser p;

private:
	bool _models_loaded;
	std::string xml_string;
};

}
#endif
