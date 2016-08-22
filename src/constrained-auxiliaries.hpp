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
    RTT::OutputPort<Eigen::MatrixXf> lambda_cstr_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_cstr_Port, jac_Dot_cstr_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_cstr_mpi_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_mpi_Port;
    RTT::OutputPort<Eigen::MatrixXf> inertia_cstr_Port;
    RTT::OutputPort<Eigen::MatrixXf> c_cstr_Port;

	// intermediate output
    Eigen::MatrixXf P;
    Eigen::MatrixXf Lamda_cstr;
    Eigen::MatrixXf jac_cstr_, jac_cstr_MPI;
    Eigen::MatrixXf jac_Dot_cstr_;
    Eigen::MatrixXf M_cstr_;
    Eigen::MatrixXf C_cstr_;
    Eigen::MatrixXf jac_MPI;

	// auxiliaries
    Eigen::MatrixXf identityDOFsizeDOFsize, identityTSdimTSdim;
    Eigen::MatrixXf tmpeyeDOFsizeDOFsize, tmpeyeTSdimTSdim;


	/**
	 * InputPorts read data.
	 */
    RTT::InputPort<Eigen::MatrixXf> jac_Port;
    RTT::InputPort<Eigen::MatrixXf> jacDot_Port;
    RTT::InputPort<Eigen::MatrixXf> jac_full_Port;
    RTT::InputPort<Eigen::MatrixXf> jacDot_full_Port;

    RTT::InputPort<Eigen::MatrixXf> inertia_Port;
	RTT::FlowStatus jac_Flow;
    RTT::FlowStatus jac_full_Flow;
    RTT::FlowStatus jacDot_Flow;
    RTT::FlowStatus jacDot_full_Flow;
    RTT::FlowStatus inertia_Flow;
    Eigen::MatrixXf jac;
    Eigen::MatrixXf jac_full;
    Eigen::MatrixXf jacDot;
    Eigen::MatrixXf jacDot_full;
    Eigen::MatrixXf M;

    void calculateAuxiliaries(const Eigen::MatrixXf& jac_task, const Eigen::MatrixXf& jac_Dot_task, const Eigen::MatrixXf& M_, const Eigen::MatrixXf& jac_full_, const Eigen::MatrixXf& jacDot_full_);
    void setDOFsize(unsigned int DOFsize);


private:
    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
};

}
#endif
