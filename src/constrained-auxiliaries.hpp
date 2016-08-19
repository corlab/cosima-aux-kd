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
    RTT::OutputPort<Eigen::MatrixXf> lambda_constraint_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_constraint_Port, jac_Dot_constraint_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_constraint_mpi_Port;
    RTT::OutputPort<Eigen::MatrixXf> jac_mpi_Port;
    RTT::OutputPort<Eigen::MatrixXf> inertia_constraint_Port;
    RTT::OutputPort<Eigen::MatrixXf> c_constraint_Port;

	// intermediate output
    Eigen::MatrixXf P;
    Eigen::MatrixXf Lamda_cstr;
    Eigen::MatrixXf jac_constraint_, jac_cstr_MPI;
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
    RTT::InputPort<Eigen::MatrixXf> jacobian_Port;
    RTT::InputPort<Eigen::MatrixXf> jacobianDot_Port;
    RTT::InputPort<Eigen::MatrixXf> jacobianOriginal_Port;
    RTT::InputPort<Eigen::MatrixXf> jacobianDotOriginal_Port;

    RTT::InputPort<Eigen::MatrixXf> inertia_Port;
	RTT::FlowStatus jacobian_Flow;
    RTT::FlowStatus jacobianOriginal_Flow;
    RTT::FlowStatus jacobianDot_Flow;
    RTT::FlowStatus jacobianDotOriginal_Flow;
    RTT::FlowStatus inertia_Flow;
    Eigen::MatrixXf jacobian;
    Eigen::MatrixXf jacobianOriginal;
    Eigen::MatrixXf jacobianDot;
    Eigen::MatrixXf jacobianDotOriginal;
    Eigen::MatrixXf M;

    void calculateAuxiliaries(const Eigen::MatrixXf& jac_, const Eigen::MatrixXf& jac_Dot_, const Eigen::MatrixXf& M_, const Eigen::MatrixXf& jacobianOriginal_, const Eigen::MatrixXf& jacobianDotOriginal_);
    void setDOFsize(unsigned int DOFsize);


private:
    unsigned int DOFsize;
    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
};

}
#endif
