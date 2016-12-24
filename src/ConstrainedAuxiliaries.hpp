/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class ConstrainedAuxiliaries: public RTT::TaskContext {
public:
    ConstrainedAuxiliaries(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void setTaskSpaceDimension(unsigned int TaskSpaceDimension);
    void setCstrSpaceDimension(unsigned int CstrSpaceDimension);
    void setConstrainedVersionMode(bool useConstrainedVersion);
    void preparePorts();
    void displayCurrentState();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<Eigen::MatrixXf> in_inertia_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianTask_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDotTask_port; //jacobianDotTask not necessary?
    RTT::InputPort<Eigen::MatrixXf> in_jacobianCstr_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDotCstr_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::MatrixXf> out_lambdaCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_MCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_CCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianCstrMPI_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianMPI_port;
    RTT::OutputPort<Eigen::MatrixXf> out_P_port;

    // Data flow:
    RTT::FlowStatus in_inertia_flow;
    RTT::FlowStatus in_jacobianTask_flow;
    RTT::FlowStatus in_jacobianDotTask_flow;
    RTT::FlowStatus in_jacobianCstr_flow;
    RTT::FlowStatus in_jacobianDotCstr_flow;

    // variables
    Eigen::MatrixXf in_inertia_var;
    Eigen::MatrixXf in_jacobianTask_var;
    Eigen::MatrixXf in_jacobianDotTask_var;
    Eigen::MatrixXf in_jacobianCstr_var;
    Eigen::MatrixXf in_jacobianDotCstr_var;
    Eigen::MatrixXf out_lambdaCstr_var;
    Eigen::MatrixXf out_MCstr_var;
    Eigen::MatrixXf out_CCstr_var;
    Eigen::MatrixXf out_jacobianCstrMPI_var;
    Eigen::MatrixXf out_jacobianMPI_var;
    Eigen::MatrixXf out_P_var;
    Eigen::MatrixXf identityDOFsizeDOFsize, identityTSdimTSdim, identityCSdimCSdim;
    Eigen::MatrixXf tmpeyeDOFsizeDOFsize, tmpeyeTSdimTSdim, tmpeyeCSdimCSdim;
    unsigned int DOFsize, TaskSpaceDimension, CstrSpaceDimension;
    bool useConstrainedVersion;
    bool portsArePrepared;
};

