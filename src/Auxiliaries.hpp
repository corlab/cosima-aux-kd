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

class Auxiliaries: public RTT::TaskContext {
public:
    Auxiliaries(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void preparePorts();
    void displayCurrentState();

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::InputPort<Eigen::MatrixXf> in_inertia_port;
    RTT::InputPort<Eigen::VectorXf> in_gravity_port;
    RTT::InputPort<Eigen::VectorXf> in_coriolis_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::MatrixXf> out_lambda_port;
    RTT::OutputPort<Eigen::VectorXf> out_pAndMu_port;
    RTT::OutputPort<Eigen::MatrixXf> out_lambdaTranslation_port;
    RTT::OutputPort<Eigen::VectorXf> out_pAndMuTranslation_port;
    RTT::OutputPort<Eigen::MatrixXf> out_lambdaOrientation_port;
    RTT::OutputPort<Eigen::VectorXf> out_pAndMuOrientation_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_inertia_flow;
    RTT::FlowStatus in_gravity_flow;
    RTT::FlowStatus in_coriolis_flow;
    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    Eigen::MatrixXf in_inertia_var;
    Eigen::VectorXf in_gravity_var;
    Eigen::VectorXf in_coriolis_var;
    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;
    Eigen::MatrixXf out_lambda_var;
    Eigen::VectorXf out_pAndMu_var;
    Eigen::MatrixXf out_lambdaTranslation_var;
    Eigen::VectorXf out_pAndMuTranslation_var;
    Eigen::MatrixXf out_lambdaOrientation_var;
    Eigen::VectorXf out_pAndMuOrientation_var;
    Eigen::MatrixXf identity66, tmpeye66;
    unsigned int DOFsize;
    bool portsArePrepared;
};

