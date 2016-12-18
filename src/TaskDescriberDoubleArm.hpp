/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

// rst-rt includes
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>

// KDL includes
#include "kdl_parser.hpp"
#include "model.h"
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class TaskDescriberDoubleArm: public RTT::TaskContext {
public:
    TaskDescriberDoubleArm(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void loadModel(std::string modelname, std::string chain_root_link_name, std::string chain_tip_link_name);
    void preparePorts();
    bool exists_test(const std::string& name);
    void castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f);
    void castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d);
    void castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f);
    void castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d);
    void displayCurrentState();
private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotFull_port;

    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTask_port;

    RTT::OutputPort<Eigen::MatrixXf> out_jacobianCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotCstr_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;

    Eigen::MatrixXf out_jacobianFull_var;
    Eigen::MatrixXf out_jacobianDotFull_var;

    Eigen::MatrixXf out_jacobianTask_var;
    Eigen::MatrixXf out_jacobianDotTask_var;

    Eigen::MatrixXf out_jacobianCstr_var;
    Eigen::MatrixXf out_jacobianDotCstr_var;

    unsigned int DOFsize;
    bool portsArePrepared;

    //KDL types
    KDL::Tree kdl_tree; // KDL::Tree needed for KDL::Chain creation
    KDL::Chain kdl_chain_; // Needed for the different solvers
    urdf::Model robot_model;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;
    KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;
    KDL::Jacobian jac_;
    KDL::Jacobian jac_dot_;
};

