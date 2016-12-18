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
//#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_nr.hpp>

// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class WrapperKDL: public RTT::TaskContext {
public:
    WrapperKDL(std::string const & name);

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
    void computeKDLinformation(
            rstrt::robot::JointState & robotstatus,
            Eigen::MatrixXf & inertia,
            Eigen::VectorXf & gravity,
            Eigen::VectorXf & coriolis,
            Eigen::VectorXf & cartPos,
            Eigen::VectorXf & cartVel,
            Eigen::MatrixXf & jacobian,
            Eigen::MatrixXf & jacobianDot);
    void displayCurrentState();
    bool exists_test(const std::string& name);
    void castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f);
    void castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d);
    void castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f);
    void castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d);

private:
    // Declare input ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::robot::JointState> out_robotstatus_port;
    RTT::OutputPort<Eigen::MatrixXf> out_inertia_port;
    RTT::OutputPort<Eigen::VectorXf> out_gravity_port;
    RTT::OutputPort<Eigen::VectorXf> out_coriolis_port;
    RTT::OutputPort<Eigen::VectorXf> out_coriolisAndGravity_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartPos_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartVel_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartAcc_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartPosTranslation_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartVelTranslation_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartAccTranslation_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartPosOrientation_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartVelOrientation_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartAccOrientation_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobian_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDot_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTranslation_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTranslation_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianOrientation_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotOrientation_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;

    // variables
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::robot::JointState out_robotstatus_var;
    Eigen::MatrixXf out_inertia_var;
    Eigen::VectorXf out_gravity_var;
    Eigen::VectorXf out_coriolis_var;
    Eigen::VectorXf out_coriolisAndGravity_var;
    Eigen::VectorXf out_cartPos_var;
    Eigen::VectorXf out_cartVel_var;
    Eigen::VectorXf out_cartAcc_var;
    Eigen::VectorXf out_cartPosTranslation_var;
    Eigen::VectorXf out_cartVelTranslation_var;
    Eigen::VectorXf out_cartAccTranslation_var;
    Eigen::VectorXf out_cartPosOrientation_var;
    Eigen::VectorXf out_cartVelOrientation_var;
    Eigen::VectorXf out_cartAccOrientation_var;
    Eigen::MatrixXf out_jacobian_var;
    Eigen::MatrixXf out_jacobianDot_var;
    Eigen::MatrixXf out_jacobianTranslation_var;
    Eigen::MatrixXf out_jacobianDotTranslation_var;
    Eigen::MatrixXf out_jacobianOrientation_var;
    Eigen::MatrixXf out_jacobianDotOrientation_var;
    Eigen::MatrixXf identity66, identity33;
    unsigned int DOFsize;
    bool portsArePrepared;

    //KDL types
    urdf::Model robot_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain_;
    KDL::Vector gravity_vector;
    boost::scoped_ptr<KDL::ChainDynParam> id_dyn_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver;
    boost::scoped_ptr<KDL::ChainJntToJacDotSolver> jnt_to_jac_dot_solver;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_cart_pos_solver;
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> jnt_to_cart_vel_solver;
    KDL::JntArrayVel jntPosConfigPlusJntVelConfig_q;
    KDL::JntArray G_;
    KDL::JntSpaceInertiaMatrix M_;
    KDL::JntArray C_;
    KDL::Jacobian jac_;
    KDL::Jacobian jac_dot_;
    KDL::Frame cartPosFrame;
    KDL::FrameVel cartVelFrame;
};

