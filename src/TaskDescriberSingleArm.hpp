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


// BOOST includes
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

class TaskDescriberSingleArm: public RTT::TaskContext {
public:
    TaskDescriberSingleArm(std::string const & name);

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
    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTranslationFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTranslationFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianOrientationFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotOrientationFull_port;

    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTranslationTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTranslationTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianOrientationTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotOrientationTask_port;

    RTT::OutputPort<Eigen::MatrixXf> out_jacobianCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTranslationCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTranslationCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianOrientationCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotOrientationCstr_port;

    // Data flow:
    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;

    // variables
    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;

    Eigen::MatrixXf out_jacobianFull_var;
    Eigen::MatrixXf out_jacobianDotFull_var;
    Eigen::MatrixXf out_jacobianTranslationFull_var;
    Eigen::MatrixXf out_jacobianDotTranslationFull_var;
    Eigen::MatrixXf out_jacobianOrientationFull_var;
    Eigen::MatrixXf out_jacobianDotOrientationFull_var;

    Eigen::MatrixXf out_jacobianTask_var;
    Eigen::MatrixXf out_jacobianDotTask_var;
    Eigen::MatrixXf out_jacobianTranslationTask_var;
    Eigen::MatrixXf out_jacobianDotTranslationTask_var;
    Eigen::MatrixXf out_jacobianOrientationTask_var;
    Eigen::MatrixXf out_jacobianDotOrientationTask_var;

    Eigen::MatrixXf out_jacobianCstr_var;
    Eigen::MatrixXf out_jacobianDotCstr_var;
    Eigen::MatrixXf out_jacobianTranslationCstr_var;
    Eigen::MatrixXf out_jacobianDotTranslationCstr_var;
    Eigen::MatrixXf out_jacobianOrientationCstr_var;
    Eigen::MatrixXf out_jacobianDotOrientationCstr_var;
    unsigned int DOFsize;
    bool portsArePrepared;
};

