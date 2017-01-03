/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <Eigen/Dense>

class TaskDescriberComan: public RTT::TaskContext {
public:
    TaskDescriberComan(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setConstrainedVersionMode(bool useConstrainedVersion);
    void setTranslationOnly(const bool translationOnly);
    void setDOFsize(unsigned int DOFsize);
    void preparePorts();
    void displayCurrentState();
private:
    // Declare input ports and their datatypes
    RTT::InputPort<Eigen::MatrixXf> in_jacobianL_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDotL_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianR_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDotR_port;
    RTT::InputPort<Eigen::VectorXf> in_cartPosL_port;
    RTT::InputPort<Eigen::VectorXf> in_cartVelL_port;
    RTT::InputPort<Eigen::VectorXf> in_cartPosR_port;
    RTT::InputPort<Eigen::VectorXf> in_cartVelR_port;


    // Declare output ports and their datatypes
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotFull_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotTask_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianCstr_port;
    RTT::OutputPort<Eigen::MatrixXf> out_jacobianDotCstr_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartPos_port;
    RTT::OutputPort<Eigen::VectorXf> out_cartVel_port;

    // Data flow:
    RTT::FlowStatus in_jacobianL_flow;
    RTT::FlowStatus in_jacobianDotL_flow;
    RTT::FlowStatus in_jacobianR_flow;
    RTT::FlowStatus in_jacobianDotR_flow;
    RTT::FlowStatus in_cartPosL_flow;
    RTT::FlowStatus in_cartVelL_flow;
    RTT::FlowStatus in_cartPosR_flow;
    RTT::FlowStatus in_cartVelR_flow;

    // variables
    Eigen::MatrixXf in_jacobianL_var;
    Eigen::MatrixXf in_jacobianDotL_var;
    Eigen::MatrixXf in_jacobianR_var;
    Eigen::MatrixXf in_jacobianDotR_var;
    Eigen::VectorXf in_cartPosL_var;
    Eigen::VectorXf in_cartVelL_var;
    Eigen::VectorXf in_cartPosR_var;
    Eigen::VectorXf in_cartVelR_var;

    Eigen::MatrixXf out_jacobianFull_var;
    Eigen::MatrixXf out_jacobianDotFull_var;
    Eigen::MatrixXf out_jacobianTask_var;
    Eigen::MatrixXf out_jacobianDotTask_var;
    Eigen::MatrixXf out_jacobianCstr_var;
    Eigen::MatrixXf out_jacobianDotCstr_var;
    Eigen::VectorXf out_cartPos_var;
    Eigen::VectorXf out_cartVel_var;

    Eigen::MatrixXf selectorTask, selectorCstr;
    unsigned int DOFsize, TaskSpaceDimension, CstrSpaceDimension;
    bool currentMode, useConstrainedVersion, sendTranslationOnly;
    bool portsArePrepared;
};

