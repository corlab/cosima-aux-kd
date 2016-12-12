/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/robot/JointState.hpp>
#include <boost/lexical_cast.hpp>

class FeedbackCombiner: public RTT::TaskContext {
public:
    FeedbackCombiner(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void addChainDOFsize(unsigned int ChainDOFsize);
    void preparePorts(std::string prefix);
    void printCurrentState();

private:
    // Declare input ports and their datatypes
    std::vector<boost::shared_ptr< RTT::InputPort<rstrt::robot::JointState> > > in_robotstatus_ports;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::robot::JointState> out_robotstatus_port;

    // Data flow:
    std::vector<RTT::FlowStatus> in_robotstatus_flow;

    // variables
    std::vector<rstrt::robot::JointState> in_robotstatus_var;
    rstrt::robot::JointState out_robotstatus_var;
    unsigned int DOFsize;
    unsigned int numInputPorts;
    std::vector<unsigned int> numChainDOFs;
    bool portsArePrepared;
};

