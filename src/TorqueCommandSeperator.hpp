/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

#include <rst-rt/dynamics/JointTorques.hpp>
#include <boost/lexical_cast.hpp>

class TorqueCommandSeperator: public RTT::TaskContext {
public:
    TorqueCommandSeperator(std::string const & name);

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
    RTT::InputPort<rstrt::dynamics::JointTorques> in_torques_port;

    // Declare output ports and their datatypes
    std::vector<boost::shared_ptr< RTT::OutputPort<rstrt::dynamics::JointTorques> > > out_torques_ports;

    // Data flow:
    RTT::FlowStatus in_torques_flow;

    // variables
    rstrt::dynamics::JointTorques in_torques_var;
    std::vector<rstrt::dynamics::JointTorques> out_torques_var;
    unsigned int DOFsize;
    unsigned int numOutputPorts;
    std::vector<unsigned int> numChainDOFs;
    bool portsArePrepared;
};

