/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "FeedbackCombiner.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


FeedbackCombiner::FeedbackCombiner(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &FeedbackCombiner::setDOFsize, this).doc("set DOF size");
    addOperation("addChainDOFsize", &FeedbackCombiner::addChainDOFsize, this).doc("add Chain DOFsize");
    addOperation("preparePorts", &FeedbackCombiner::preparePorts, this).doc("prepare ports");
    addOperation("printCurrentState", &FeedbackCombiner::printCurrentState, this).doc("printCurrentState");

    //other stuff
    numInputPorts = 0;
    portsArePrepared = false;
}

bool FeedbackCombiner::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion
    for(unsigned int portNr=0; portNr<numInputPorts; portNr++){
        if (!in_robotstatus_ports[portNr]->connected()){
            std::cerr << "in_robotstatus_port " << portNr << " is not connected" << std::endl;
            return false;
        }
    }

    if (!out_robotstatus_port.connected())
        return false;
    else
        return true;
}

bool FeedbackCombiner::startHook() {
    // this method starts the component
    return true;
}

void FeedbackCombiner::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    out_robotstatus_var.angles.setZero();
    out_robotstatus_var.velocities.setZero();
    out_robotstatus_var.torques.setZero();
    unsigned int counter = 0;
    for(unsigned int portNr=0; portNr<numInputPorts; portNr++){
        if (in_robotstatus_ports[portNr]->connected()) {
            // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
            in_robotstatus_flow[portNr] = in_robotstatus_ports[portNr]->read(in_robotstatus_var[portNr]);
            assert(in_robotstatus_var[portNr].torques.rows()==numChainDOFs.at(portNr));
            assert(in_robotstatus_var[portNr].torques.cols()==1);
        } else {
            // handle the situation
        }

        // you can handle cases when there is no new data.
        if (in_robotstatus_flow[portNr] == RTT::NewData || in_robotstatus_flow[portNr] == RTT::OldData){
            for(unsigned int jointID=0; jointID<in_robotstatus_var[portNr].torques.size(); jointID++){
                out_robotstatus_var.angles(counter)     += in_robotstatus_var[portNr].angles(jointID);
                out_robotstatus_var.velocities(counter) += in_robotstatus_var[portNr].velocities(jointID);
                out_robotstatus_var.torques(counter)    += in_robotstatus_var[portNr].torques(jointID);
                counter++;
            }
        } else if (in_robotstatus_flow[portNr] == RTT::NoData){
            assert(false); //TODO
        } else {
            // there should be something really wrong!
        }
    }
    assert(counter == DOFsize);
    out_robotstatus_port.write(out_robotstatus_var);
}

void FeedbackCombiner::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void FeedbackCombiner::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void FeedbackCombiner::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void FeedbackCombiner::addChainDOFsize(unsigned int ChainDOFsize){
    assert(ChainDOFsize > 0);
    this->numInputPorts++;
    this->numChainDOFs.push_back(ChainDOFsize);
}

void FeedbackCombiner::preparePorts(std::string prefix){
    if (portsArePrepared){
        for(unsigned int portNr=0; portNr<numInputPorts; portNr++){
            ports()->removePort("in_robotstatus" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        }
        ports()->removePort("out_robotstatus" + prefix + "_port");
    }

    //prepare input
    for(unsigned int portNr=0; portNr<numInputPorts; portNr++){
        in_robotstatus_var.push_back(rstrt::robot::JointState(numChainDOFs.at(portNr)));

        boost::shared_ptr< RTT::InputPort<rstrt::robot::JointState> > tmpPort(new RTT::InputPort<rstrt::robot::JointState>());
        tmpPort->setName("in_robotstatus" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        tmpPort->doc("Input port for torque values");
        ports()->addPort(*tmpPort);
        in_robotstatus_flow.push_back(RTT::NoData);
        in_robotstatus_ports.push_back(tmpPort);
    }

    //prepare output
    out_robotstatus_var = rstrt::robot::JointState(DOFsize);
    out_robotstatus_var.torques.setZero();
    out_robotstatus_port.setName("out_robotstatus" + prefix + "_port");
    out_robotstatus_port.doc("Output port for sending cost value");
    out_robotstatus_port.setDataSample(out_robotstatus_var);
    ports()->addPort(out_robotstatus_port);

    portsArePrepared = true;
}

void FeedbackCombiner::printCurrentState(){
    std::cout << "############## FeedbackCombiner State begin " << std::endl;
    for(unsigned int portNr=0; portNr<numInputPorts; portNr++){
        std::cout << " in_robotstatus_var[" << portNr << "].angles " << in_robotstatus_var[portNr].angles << std::endl;
        std::cout << " in_robotstatus_var[" << portNr << "].velocities " << in_robotstatus_var[portNr].velocities << std::endl;
        std::cout << " in_robotstatus_var[" << portNr << "].torques " << in_robotstatus_var[portNr].torques << std::endl;
    }
    std::cout << " out_robotstatus_var.angles " << out_robotstatus_var.angles << std::endl;
    std::cout << " out_robotstatus_var.velocities " << out_robotstatus_var.velocities << std::endl;
    std::cout << " out_robotstatus_var.torques " << out_robotstatus_var.torques << std::endl;
    std::cout << "############## FeedbackCombiner State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(FeedbackCombiner)

