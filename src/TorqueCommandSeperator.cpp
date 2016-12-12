/* Author: Niels Dehio
 * Date:   16 June 2016
 *
 * Description: 
 */

#include "TorqueCommandSeperator.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TorqueCommandSeperator::TorqueCommandSeperator(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &TorqueCommandSeperator::setDOFsize, this).doc("set DOF size");
    addOperation("addChainDOFsize", &TorqueCommandSeperator::addChainDOFsize, this).doc("add Chain DOFsize");
    addOperation("preparePorts", &TorqueCommandSeperator::preparePorts, this).doc("prepare ports");
    addOperation("printCurrentState", &TorqueCommandSeperator::printCurrentState, this).doc("printCurrentState");

    //other stuff
    numOutputPorts = 0;
    portsArePrepared = false;
}

bool TorqueCommandSeperator::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion
    if (!in_torques_port.connected()){
        return false;
    }

    for(unsigned int portNr=0; portNr<numOutputPorts; portNr++){
        if (!out_torques_ports[portNr]->connected()){
            std::cerr << "out_torques_port " << portNr << " is not connected" << std::endl;
            return false;
        }
    }
    return true;
}

bool TorqueCommandSeperator::startHook() {
    // this method starts the component
    return true;
}

void TorqueCommandSeperator::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_torques_port.connected()) {
        // read data and save state of data into "Flow", which can be "NewData", "OldData" or "NoData".
        in_torques_flow = in_torques_port.read(in_torques_var);
        assert(in_torques_var.torques.rows()==DOFsize);
        assert(in_torques_var.torques.cols()==1);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
//    if (in_torques_flow == RTT::NewData || in_torques_flow == RTT::OldData) {
//        //TODO
//    } else if (in_torques_flow == RTT::NoData) {
//        //TODO
//    } else {
//        // there should be something really wrong!
//    }


    unsigned int portNr, counter;
    counter = 0;
    for(portNr=0; portNr<numOutputPorts; portNr++){
        for(unsigned int i=0; i< numChainDOFs.at(portNr); i++){
            out_torques_var[portNr].torques(i) = in_torques_var.torques(counter+i);
        }
        counter=counter+numChainDOFs.at(portNr);
    }
    assert(counter==DOFsize);

    for(portNr=0; portNr<numOutputPorts; portNr++){
        out_torques_ports[portNr]->write(out_torques_var[portNr]);
    }

}

void TorqueCommandSeperator::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TorqueCommandSeperator::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void TorqueCommandSeperator::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void TorqueCommandSeperator::addChainDOFsize(unsigned int ChainDOFsize){
    assert(ChainDOFsize > 0);
    this->numOutputPorts++;
    this->numChainDOFs.push_back(ChainDOFsize);
}

void TorqueCommandSeperator::preparePorts(std::string prefix){
    if (portsArePrepared){
        ports()->removePort("in_torques" + prefix + "_port");
        for(unsigned int portNr=0; portNr<numOutputPorts; portNr++){
            ports()->removePort("out_torques" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        }
    }

    //prepare input
    in_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    in_torques_port.setName("in_torques" + prefix + "_port");
    in_torques_port.doc("Input port for reading torques values");
    ports()->addPort(in_torques_port);
    in_torques_flow = RTT::NoData;

    //prepare output
    for(unsigned int portNr=0; portNr<numOutputPorts; portNr++){
        out_torques_var.push_back(rstrt::dynamics::JointTorques(numChainDOFs.at(portNr)));
        out_torques_var[portNr].torques.setZero();

        boost::shared_ptr< RTT::OutputPort<rstrt::dynamics::JointTorques> > tmpPort(new RTT::OutputPort<rstrt::dynamics::JointTorques>());
        tmpPort->setName("out_torques" + prefix + "_port_" + boost::lexical_cast<std::string>(portNr));
        tmpPort->doc("Output port for sending torques values");
        tmpPort->setDataSample(out_torques_var[portNr]);
        ports()->addPort(*tmpPort);
        out_torques_ports.push_back(tmpPort);
    }

    portsArePrepared = true;
}

void TorqueCommandSeperator::printCurrentState(){
    std::cout << "############## TorqueCommandSeperator State begin " << std::endl;
    std::cout << " in_torques_var.torques " << in_torques_var.torques << std::endl;
    for(unsigned int portNr=0; portNr<numOutputPorts; portNr++){
        std::cout << " out_torques_var[" << portNr << "].torques " << out_torques_var[portNr].torques << std::endl;
    }
    std::cout << "############## TorqueCommandSeperator State end " << std::endl;
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TorqueCommandSeperator)

