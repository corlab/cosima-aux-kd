/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#include "Auxiliaries.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


Auxiliaries::Auxiliaries(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &Auxiliaries::setDOFsize, this).doc("set DOF size");
    addOperation("displayCurrentState", &Auxiliaries::displayCurrentState, this).doc("print current state");

    //other stuff
    portsArePrepared = false;
    identity66 = Eigen::MatrixXf::Identity(6,6);
    tmpeye66 = 0.0 * identity66; //0.00001;
}

bool Auxiliaries::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool Auxiliaries::startHook() {
    // this method starts the component
    return true;
}

void Auxiliaries::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }

    if (in_inertia_port.connected()) {
        in_inertia_flow = in_inertia_port.read(in_inertia_var);
    } else {
        // handle the situation
    }

    if (in_gravity_port.connected()) {
        in_gravity_flow = in_gravity_port.read(in_gravity_var);
    } else {
        // handle the situation
    }

    if (in_coriolis_port.connected()) {
        in_coriolis_flow = in_coriolis_port.read(in_coriolis_var);
    } else {
        // handle the situation
    }

    if (in_jacobian_port.connected()) {
        in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    } else {
        // handle the situation
    }

    if (in_jacobianDot_port.connected()) {
        in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);
    } else {
        // handle the situation
    }


    // you can handle cases when there is no new data.
    if ((in_robotstatus_flow == RTT::NewData || in_robotstatus_flow == RTT::OldData) &&
            (in_inertia_flow == RTT::NewData || in_inertia_flow == RTT::OldData) &&
            (in_gravity_flow == RTT::NewData || in_gravity_flow == RTT::OldData) &&
            (in_coriolis_flow == RTT::NewData || in_coriolis_flow == RTT::OldData) &&
            (in_jacobian_flow == RTT::NewData || in_jacobian_flow == RTT::OldData) &&
            (in_jacobianDot_flow == RTT::NewData || in_jacobianDot_flow == RTT::OldData) ){

        assert(in_robotstatus_var.angles.rows()==DOFsize);
        assert(in_robotstatus_var.angles.cols()==1);
        assert(in_robotstatus_var.velocities.rows()==DOFsize);
        assert(in_robotstatus_var.velocities.cols()==1);
        assert(in_robotstatus_var.torques.rows()==DOFsize);
        assert(in_robotstatus_var.torques.cols()==1);

        assert(in_inertia_var.rows()==DOFsize);
        assert(in_inertia_var.cols()==DOFsize);

        assert(in_gravity_var.rows()==DOFsize);
        assert(in_gravity_var.cols()==1);

        assert(in_coriolis_var.rows()==DOFsize);
        assert(in_coriolis_var.cols()==1);

        assert(in_jacobian_var.rows()==6);
        assert(in_jacobian_var.cols()==DOFsize);

        assert(in_jacobianDot_var.rows()==6);
        assert(in_jacobianDot_var.cols()==DOFsize);


        out_lambda_var = ( in_jacobian_var * in_inertia_var.inverse() * in_jacobian_var.transpose() + tmpeye66).inverse();
        out_pAndMu_var = out_lambda_var * (in_jacobian_var * in_inertia_var.inverse() * (in_gravity_var + in_coriolis_var) - in_jacobianDot_var * in_robotstatus_var.velocities);
        out_pAndMu_var = out_lambda_var * (in_jacobian_var * in_inertia_var.inverse() * in_gravity_var); //TODO: here without coriolis

        out_lambdaTranslation_var = out_lambda_var.topRows<3>();
        out_pAndMuTranslation_var = out_pAndMu_var.topRows<3>();

        out_lambdaOrientation_var = out_lambda_var.bottomRows<3>();
        out_pAndMuOrientation_var = out_pAndMu_var.bottomRows<3>();

    } else {
        out_lambda_var.setZero();
        out_pAndMu_var.setZero();
        out_lambdaTranslation_var.setZero();
        out_pAndMuTranslation_var.setZero();
        out_lambdaOrientation_var.setZero();
        out_pAndMuOrientation_var.setZero();
    }

    out_lambda_port.write(out_lambda_var);
    out_pAndMu_port.write(out_pAndMu_var);
    out_lambdaTranslation_port.write(out_lambdaTranslation_var);
    out_pAndMuTranslation_port.write(out_pAndMuTranslation_var);
    out_lambdaOrientation_port.write(out_lambdaOrientation_var);
    out_pAndMuOrientation_port.write(out_pAndMuOrientation_var);
}

void Auxiliaries::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void Auxiliaries::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void Auxiliaries::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
    this->preparePorts();
}


void Auxiliaries::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("in_inertia_port");
        ports()->removePort("in_gravity_port");
        ports()->removePort("in_coriolis_port");
        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_jacobianDot_port");
        ports()->removePort("out_lambda_port");
        ports()->removePort("out_pAndMu_port");
        ports()->removePort("out_lambdaTranslation_port");
        ports()->removePort("out_pAndMuTranslation_port");
        ports()->removePort("out_lambdaOrientation_port");
        ports()->removePort("out_pAndMuOrientation_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_inertia_var.setZero();
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("Input port for inertia matrix");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

    in_gravity_var = Eigen::VectorXf(DOFsize);
    in_gravity_var.setZero();
    in_gravity_port.setName("in_gravity_port");
    in_gravity_port.doc("Input port for gravity vector");
    ports()->addPort(in_gravity_port);
    in_gravity_flow = RTT::NoData;

    in_coriolis_var = Eigen::VectorXf(DOFsize);
    in_coriolis_var.setZero();
    in_coriolis_port.setName("in_coriolis_port");
    in_coriolis_port.doc("Input port for coriolis vector");
    ports()->addPort(in_coriolis_port);
    in_coriolis_flow = RTT::NoData;

    in_jacobian_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobian_var.setZero();
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for jacobian matrix");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    in_jacobianDot_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianDot_var.setZero();
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc("Input port for jacobianDot matrix");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;


    //prepare output
    out_lambda_var = Eigen::MatrixXf(6,6);
    out_lambda_var.setZero();
    out_lambda_port.setName("out_lambda_port");
    out_lambda_port.doc("Output port for lambda matrix");
    out_lambda_port.setDataSample(out_lambda_var);
    ports()->addPort(out_lambda_port);

    out_pAndMu_var = Eigen::VectorXf(6);
    out_pAndMu_var.setZero();
    out_pAndMu_port.setName("out_pAndMu_port");
    out_pAndMu_port.doc("Output port for pAndMu vector");
    out_pAndMu_port.setDataSample(out_pAndMu_var);
    ports()->addPort(out_pAndMu_port);

    out_lambdaTranslation_var = Eigen::MatrixXf(3,3);
    out_lambdaTranslation_var.setZero();
    out_lambdaTranslation_port.setName("out_lambdaTranslation_port");
    out_lambdaTranslation_port.doc("Output port for lambdaTranslation matrix");
    out_lambdaTranslation_port.setDataSample(out_lambdaTranslation_var);
    ports()->addPort(out_lambdaTranslation_port);

    out_pAndMuTranslation_var = Eigen::VectorXf(3);
    out_pAndMuTranslation_var.setZero();
    out_pAndMuTranslation_port.setName("out_pAndMuTranslation_port");
    out_pAndMuTranslation_port.doc("Output port for pAndMuTranslation vector");
    out_pAndMuTranslation_port.setDataSample(out_pAndMuTranslation_var);
    ports()->addPort(out_pAndMuTranslation_port);

    out_lambdaOrientation_var = Eigen::MatrixXf(3,3);
    out_lambdaOrientation_var.setZero();
    out_lambdaOrientation_port.setName("out_lambdaOrientation_port");
    out_lambdaOrientation_port.doc("Output port for lambdaOrientation matrix");
    out_lambdaOrientation_port.setDataSample(out_lambdaOrientation_var);
    ports()->addPort(out_lambdaOrientation_port);

    out_pAndMuOrientation_var = Eigen::VectorXf(3);
    out_pAndMuOrientation_var.setZero();
    out_pAndMuOrientation_port.setName("out_pAndMuOrientation_port");
    out_pAndMuOrientation_port.doc("Output port for pAndMuOrientation vector");
    out_pAndMuOrientation_port.setDataSample(out_pAndMuOrientation_var);
    ports()->addPort(out_pAndMuOrientation_port);

    portsArePrepared = true;
}


void Auxiliaries::displayCurrentState() {
    std::cout << "############## Auxiliaries State begin " << std::endl;
    std::cout << " angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " inertia " << in_inertia_var << std::endl;
    std::cout << " gravity " << in_gravity_var << std::endl;
    std::cout << " coriolis " << in_coriolis_var << std::endl;
    std::cout << " jacobian " << in_jacobian_var << std::endl;
    std::cout << " jacobianDot " << in_jacobianDot_var << std::endl;
    std::cout << " lambda " << out_lambda_var << std::endl;
    std::cout << " lambdaTranslation " << out_lambdaTranslation_var << std::endl;
    std::cout << " lambdaOrientation " << out_lambdaOrientation_var << std::endl;
    std::cout << " pAndMu " << out_pAndMu_var << std::endl;
    std::cout << " pAndMuTranslation " << out_pAndMuTranslation_var << std::endl;
    std::cout << " pAndMuOrientation " << out_pAndMuOrientation_var << std::endl;
    std::cout << "############## Auxiliaries State end " << std::endl;
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(Auxiliaries)
