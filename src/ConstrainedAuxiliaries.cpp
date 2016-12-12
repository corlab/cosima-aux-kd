/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#include "ConstrainedAuxiliaries.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


ConstrainedAuxiliaries::ConstrainedAuxiliaries(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &ConstrainedAuxiliaries::setDOFsize, this).doc("set DOF size");
    addOperation("setTaskSpaceDimension", &ConstrainedAuxiliaries::setTaskSpaceDimension, this).doc("set TaskSpaceDimension");
    addOperation("setCstrSpaceDimension", &ConstrainedAuxiliaries::setCstrSpaceDimension, this).doc("set CstrSpaceDimension");
    addOperation("preparePorts", &ConstrainedAuxiliaries::preparePorts, this).doc("prepare ports");
    addOperation("displayCurrentState", &ConstrainedAuxiliaries::displayCurrentState, this).doc("print current state");

    //other stuff
    portsArePrepared = false;
}

bool ConstrainedAuxiliaries::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool ConstrainedAuxiliaries::startHook() {
    // this method starts the component
    return true;
}

void ConstrainedAuxiliaries::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_inertia_port.connected()) {
        in_inertia_flow = in_inertia_port.read(in_inertia_var);
    } else {
        // handle the situation
    }

    if (in_jacobianTask_port.connected()) {
        in_jacobianTask_flow = in_jacobianTask_port.read(in_jacobianTask_var);
    } else {
        // handle the situation
    }

    if (in_jacobianDotTask_port.connected()) {
        in_jacobianDotTask_flow = in_jacobianDotTask_port.read(in_jacobianDotTask_var);
    } else {
        // handle the situation
    }

    if (in_jacobianCstr_port.connected()) {
        in_jacobianCstr_flow = in_jacobianCstr_port.read(in_jacobianCstr_var);
    } else {
        // handle the situation
    }

    if (in_jacobianDotCstr_port.connected()) {
        in_jacobianDotCstr_flow = in_jacobianDotCstr_port.read(in_jacobianDotCstr_var);
    } else {
        // handle the situation
    }


    // you can handle cases when there is no new data.
    if (    (in_inertia_flow == RTT::NewData || in_inertia_flow == RTT::OldData) &&
            (in_jacobianTask_flow == RTT::NewData || in_jacobianTask_flow == RTT::OldData) &&
            (in_jacobianDotTask_flow == RTT::NewData || in_jacobianDotTask_flow == RTT::OldData) &&
            (in_jacobianCstr_flow == RTT::NewData || in_jacobianCstr_flow == RTT::OldData) &&
            (in_jacobianDotCstr_flow == RTT::NewData || in_jacobianDotCstr_flow == RTT::OldData)){

        assert(in_inertia_var.rows()==DOFsize);
        assert(in_inertia_var.cols()==DOFsize);

        assert(in_jacobianTask_var.rows()==TaskSpaceDimension);
        assert(in_jacobianTask_var.cols()==DOFsize);

        assert(in_jacobianDotTask_var.rows()==TaskSpaceDimension);
        assert(in_jacobianDotTask_var.cols()==DOFsize);

        assert(in_jacobianCstr_var.rows()==TaskSpaceDimension);
        assert(in_jacobianCstr_var.cols()==DOFsize);

        assert(in_jacobianDotCstr_var.rows()==TaskSpaceDimension);
        assert(in_jacobianDotCstr_var.cols()==DOFsize);



        //Eq. under Eq. 10
        // DOFsize x CSdim = (DOFsize x CSdim * CSdim x DOFsize) * DOFsize x CSdim
        out_jacobianCstrMPI_var = (in_jacobianCstr_var.transpose() * in_jacobianCstr_var + tmpeyeDOFsizeDOFsize).inverse() * in_jacobianCstr_var.transpose();
//        RTT::log(RTT::Warning) << "out_jacobianCstrMPI_var\n" << out_jacobianCstrMPI_var << RTT::endlog();

        //Eq. under Eq. 10
        // DOFsize x DOFsize = DOFsize x CSdim * CSdim x DOFsize
        out_P_var = identityDOFsizeDOFsize - (out_jacobianCstrMPI_var * in_jacobianCstr_var);
//        RTT::log(RTT::Warning) << "out_P_var\n" << out_P_var << RTT::endlog();

        //Eq. under Eq. 11
        // DOFsize x DOFsize = DOFsize x DOFsize * DOFsize x DOFsize
        out_MCstr_var = out_P_var * in_inertia_var + identityDOFsizeDOFsize - out_P_var;
//        RTT::log(RTT::Warning) << "out_MCstr_var\n" << out_MCstr_var << RTT::endlog();

        //Eq. under Eq. 11
        // DOFsize x DOFsize = DOFsize x CSdim * CSdim x DOFsize
        out_CCstr_var = -(out_jacobianCstrMPI_var * in_jacobianDotCstr_var);
//        RTT::log(RTT::Warning) << "out_CCstr_var\n" << out_CCstr_var << RTT::endlog();

        //Eq. under Eq. 11
        // TSdim x TSdim = TSdim x DOFsize * DOFsize x DOFsize * DOFsize x DOFsize * DOFsize x TSdim
        out_lambdaCstr_var = (in_jacobianTask_var * out_MCstr_var.inverse() * out_P_var * in_jacobianTask_var.transpose() + tmpeyeTSdimTSdim).inverse();
//        RTT::log(RTT::Warning) << "out_lambdaCstr_var\n" << out_lambdaCstr_var << RTT::endlog();

        //Eq. 14
        // TSdim x DOFsize = TSdim x TSdim * TSdim x DOFsize * DOFsize x DOFsize * DOFsize x DOFsize
//        out_jacobianMPI_var = (in_jacobianTask_var * out_MCstr_var.inverse() * out_P_var * in_jacobianTask_var.transpose() + tmpeyeTSdimTSdim).inverse() * in_jacobianTask_var * out_MCstr_var.inverse() * out_P_var;
        out_jacobianMPI_var = out_lambdaCstr_var * in_jacobianTask_var * out_MCstr_var.inverse() * out_P_var;
//        RTT::log(RTT::Warning) << "out_jacobianMPI_var\n" << out_jacobianMPI_var << RTT::endlog();

    } else {
        out_lambdaCstr_var.setZero();
        out_MCstr_var.setZero();
        out_CCstr_var.setZero();
        out_jacobianCstrMPI_var.setZero();
        out_jacobianMPI_var.setZero();
        out_P_var.setZero();
    }

    out_lambdaCstr_port.write(out_lambdaCstr_var);
    out_MCstr_port.write(out_MCstr_var);
    out_CCstr_port.write(out_CCstr_var);
    out_jacobianCstrMPI_port.write(out_jacobianCstrMPI_var);
    out_jacobianMPI_port.write(out_jacobianMPI_var);
    out_P_port.write(out_P_var);
}

void ConstrainedAuxiliaries::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ConstrainedAuxiliaries::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}


void ConstrainedAuxiliaries::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;

    identityDOFsizeDOFsize = Eigen::MatrixXf::Identity(DOFsize,DOFsize);
    tmpeyeDOFsizeDOFsize = 0.001 * identityDOFsizeDOFsize;
}

void ConstrainedAuxiliaries::setTaskSpaceDimension(unsigned int TaskSpaceDimension){
    assert(TaskSpaceDimension > 0);
    this->TaskSpaceDimension = TaskSpaceDimension;

    identityTSdimTSdim = Eigen::MatrixXf::Identity(TaskSpaceDimension,TaskSpaceDimension);
    tmpeyeTSdimTSdim = 0.001 * identityTSdimTSdim;
}

void ConstrainedAuxiliaries::setCstrSpaceDimension(unsigned int CstrSpaceDimension){
    assert(CstrSpaceDimension > 0);
    this->CstrSpaceDimension = CstrSpaceDimension;
}

void ConstrainedAuxiliaries::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_inertia_port");
        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_jacobianDot_port");
        ports()->removePort("out_lambdaCstr_port");
        ports()->removePort("out_MCstr_port");
        ports()->removePort("out_CCstr_port");
        ports()->removePort("out_jacobianCstrMPI_port");
        ports()->removePort("out_jacobianMPI_port");
        ports()->removePort("out_P_port");
    }

    //prepare input
    in_inertia_var = Eigen::MatrixXf(DOFsize,DOFsize);
    in_inertia_var.setZero();
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("Input port for inertia matrix");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;

    in_jacobianTask_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    in_jacobianTask_var.setZero();
    in_jacobianTask_port.setName("in_jacobianTask_port");
    in_jacobianTask_port.doc("Input port for jacobian matrix");
    ports()->addPort(in_jacobianTask_port);
    in_jacobianTask_flow = RTT::NoData;

    in_jacobianDotTask_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    in_jacobianDotTask_var.setZero();
    in_jacobianDotTask_port.setName("in_jacobianDotTask_port");
    in_jacobianDotTask_port.doc("Input port for jacobianDot matrix");
    ports()->addPort(in_jacobianDotTask_port);
    in_jacobianDotTask_flow = RTT::NoData;

    in_jacobianCstr_var = Eigen::MatrixXf(CstrSpaceDimension,DOFsize);
    in_jacobianCstr_var.setZero();
    in_jacobianCstr_port.setName("in_jacobianCstr_port");
    in_jacobianCstr_port.doc("Input port for jacobian matrix");
    ports()->addPort(in_jacobianCstr_port);
    in_jacobianCstr_flow = RTT::NoData;

    in_jacobianDotCstr_var = Eigen::MatrixXf(CstrSpaceDimension,DOFsize);
    in_jacobianDotCstr_var.setZero();
    in_jacobianDotCstr_port.setName("in_jacobianDotCstr_port");
    in_jacobianDotCstr_port.doc("Input port for jacobianDot matrix");
    ports()->addPort(in_jacobianDotCstr_port);
    in_jacobianDotCstr_flow = RTT::NoData;


    //prepare output
    out_lambdaCstr_var = Eigen::MatrixXf(TaskSpaceDimension,TaskSpaceDimension);
    out_lambdaCstr_var.setZero();
    out_lambdaCstr_port.setName("out_lambdaCstr_port");
    out_lambdaCstr_port.doc("Output port for lambdaCstr matrix");
    out_lambdaCstr_port.setDataSample(out_lambdaCstr_var);
    ports()->addPort(out_lambdaCstr_port);

    out_MCstr_var = Eigen::MatrixXf(DOFsize, DOFsize);
    out_MCstr_var.setZero();
    out_MCstr_port.setName("out_MCstr_port");
    out_MCstr_port.doc("Output port for MCstr_ vector");
    out_MCstr_port.setDataSample(out_MCstr_var);
    ports()->addPort(out_MCstr_port);

    out_CCstr_var = Eigen::MatrixXf(DOFsize, DOFsize);
    out_CCstr_var.setZero();
    out_CCstr_port.setName("out_CCstr_port");
    out_CCstr_port.doc("Output port for CCstr matrix");
    out_CCstr_port.setDataSample(out_CCstr_var);
    ports()->addPort(out_CCstr_port);

    out_jacobianCstrMPI_var = Eigen::MatrixXf(DOFsize, CstrSpaceDimension);
    out_jacobianCstrMPI_var.setZero();
    out_jacobianCstrMPI_port.setName("out_jacobianCstrMPI_port");
    out_jacobianCstrMPI_port.doc("Output port for jacobianCstrMPI vector");
    out_jacobianCstrMPI_port.setDataSample(out_jacobianCstrMPI_var);
    ports()->addPort(out_jacobianCstrMPI_port);

    out_jacobianMPI_var = Eigen::MatrixXf(TaskSpaceDimension, DOFsize);
    out_jacobianMPI_var.setZero();
    out_jacobianMPI_port.setName("out_jacobianMPI_port");
    out_jacobianMPI_port.doc("Output port for jacobianMPI matrix");
    out_jacobianMPI_port.setDataSample(out_jacobianMPI_var);
    ports()->addPort(out_jacobianMPI_port);

    out_P_var = Eigen::MatrixXf(DOFsize, DOFsize);
    out_P_var.setZero();
    out_P_port.setName("out_P_port");
    out_P_port.doc("Output port for P vector");
    out_P_port.setDataSample(out_P_var);
    ports()->addPort(out_P_port);

    portsArePrepared = true;
}


void ConstrainedAuxiliaries::displayCurrentState() {
    std::cout << "############## ConstrainedAuxiliaries State begin " << std::endl;
    std::cout << " inertia " << in_inertia_var << std::endl;
    std::cout << " jacobianTask " << in_jacobianTask_var << std::endl;
    std::cout << " jacobianDotTask " << in_jacobianDotTask_var << std::endl;
    std::cout << " jacobianCstr " << in_jacobianCstr_var << std::endl;
    std::cout << " jacobianDotCstr " << in_jacobianDotCstr_var << std::endl;

    std::cout << " lambdaCstr " << out_lambdaCstr_var << std::endl;
    std::cout << " MCstr " << out_MCstr_var << std::endl;
    std::cout << " CCstr " << out_CCstr_var << std::endl;
    std::cout << " jacobianCstrMPI " << out_jacobianCstrMPI_var << std::endl;
    std::cout << " jacobianMPI " << out_jacobianMPI_var << std::endl;
    std::cout << " P " << out_P_var << std::endl;
    std::cout << "############## ConstrainedAuxiliaries State end " << std::endl;
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(ConstrainedAuxiliaries)
