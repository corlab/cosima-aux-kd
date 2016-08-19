#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include "constrained-auxiliaries.hpp"

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

ConstrainedAuxiliaries::ConstrainedAuxiliaries(const std::string &name) :
        TaskContext(name), jacobian_Flow(RTT::NoData), jacobianDot_Flow(RTT::NoData), inertia_Port(
				"inertia"), inertia_Flow(RTT::NoData), jacobian_Port(
                "jacobian"), jacobianDot_Port("jacobianDot"),  p_Port("pMatrix"), lambda_constraint_Port(
                "lambdaConstrained"), jac_constraint_Port("jacConstrained"), jac_Dot_constraint_Port("jacConstrained"),
        jac_constraint_mpi_Port("jacConstrainedMPI"), jac_mpi_Port("jacMPI"), inertia_constraint_Port(
				"inertiaConstrained"), c_constraint_Port("cConstrained") {

    this->addOperation("calculateAuxiliaries",&ConstrainedAuxiliaries::calculateAuxiliaries, this, RTT::ClientThread);
    this->addOperation("setDOFsize", &ConstrainedAuxiliaries::setDOFsize, this, RTT::ClientThread).doc("set DOF size");

	// input ports
	this->ports()->addPort(inertia_Port).doc("Receiving inertia.");
	this->ports()->addPort(jacobian_Port).doc("Receiving jacobian.");

	// output ports
	this->ports()->addPort(p_Port).doc("Sending joint feedback.");
	this->ports()->addPort(lambda_constraint_Port).doc("Sending P matrix.");
	this->ports()->addPort(jac_constraint_Port).doc(
			"Sending constrained jacobian.");
    this->ports()->addPort(jac_Dot_constraint_Port).doc(
            "Sending constrained jacobianDot.");
	this->ports()->addPort(jac_constraint_mpi_Port).doc(
			"Sending constrained jacobian MPI.");
    this->ports()->addPort(jac_mpi_Port).doc(
            "Sending jacobian MPI.");
	this->ports()->addPort(inertia_constraint_Port).doc(
			"Sending constrained inertia.");
	this->ports()->addPort(c_constraint_Port).doc("Sending constrained C.");

    receiveTranslationOnly = true;
    if(receiveTranslationOnly){
        TaskSpaceDimension = 3;
    }
    else{
        TaskSpaceDimension = 6;
    }
}

void ConstrainedAuxiliaries::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;

    M = Eigen::MatrixXf::Zero(DOFsize,DOFsize);

    jacobian = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
    jacobianDot = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);

    jac_cstr_.resize(TaskSpaceDimension, DOFsize);
    jac_Dot_cstr_.resize(TaskSpaceDimension, DOFsize);
    jac_constraint_Port.setDataSample(jac_cstr_);
    jac_Dot_constraint_Port.setDataSample(jac_Dot_cstr_);

    jac_cstr_MPI.resize(DOFsize, TaskSpaceDimension);
    jac_MPI.resize(DOFsize, TaskSpaceDimension);
    jac_constraint_mpi_Port.setDataSample(jac_cstr_MPI);
    jac_mpi_Port.setDataSample(jac_MPI);

    P.resize(DOFsize, DOFsize);
    p_Port.setDataSample(P);

    Lamda_cstr.resize(TaskSpaceDimension,TaskSpaceDimension);
    lambda_constraint_Port.setDataSample(Lamda_cstr);

    M_cstr_.resize(DOFsize, DOFsize);
    inertia_constraint_Port.setDataSample(M_cstr_);

    C_cstr_.resize(DOFsize,DOFsize);
    c_constraint_Port.setDataSample(C_cstr_);

    identityDOFsizeDOFsize.resize(DOFsize,DOFsize);
    identityDOFsizeDOFsize = Eigen::MatrixXf::Identity(DOFsize,DOFsize);

    identityTSdimTSdim.resize(TaskSpaceDimension,TaskSpaceDimension);
    identityTSdimTSdim = Eigen::MatrixXf::Identity(TaskSpaceDimension,TaskSpaceDimension);

    tmpeyeDOFsizeDOFsize.resize(DOFsize, DOFsize);
    tmpeyeTSdimTSdim.resize(TaskSpaceDimension, TaskSpaceDimension);

    tmpeyeDOFsizeDOFsize = 0.0001 * identityDOFsizeDOFsize;
    tmpeyeTSdimTSdim = 0.0001 * identityTSdimTSdim;
}

void ConstrainedAuxiliaries::updateHook() {

    jacobian_Flow = jacobian_Port.read(jacobian);
    jacobianDot_Flow = jacobianDot_Port.read(jacobianDot);
    inertia_Flow = inertia_Port.read(M);
    if ((jacobian_Flow != RTT::NoData) && (jacobianDot_Flow != RTT::NoData) && (inertia_Flow != RTT::NoData)) {
        if ((jacobian_Flow == RTT::NewData) || (jacobianDot_Flow == RTT::NewData) || (inertia_Flow == RTT::NewData)) {

            calculateAuxiliaries(jacobian, jacobianDot, M);

            // publish items!
            jac_constraint_Port.write(jac_cstr_);
            jac_Dot_constraint_Port.write(jac_Dot_cstr_);

            jac_constraint_mpi_Port.write(jac_cstr_MPI);
            jac_mpi_Port.write(jac_MPI);

            p_Port.write(P);

            lambda_constraint_Port.write(Lamda_cstr);

            inertia_constraint_Port.write(M_cstr_);

            c_constraint_Port.write(C_cstr_);

        }

    }
}

bool ConstrainedAuxiliaries::startHook() {
    return true;
}

bool ConstrainedAuxiliaries::configureHook() {
	jacobian_Flow = RTT::NoData;
    jacobianDot_Flow = RTT::NoData;
	inertia_Flow = RTT::NoData;
	return true;
}


void ConstrainedAuxiliaries::calculateAuxiliaries(const Eigen::MatrixXf& jac_, const Eigen::MatrixXf& jac_Dot_,
        const Eigen::MatrixXf& M_) {

    // TODO constraint jacobian needs to be moved and need to depend on the constrained matrix!
    jac_cstr_ = jac_;
    jac_Dot_cstr_ = jac_Dot_;

    if(receiveTranslationOnly){
        jac_cstr_.row(0).setZero();
        jac_cstr_.row(1).setZero();
    //	jac_cstr_.row(2).setZero();

        jac_Dot_cstr_.row(0).setZero();
        jac_Dot_cstr_.row(1).setZero();
    //	jac_Dot_cstr_.row(2).setZero();
    }
    else{
        jac_cstr_.row(0).setZero();
        jac_cstr_.row(1).setZero();
    //	jac_cstr_.row(2).setZero();
        jac_cstr_.row(3).setZero();
        jac_cstr_.row(4).setZero();
    //	jac_cstr_.row(5).setZero();

        jac_Dot_cstr_.row(0).setZero();
        jac_Dot_cstr_.row(1).setZero();
    //	jac_Dot_cstr_.row(2).setZero();
        jac_Dot_cstr_.row(3).setZero();
        jac_Dot_cstr_.row(4).setZero();
    //	jac_Dot_cstr_.row(5).setZero();
    }

    //Eq. under Eq. 10
    jac_cstr_MPI = (jac_cstr_.transpose() * jac_cstr_ + tmpeyeDOFsizeDOFsize).inverse() * jac_cstr_.transpose();

    //Eq. under Eq. 10
    P = identityDOFsizeDOFsize - (jac_cstr_MPI * jac_cstr_);

    //Eq. under Eq. 11
    M_cstr_ = P * M_ + identityDOFsizeDOFsize - P;

    //Eq. under Eq. 11
    //C_cstr_ = -(jac_cstr_MPI * jac_cstr_); //TODO which one is correct???
    C_cstr_ = -(jac_cstr_MPI * jac_Dot_cstr_); //TODO

    //Eq. under Eq. 11
    Lamda_cstr = (jac_ * M_cstr_.inverse() * P * jac_.transpose() + tmpeyeTSdimTSdim).inverse();

    //Eq. 14
    jac_MPI = (jac_ * M_cstr_.inverse() * P * jac_.transpose()).inverse() * jac_ * M_cstr_.inverse() * P;
}

ORO_LIST_COMPONENT_TYPE(cosima::ConstrainedAuxiliaries)

