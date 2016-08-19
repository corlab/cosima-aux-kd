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
                "inertiaConstrained"), c_constraint_Port("cConstrained"),
        jacobianOriginal_Port("jacobianOriginal"),
        jacobianDotOriginal_Port("jacobianDotOriginal") {

    this->addOperation("calculateAuxiliaries",&ConstrainedAuxiliaries::calculateAuxiliaries, this, RTT::ClientThread);
    this->addOperation("setDOFsize", &ConstrainedAuxiliaries::setDOFsize, this, RTT::ClientThread).doc("set DOF size");

	// input ports
	this->ports()->addPort(inertia_Port).doc("Receiving inertia.");
	this->ports()->addPort(jacobian_Port).doc("Receiving jacobian.");
    this->ports()->addPort(jacobianDot_Port).doc("Receiving jacobian dot.");

	// output ports
	this->ports()->addPort(p_Port).doc("Sending joint feedback.");
	this->ports()->addPort(lambda_constraint_Port).doc("Sending P matrix.");
	this->ports()->addPort(jac_constraint_Port).doc(
			"Sending constrained jacobian.");
    this->ports()->addPort(jac_Dot_constraint_Port).doc(
            "Sending constrained jacobianDot.");

    this->ports()->addPort(jacobianDotOriginal_Port).doc(
            "Receiving jacobianDotOriginal.");

    this->ports()->addPort(jacobianOriginal_Port).doc(
            "Receiving jacobianOriginal.");

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
    jacobianOriginal = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
    jacobianDot = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
    jacobianDotOriginal = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);

    jac_constraint_.resize(TaskSpaceDimension, DOFsize);
    jac_Dot_cstr_.resize(TaskSpaceDimension, DOFsize);
    jac_constraint_Port.setDataSample(jac_constraint_);
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

    tmpeyeDOFsizeDOFsize = 0.001 * identityDOFsizeDOFsize;
    tmpeyeTSdimTSdim = 0.001 * identityTSdimTSdim;
}

void ConstrainedAuxiliaries::updateHook() {
    jacobianOriginal_Flow = jacobianOriginal_Port.read(jacobianOriginal);
    jacobian_Flow = jacobian_Port.read(jacobian);
    jacobianDot_Flow = jacobianDot_Port.read(jacobianDot);
    jacobianDotOriginal_Flow = jacobianDotOriginal_Port.read(jacobianDotOriginal);
    inertia_Flow = inertia_Port.read(M);

    if ((jacobian_Flow != RTT::NoData) && (jacobianDot_Flow != RTT::NoData) && (inertia_Flow != RTT::NoData) && (jacobianOriginal_Flow != RTT::NoData) && (jacobianDotOriginal_Flow != RTT::NoData)) {
        if ((jacobian_Flow == RTT::NewData) || (jacobianDot_Flow == RTT::NewData) || (inertia_Flow == RTT::NewData) || (jacobianOriginal_Flow == RTT::NewData) || (jacobianDotOriginal_Flow == RTT::NewData)) {

            calculateAuxiliaries(jacobian, jacobianDot, M, jacobianOriginal, jacobianDotOriginal);

            // publish items!
            jac_constraint_Port.write(jac_constraint_);
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
    jacobianOriginal_Flow = RTT::NoData;
    jacobianDot_Flow = RTT::NoData;
    jacobianDotOriginal_Flow = RTT::NoData;
	inertia_Flow = RTT::NoData;
	return true;
}

void ConstrainedAuxiliaries::calculateAuxiliaries(const Eigen::MatrixXf& jac_, const Eigen::MatrixXf& jac_Dot_, const Eigen::MatrixXf& M_, const Eigen::MatrixXf& jacobianOriginal_, const Eigen::MatrixXf& jacobianDotOriginal_) {

    // TODO constraint jacobian needs to be moved and need to depend on the constrained matrix!
    jac_constraint_ = jacobianOriginal_;
    jac_Dot_cstr_ = jacobianDotOriginal_;

    if(receiveTranslationOnly){
        jac_constraint_.row(0).setZero();
        jac_constraint_.row(1).setZero();
    //	jac_cstr_.row(2).setZero();

        jac_Dot_cstr_.row(0).setZero();
        jac_Dot_cstr_.row(1).setZero();
    //	jac_Dot_cstr_.row(2).setZero();
    }
    else{
        jac_constraint_.row(0).setZero();
        jac_constraint_.row(1).setZero();
    //	jac_cstr_.row(2).setZero();
        jac_constraint_.row(3).setZero();
        jac_constraint_.row(4).setZero();
    //	jac_cstr_.row(5).setZero();

        jac_Dot_cstr_.row(0).setZero();
        jac_Dot_cstr_.row(1).setZero();
    //	jac_Dot_cstr_.row(2).setZero();
        jac_Dot_cstr_.row(3).setZero();
        jac_Dot_cstr_.row(4).setZero();
    //	jac_Dot_cstr_.row(5).setZero();
    }

    //Eq. under Eq. 10
    jac_cstr_MPI = (jac_constraint_.transpose() * jac_constraint_ + tmpeyeDOFsizeDOFsize).inverse() * jac_constraint_.transpose();

    //Eq. under Eq. 10
    P = identityDOFsizeDOFsize - (jac_cstr_MPI * jac_constraint_);

    RTT::log(RTT::Warning) << "jac_\n" << jac_ << RTT::endlog();
    RTT::log(RTT::Warning) << "jac_cstr_MPI\n" << jac_cstr_MPI << RTT::endlog();
    RTT::log(RTT::Warning) << "jac_cstr_\n" << jac_constraint_ << RTT::endlog();

    //Eq. under Eq. 11
    M_cstr_ = P * M_ + identityDOFsizeDOFsize - P;

    RTT::log(RTT::Warning) << "M_cstr_\n" << M_cstr_ << RTT::endlog();

    //Eq. under Eq. 11
    //C_cstr_ = -(jac_cstr_MPI * jac_cstr_); //TODO which one is correct???
    C_cstr_ = -(jac_cstr_MPI * jac_Dot_cstr_); //TODO

    RTT::log(RTT::Warning) << "C_cstr_\n" << C_cstr_ << RTT::endlog();

    //Eq. under Eq. 11
    Lamda_cstr = (jac_ * M_cstr_.inverse() * P * jac_.transpose() + tmpeyeTSdimTSdim).inverse();

    RTT::log(RTT::Warning) << "Lamda_cstr\n" << Lamda_cstr << RTT::endlog();

    //Eq. 14
    //jac_MPI = (jac_ * M_cstr_.inverse() * P * jac_.transpose() + tmpeyeTSdimTSdim).inverse() * jac_ * M_cstr_.inverse() * P;
    jac_MPI = Lamda_cstr * jac_ * M_cstr_.inverse() * P;

    RTT::log(RTT::Warning) << "jac_MPI\n" << jac_MPI << RTT::endlog();
}

ORO_LIST_COMPONENT_TYPE(cosima::ConstrainedAuxiliaries)

