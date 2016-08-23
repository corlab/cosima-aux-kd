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
        TaskContext(name), jac_Flow(RTT::NoData), jacDot_Flow(RTT::NoData), inertia_Port(
				"inertia"), inertia_Flow(RTT::NoData), jac_Port(
                "jac"), jacDot_Port("jacDot"),  p_Port("pMatrix"), lambda_cstr_Port(
                "lambdaConstrained"), jac_cstr_Port("jac_cstr"), jac_Dot_cstr_Port("jacDot_cstr"),
        jac_cstr_mpi_Port("jac_cstrMPI"), jac_mpi_Port("jacMPI"), inertia_cstr_Port(
                "inertiaConstrained"), c_cstr_Port("cConstrained"),
        jac_full_Port("jac_full"),
        jacDot_full_Port("jacDot_full"),
		receiveTranslationOnly(true) {

    this->addOperation("calculateAuxiliaries",&ConstrainedAuxiliaries::calculateAuxiliaries, this, RTT::ClientThread);
    this->addOperation("setDOFsize", &ConstrainedAuxiliaries::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    this->addOperation("setTranslationOnly", &ConstrainedAuxiliaries::setTranslationOnly, this, RTT::ClientThread).doc("set translation only, or use also orientation");

	// input ports
	this->ports()->addPort(inertia_Port).doc("Receiving inertia.");
	this->ports()->addPort(jac_Port).doc("Receiving jac.");
    this->ports()->addPort(jacDot_Port).doc("Receiving jac dot.");

	// output ports
	this->ports()->addPort(p_Port).doc("Sending joint feedback.");
	this->ports()->addPort(lambda_cstr_Port).doc("Sending P matrix.");
	this->ports()->addPort(jac_cstr_Port).doc(
			"Sending constrained jac.");
    this->ports()->addPort(jac_Dot_cstr_Port).doc(
            "Sending constrained jacDot.");

    this->ports()->addPort(jacDot_full_Port).doc(
            "Receiving jacDot_full.");

    this->ports()->addPort(jac_full_Port).doc(
            "Receiving jac_full.");

	this->ports()->addPort(jac_cstr_mpi_Port).doc(
			"Sending constrained jac MPI.");
    this->ports()->addPort(jac_mpi_Port).doc(
            "Sending jac MPI.");
	this->ports()->addPort(inertia_cstr_Port).doc(
			"Sending constrained inertia.");
	this->ports()->addPort(c_cstr_Port).doc("Sending constrained C.");
}

void ConstrainedAuxiliaries::setTranslationOnly(const bool translationOnly) {
    receiveTranslationOnly = translationOnly;
    if(receiveTranslationOnly) {
        TaskSpaceDimension = 3;
    }
    else{
        TaskSpaceDimension = 6;
    }

    M = Eigen::MatrixXf::Zero(DOFsize,DOFsize);

   jac = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
   jac_full = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
   jacDot = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);
   jacDot_full = Eigen::MatrixXf::Zero(TaskSpaceDimension,DOFsize);

   jac_cstr_.resize(TaskSpaceDimension, DOFsize);
   jac_Dot_cstr_.resize(TaskSpaceDimension, DOFsize);
   jac_cstr_Port.setDataSample(jac_cstr_);
   jac_Dot_cstr_Port.setDataSample(jac_Dot_cstr_);

   jac_cstr_MPI.resize(DOFsize, TaskSpaceDimension);
   jac_MPI.resize(DOFsize, TaskSpaceDimension);
   jac_cstr_mpi_Port.setDataSample(jac_cstr_MPI);
   jac_mpi_Port.setDataSample(jac_MPI);

   P.resize(DOFsize, DOFsize);
   p_Port.setDataSample(P);

   Lamda_cstr.resize(TaskSpaceDimension,TaskSpaceDimension);
   lambda_cstr_Port.setDataSample(Lamda_cstr);

   M_cstr_.resize(DOFsize, DOFsize);
   inertia_cstr_Port.setDataSample(M_cstr_);

   C_cstr_.resize(DOFsize,DOFsize);
   c_cstr_Port.setDataSample(C_cstr_);

   identityDOFsizeDOFsize.resize(DOFsize,DOFsize);
   identityDOFsizeDOFsize = Eigen::MatrixXf::Identity(DOFsize,DOFsize);

   identityTSdimTSdim.resize(TaskSpaceDimension,TaskSpaceDimension);
   identityTSdimTSdim = Eigen::MatrixXf::Identity(TaskSpaceDimension,TaskSpaceDimension);

   tmpeyeDOFsizeDOFsize.resize(DOFsize, DOFsize);
   tmpeyeTSdimTSdim.resize(TaskSpaceDimension, TaskSpaceDimension);

   tmpeyeDOFsizeDOFsize = 0.001 * identityDOFsizeDOFsize;
   tmpeyeTSdimTSdim = 0.001 * identityTSdimTSdim;
}

void ConstrainedAuxiliaries::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;
}

void ConstrainedAuxiliaries::updateHook() {
    jac_full_Flow = jac_full_Port.read(jac_full);
    jac_Flow = jac_Port.read(jac);
    jacDot_Flow = jacDot_Port.read(jacDot);
    jacDot_full_Flow = jacDot_full_Port.read(jacDot_full);
    inertia_Flow = inertia_Port.read(M);

    if ((jac_Flow != RTT::NoData) && (jacDot_Flow != RTT::NoData) && (inertia_Flow != RTT::NoData) && (jac_full_Flow != RTT::NoData) && (jacDot_full_Flow != RTT::NoData)) {
        if ((jac_Flow == RTT::NewData) || (jacDot_Flow == RTT::NewData) || (inertia_Flow == RTT::NewData) || (jac_full_Flow == RTT::NewData) || (jacDot_full_Flow == RTT::NewData)) {

            calculateAuxiliaries(jac, jacDot, M, jac_full, jacDot_full);

            // publish items!
            jac_cstr_Port.write(jac_cstr_);
            jac_Dot_cstr_Port.write(jac_Dot_cstr_);

            jac_cstr_mpi_Port.write(jac_cstr_MPI);
            jac_mpi_Port.write(jac_MPI);

            p_Port.write(P);

            lambda_cstr_Port.write(Lamda_cstr);

            inertia_cstr_Port.write(M_cstr_);

            c_cstr_Port.write(C_cstr_);

        }

    }
}

bool ConstrainedAuxiliaries::startHook() {
    return true;
}

bool ConstrainedAuxiliaries::configureHook() {
	jac_Flow = RTT::NoData;
    jac_full_Flow = RTT::NoData;
    jacDot_Flow = RTT::NoData;
    jacDot_full_Flow = RTT::NoData;
	inertia_Flow = RTT::NoData;
	return true;
}

void ConstrainedAuxiliaries::calculateAuxiliaries(const Eigen::MatrixXf& jac_task, const Eigen::MatrixXf& jac_Dot_task, const Eigen::MatrixXf& M_, const Eigen::MatrixXf& jac_full_, const Eigen::MatrixXf& jacDot_full_) {

    // TODO cstr jac needs to be moved and need to depend on the constrained matrix!
    jac_cstr_ = jac_full_;
    jac_Dot_cstr_ = jacDot_full_;

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

//    RTT::log(RTT::Warning) << "jac_\n" << jac_ << RTT::endlog();
//    RTT::log(RTT::Warning) << "jac_cstr_MPI\n" << jac_cstr_MPI << RTT::endlog();
//    RTT::log(RTT::Warning) << "jac_cstr_\n" << jac_cstr_ << RTT::endlog();

    //Eq. under Eq. 11
    M_cstr_ = P * M_ + identityDOFsizeDOFsize - P;

//    RTT::log(RTT::Warning) << "M_cstr_\n" << M_cstr_ << RTT::endlog();

    //Eq. under Eq. 11
    C_cstr_ = -(jac_cstr_MPI * jac_Dot_cstr_);

//    RTT::log(RTT::Warning) << "C_cstr_\n" << C_cstr_ << RTT::endlog();

    //Eq. under Eq. 11
    Lamda_cstr = (jac_task * M_cstr_.inverse() * P * jac_task.transpose() + tmpeyeTSdimTSdim).inverse();

//    RTT::log(RTT::Warning) << "Lamda_cstr\n" << Lamda_cstr << RTT::endlog();

    //Eq. 14
    //jac_MPI = (jac_task * M_cstr_.inverse() * P * jac_task.transpose() + tmpeyeTSdimTSdim).inverse() * jac_task * M_cstr_.inverse() * P;
    jac_MPI = Lamda_cstr * jac_task * M_cstr_.inverse() * P;

//    RTT::log(RTT::Warning) << "jac_MPI\n" << jac_MPI << RTT::endlog();
}

ORO_LIST_COMPONENT_TYPE(cosima::ConstrainedAuxiliaries)

