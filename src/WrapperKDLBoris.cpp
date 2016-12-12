/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#include "WrapperKDLBoris.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


WrapperKDLBoris::WrapperKDLBoris(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &WrapperKDLBoris::setDOFsize, this).doc("set DOF size");
    addOperation("loadModel", &WrapperKDLBoris::loadModel, this).doc("load model");
//    addOperation("computeKDLinformation", &WrapperKDLBoris::computeKDLinformation, this).doc("computeKDLinformation");
    addOperation("displayCurrentState", &WrapperKDLBoris::displayCurrentState, this).doc("print current state");

    //other stuff
    portsArePrepared = false;
    identity33 = Eigen::MatrixXf::Identity(3,3);
    identity66 = Eigen::MatrixXf::Identity(6,6);
}

bool WrapperKDLBoris::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion of input port, the output ports must not necessarily be connected
//    if (!in_robotstatus_port.connected())
//        return false;
//    else
        return true;
}

bool WrapperKDLBoris::startHook() {
    // this method starts the component
    return true;
}

void WrapperKDLBoris::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if (in_robotstatus_flow == RTT::NewData || in_robotstatus_flow == RTT::OldData){
        assert(in_robotstatus_var.angles.rows()==DOFsize);
        assert(in_robotstatus_var.angles.cols()==1);
        assert(in_robotstatus_var.velocities.rows()==DOFsize);
        assert(in_robotstatus_var.velocities.cols()==1);
        assert(in_robotstatus_var.torques.rows()==DOFsize);
        assert(in_robotstatus_var.torques.cols()==1);

        this->computeKDLinformation(
                    in_robotstatus_var,
                    out_inertia_var,
                    out_gravity_var,
                    out_coriolis_var,
                    out_cartPos_var,
                    out_cartVel_var,
                    out_jacobian_var,
                    out_jacobianDot_var);

        out_robotstatus_var.angles = in_robotstatus_var.angles;
        out_robotstatus_var.velocities = in_robotstatus_var.velocities;
        out_robotstatus_var.torques = in_robotstatus_var.torques;

        out_coriolisAndGravity_var = out_gravity_var + out_coriolis_var;

        out_cartAcc_var = out_jacobianDot_var * in_robotstatus_var.velocities; //TODO: add out_jacobian_var * in_robotstatus_var.accelerations

        out_cartPosTranslation_var = out_cartPos_var.head<6>();
        out_cartVelTranslation_var = out_cartVel_var.head<6>();
        out_cartAccTranslation_var = out_cartAcc_var.head<6>();

        out_cartPosOrientation_var = out_cartPos_var.tail<6>();
        out_cartVelOrientation_var = out_cartVel_var.tail<6>();
        out_cartAccOrientation_var = out_cartAcc_var.tail<6>();

        out_jacobianTranslation_var = out_jacobian_var.topRows<6>();
        out_jacobianDotTranslation_var = out_jacobianDot_var.topRows<6>();

        out_jacobianOrientation_var = out_jacobian_var.bottomRows<6>();
        out_jacobianDotOrientation_var = out_jacobianDot_var.bottomRows<6>();

    } else if (in_robotstatus_flow == RTT::NoData){
        out_robotstatus_var.angles.setZero();
        out_robotstatus_var.velocities.setZero();
        out_robotstatus_var.torques.setZero();
        out_inertia_var.setZero();
        out_gravity_var.setZero();
        out_coriolis_var.setZero();
        out_coriolisAndGravity_var.setZero();
        out_cartPos_var.setZero();
        out_cartVel_var.setZero();
        out_cartAcc_var.setZero();
        out_cartPosTranslation_var.setZero();
        out_cartVelTranslation_var.setZero();
        out_cartAccTranslation_var.setZero();
        out_cartPosOrientation_var.setZero();
        out_cartVelOrientation_var.setZero();
        out_cartAccOrientation_var.setZero();
        out_jacobian_var.setZero();
        out_jacobianDot_var.setZero();
        out_jacobianTranslation_var.setZero();
        out_jacobianDotTranslation_var.setZero();
        out_jacobianOrientation_var.setZero();
        out_jacobianDotOrientation_var.setZero();
    } else {
        // there should be something really wrong!
    }

    out_robotstatus_port.write(out_robotstatus_var);
    out_inertia_port.write(out_inertia_var);
    out_gravity_port.write(out_gravity_var);
    out_coriolis_port.write(out_coriolis_var);
    out_coriolisAndGravity_port.write(out_coriolisAndGravity_var);
    out_cartPos_port.write(out_cartPos_var);
    out_cartVel_port.write(out_cartVel_var);
    out_cartAcc_port.write(out_cartAcc_var);
    out_cartPosTranslation_port.write(out_cartPosTranslation_var);
    out_cartVelTranslation_port.write(out_cartVelTranslation_var);
    out_cartAccTranslation_port.write(out_cartAccTranslation_var);
    out_cartPosOrientation_port.write(out_cartPosOrientation_var);
    out_cartVelOrientation_port.write(out_cartVelOrientation_var);
    out_cartAccOrientation_port.write(out_cartAccOrientation_var);
    out_jacobian_port.write(out_jacobian_var);
    out_jacobianDot_port.write(out_jacobianDot_var);
    out_jacobianTranslation_port.write(out_jacobianTranslation_var);
    out_jacobianDotTranslation_port.write(out_jacobianDotTranslation_var);
    out_jacobianOrientation_port.write(out_jacobianOrientation_var);
    out_jacobianDotOrientation_port.write(out_jacobianDotOrientation_var);
}

void WrapperKDLBoris::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void WrapperKDLBoris::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void WrapperKDLBoris::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    if (kdl_chain1_.getNrOfJoints() + kdl_chain2_.getNrOfJoints() != DOFsize){
        std::cerr << "kdl found " << kdl_chain1_.getNrOfJoints() + kdl_chain2_.getNrOfJoints() << " joints ..." << std::endl;
    }
    assert(kdl_chain1_.getNrOfJoints() + kdl_chain2_.getNrOfJoints() == DOFsize);
    this->DOFsize = DOFsize;
    jntPosConfigPlusJntVelConfig_q1 = KDL::JntArrayVel(kdl_chain1_.getNrOfJoints());
    jntPosConfigPlusJntVelConfig_q2 = KDL::JntArrayVel(kdl_chain2_.getNrOfJoints());
    G_ = KDL::JntArray(DOFsize);
    M_ = KDL::JntSpaceInertiaMatrix(DOFsize);
    C_ = KDL::JntArray(DOFsize);
    G1_ = KDL::JntArray(kdl_chain1_.getNrOfJoints());
    M1_ = KDL::JntSpaceInertiaMatrix(kdl_chain1_.getNrOfJoints());
    C1_ = KDL::JntArray(kdl_chain1_.getNrOfJoints());
    G2_ = KDL::JntArray(kdl_chain2_.getNrOfJoints());
    M2_ = KDL::JntSpaceInertiaMatrix(kdl_chain2_.getNrOfJoints());
    C2_ = KDL::JntArray(kdl_chain2_.getNrOfJoints());
    jac1_ = KDL::Jacobian(kdl_chain1_.getNrOfJoints());
    jac_dot1_ = KDL::Jacobian(kdl_chain1_.getNrOfJoints());
    jac2_ = KDL::Jacobian(kdl_chain2_.getNrOfJoints());
    jac_dot2_ = KDL::Jacobian(kdl_chain2_.getNrOfJoints());
    cartPosFrame1 = KDL::Frame();
    cartVelFrame1 = KDL::FrameVel();
    cartPosFrame2 = KDL::Frame();
    cartVelFrame2 = KDL::FrameVel();

    jacobian1 = Eigen::MatrixXf::Zero(6,kdl_chain1_.getNrOfJoints());
    jacobian2 = Eigen::MatrixXf::Zero(6,kdl_chain2_.getNrOfJoints());
    jacobianDot1 = Eigen::MatrixXf::Zero(6,kdl_chain1_.getNrOfJoints());
    jacobianDot2 = Eigen::MatrixXf::Zero(6,kdl_chain2_.getNrOfJoints());

    this->preparePorts();
}


void WrapperKDLBoris::loadModel(std::string modelname, std::string chain_root_link_name, std::string chain_tip1_link_name, std::string chain_tip2_link_name){
    assert(modelname.length() > 0);
    assert(chain_root_link_name.length() > 0);
    assert(chain_tip1_link_name.length() > 0);
    assert(chain_tip2_link_name.length() > 0);
    assert(exists_test(modelname) == true);

    std::cout << " loadModel #######################" << std::endl;

    //init KDL tree and KDL chain
    //    robot_model = urdf::Model();
    if (!robot_model.initFile(modelname.c_str())) {
        std::cerr << "Could not generate robot model" << std::endl;
    }
    //    kdl_tree = KDL::Tree();
    if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
        std::cerr << "Could not extract kdl tree" << std::endl;
    }
    assert(kdl_tree.getNrOfJoints() > 0);
    //    kdl_chain1_ = KDL::Chain();
    //    kdl_chain2_ = KDL::Chain();
    kdl_tree.getChain(chain_root_link_name, chain_tip1_link_name, kdl_chain1_);
    kdl_tree.getChain(chain_root_link_name, chain_tip2_link_name, kdl_chain2_);
    assert(kdl_chain1_.getNrOfJoints() > 0);
    assert(kdl_chain2_.getNrOfJoints() > 0);

    std::cout << " tree NrOfJoints " << kdl_tree.getNrOfJoints() << std::endl;
    std::cout << " tree getNrOfSegments " << kdl_tree.getNrOfSegments() << std::endl;
    std::cout << " chain1 NrOfJoints " << kdl_chain1_.getNrOfJoints() << std::endl;
    std::cout << " chain1 getNrOfSegments " << kdl_chain1_.getNrOfSegments() << std::endl;
    std::cout << " chain2 NrOfJoints " << kdl_chain2_.getNrOfJoints() << std::endl;
    std::cout << " chain2 getNrOfSegments " << kdl_chain2_.getNrOfSegments() << std::endl;

    // initialize KDL solver
    gravity_vector = KDL::Vector();
    gravity_vector.data[0] = 0.0;
    gravity_vector.data[1] = 0.0;
    gravity_vector.data[2] =-9.81;
    id_dyn_solver1.reset(new KDL::ChainDynParam(kdl_chain1_, gravity_vector));
    jnt_to_jac_solver1.reset(new KDL::ChainJntToJacSolver(kdl_chain1_));
    jnt_to_jac_dot_solver1.reset(new KDL::ChainJntToJacDotSolver(kdl_chain1_));
    jnt_to_cart_pos_solver1.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain1_));
    jnt_to_cart_vel_solver1.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain1_));
    id_dyn_solver2.reset(new KDL::ChainDynParam(kdl_chain2_, gravity_vector));
    jnt_to_jac_solver2.reset(new KDL::ChainJntToJacSolver(kdl_chain2_));
    jnt_to_jac_dot_solver2.reset(new KDL::ChainJntToJacDotSolver(kdl_chain2_));
    jnt_to_cart_pos_solver2.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain2_));
    jnt_to_cart_vel_solver2.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain2_));

    std::cout << "  #######################" << std::endl;
}


void WrapperKDLBoris::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_robotstatus_port");
        ports()->removePort("out_inertia_port");
        ports()->removePort("out_gravity_port");
        ports()->removePort("out_coriolis_port");
        ports()->removePort("out_coriolisAndGravity_port");
        ports()->removePort("out_cartPos_port");
        ports()->removePort("out_cartVel_port");
        ports()->removePort("out_cartAcc_port");
        ports()->removePort("out_cartPosTranslation_port");
        ports()->removePort("out_cartVelTranslation_port");
        ports()->removePort("out_cartAccTranslation_port");
        ports()->removePort("out_cartPosOrientation_port");
        ports()->removePort("out_cartVelOrientation_port");
        ports()->removePort("out_cartAccOrientation_port");
        ports()->removePort("out_jacobian_port");
        ports()->removePort("out_jacobianDot_port");
        ports()->removePort("out_jacobianTranslation_port");
        ports()->removePort("out_jacobianDotTranslation_port");
        ports()->removePort("out_jacobianOrientation_port");
        ports()->removePort("out_jacobianDotOrientation_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output
    out_robotstatus_var = rstrt::robot::JointState(DOFsize);
    out_robotstatus_port.setName("out_robotstatus_port");
    out_robotstatus_port.doc("Output port for robotstatus values");
    out_robotstatus_port.setDataSample(out_robotstatus_var);
    ports()->addPort(out_robotstatus_port);

    out_inertia_var = Eigen::MatrixXf(DOFsize,DOFsize);
    out_inertia_var.setZero();
    out_inertia_port.setName("out_inertia_port");
    out_inertia_port.doc("Output port for inertia matrix");
    out_inertia_port.setDataSample(out_inertia_var);
    ports()->addPort(out_inertia_port);

    out_gravity_var = Eigen::VectorXf(DOFsize);
    out_gravity_var.setZero();
    out_gravity_port.setName("out_gravity_port");
    out_gravity_port.doc("Output port for gravity vector");
    out_gravity_port.setDataSample(out_gravity_var);
    ports()->addPort(out_gravity_port);

    out_coriolis_var = Eigen::VectorXf(DOFsize);
    out_coriolis_var.setZero();
    out_coriolis_port.setName("out_coriolis_port");
    out_coriolis_port.doc("Output port for coriolis vector");
    out_coriolis_port.setDataSample(out_coriolis_var);
    ports()->addPort(out_coriolis_port);

    out_coriolisAndGravity_var = Eigen::VectorXf(DOFsize);
    out_coriolisAndGravity_var.setZero();
    out_coriolisAndGravity_port.setName("out_coriolisAndGravity_port");
    out_coriolisAndGravity_port.doc("Output port for combined coriolis and gravity vector");
    out_coriolisAndGravity_port.setDataSample(out_coriolisAndGravity_var);
    ports()->addPort(out_coriolisAndGravity_port);

    out_cartPos_var = Eigen::VectorXf(12);
    out_cartPos_var.setZero();
    out_cartPos_port.setName("out_cartPos_port");
    out_cartPos_port.doc("Output port for cartesian position vector");
    out_cartPos_port.setDataSample(out_cartPos_var);
    ports()->addPort(out_cartPos_port);

    out_cartVel_var = Eigen::VectorXf(12);
    out_cartVel_var.setZero();
    out_cartVel_port.setName("out_cartVel_port");
    out_cartVel_port.doc("Output port for cartesian velocity vector");
    out_cartVel_port.setDataSample(out_cartVel_var);
    ports()->addPort(out_cartVel_port);

    out_cartAcc_var = Eigen::VectorXf(12);
    out_cartAcc_var.setZero();
    out_cartAcc_port.setName("out_cartAcc_port");
    out_cartAcc_port.doc("Output port for cartesian acceleration vector");
    out_cartAcc_port.setDataSample(out_cartAcc_var);
    ports()->addPort(out_cartAcc_port);

    out_cartPosTranslation_var = Eigen::VectorXf(6);
    out_cartPosTranslation_var.setZero();
    out_cartPosTranslation_port.setName("out_cartPosTranslation_port");
    out_cartPosTranslation_port.doc("Output port for cartesian position Translation vector");
    out_cartPosTranslation_port.setDataSample(out_cartPosTranslation_var);
    ports()->addPort(out_cartPosTranslation_port);

    out_cartVelTranslation_var = Eigen::VectorXf(6);
    out_cartVelTranslation_var.setZero();
    out_cartVelTranslation_port.setName("out_cartVelTranslation_port");
    out_cartVelTranslation_port.doc("Output port for cartesian velocity Translation vector");
    out_cartVelTranslation_port.setDataSample(out_cartVelTranslation_var);
    ports()->addPort(out_cartVelTranslation_port);

    out_cartAccTranslation_var = Eigen::VectorXf(6);
    out_cartAccTranslation_var.setZero();
    out_cartAccTranslation_port.setName("out_cartAccTranslation_port");
    out_cartAccTranslation_port.doc("Output port for cartesian acceleration Translation vector");
    out_cartAccTranslation_port.setDataSample(out_cartAccTranslation_var);
    ports()->addPort(out_cartAccTranslation_port);

    out_cartPosOrientation_var = Eigen::VectorXf(6);
    out_cartPosOrientation_var.setZero();
    out_cartPosOrientation_port.setName("out_cartPosOrientation_port");
    out_cartPosOrientation_port.doc("Output port for cartesian position Orientation vector");
    out_cartPosOrientation_port.setDataSample(out_cartPosOrientation_var);
    ports()->addPort(out_cartPosOrientation_port);

    out_cartVelOrientation_var = Eigen::VectorXf(6);
    out_cartVelOrientation_var.setZero();
    out_cartVelOrientation_port.setName("out_cartVelOrientation_port");
    out_cartVelOrientation_port.doc("Output port for cartesian velocity Orientation vector");
    out_cartVelOrientation_port.setDataSample(out_cartVelOrientation_var);
    ports()->addPort(out_cartVelOrientation_port);

    out_cartAccOrientation_var = Eigen::VectorXf(6);
    out_cartAccOrientation_var.setZero();
    out_cartAccOrientation_port.setName("out_cartAccOrientation_port");
    out_cartAccOrientation_port.doc("Output port for cartesian acceleration Orientation vector");
    out_cartAccOrientation_port.setDataSample(out_cartAccOrientation_var);
    ports()->addPort(out_cartAccOrientation_port);

    out_jacobian_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobian_var.setZero();
    out_jacobian_port.setName("out_jacobian_port");
    out_jacobian_port.doc("Output port for jacobian matrix");
    out_jacobian_port.setDataSample(out_jacobian_var);
    ports()->addPort(out_jacobian_port);

    out_jacobianDot_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobianDot_var.setZero();
    out_jacobianDot_port.setName("out_jacobianDot_port");
    out_jacobianDot_port.doc("Output port for jacobianDot matrix");
    out_jacobianDot_port.setDataSample(out_jacobianDot_var);
    ports()->addPort(out_jacobianDot_port);

    out_jacobianTranslation_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianTranslation_var.setZero();
    out_jacobianTranslation_port.setName("out_jacobianTranslation_port");
    out_jacobianTranslation_port.doc("Output port for jacobianTranslation matrix");
    out_jacobianTranslation_port.setDataSample(out_jacobianTranslation_var);
    ports()->addPort(out_jacobianTranslation_port);

    out_jacobianDotTranslation_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianDotTranslation_var.setZero();
    out_jacobianDotTranslation_port.setName("out_jacobianDotTranslation_port");
    out_jacobianDotTranslation_port.doc("Output port for jacobianDotTranslation matrix");
    out_jacobianDotTranslation_port.setDataSample(out_jacobianDotTranslation_var);
    ports()->addPort(out_jacobianDotTranslation_port);

    out_jacobianOrientation_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianOrientation_var.setZero();
    out_jacobianOrientation_port.setName("out_jacobianOrientation_port");
    out_jacobianOrientation_port.doc("Output port for jacobianOrientation matrix");
    out_jacobianOrientation_port.setDataSample(out_jacobianOrientation_var);
    ports()->addPort(out_jacobianOrientation_port);

    out_jacobianDotOrientation_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianDotOrientation_var.setZero();
    out_jacobianDotOrientation_port.setName("out_jacobianDotOrientation_port");
    out_jacobianDotOrientation_port.doc("Output port for jacobianDotOrientation matrix");
    out_jacobianDotOrientation_port.setDataSample(out_jacobianDotOrientation_var);
    ports()->addPort(out_jacobianDotOrientation_port);

    portsArePrepared = true;
}

void WrapperKDLBoris::computeKDLinformation(
        rstrt::robot::JointState & robotstatus,
        Eigen::MatrixXf & inertia,
        Eigen::VectorXf & gravity,
        Eigen::VectorXf & coriolis,
        Eigen::VectorXf & cartPos,
        Eigen::VectorXf & cartVel,
        Eigen::MatrixXf & jacobian,
        Eigen::MatrixXf & jacobianDot) {

    // initialize KDL fields
    jntPosConfigPlusJntVelConfig_q1.q.data.setZero();
    jntPosConfigPlusJntVelConfig_q1.qdot.data.setZero();
    jntPosConfigPlusJntVelConfig_q2.q.data.setZero();
    jntPosConfigPlusJntVelConfig_q2.qdot.data.setZero();
    G_.data.setZero();
    M_.data.setZero();
    C_.data.setZero();
    G1_.data.setZero();
    M1_.data.setZero();
    C1_.data.setZero();
    G2_.data.setZero();
    M2_.data.setZero();
    C2_.data.setZero();
    jac1_.data.setZero();
    jac_dot1_.data.setZero();
    jac2_.data.setZero();
    jac_dot2_.data.setZero();
//    cartFrame1.setZero();
//    velFrame1.setZero();
//    cartFrame2.setZero();
//    velFrame2.setZero();

    //set current joint positions and velocities
    /* initialize strange stuff for solvers */
    this->castEigenVectorFtoD(robotstatus.angles.head(kdl_chain1_.getNrOfJoints()), jntPosConfigPlusJntVelConfig_q1.q.data);
    this->castEigenVectorFtoD(robotstatus.velocities.head(kdl_chain1_.getNrOfJoints()), jntPosConfigPlusJntVelConfig_q1.qdot.data);
    this->castEigenVectorFtoD(robotstatus.angles.tail(kdl_chain2_.getNrOfJoints()), jntPosConfigPlusJntVelConfig_q2.q.data);
    this->castEigenVectorFtoD(robotstatus.velocities.tail(kdl_chain2_.getNrOfJoints()), jntPosConfigPlusJntVelConfig_q2.qdot.data);

    /* execute solvers for inv.Dynamics */
    // calculate matrices G(gravitation), M(inertia) and C(coriolis)
    id_dyn_solver1->JntToGravity(jntPosConfigPlusJntVelConfig_q1.q, G1_);
    id_dyn_solver1->JntToMass(jntPosConfigPlusJntVelConfig_q1.q, M1_);
    id_dyn_solver1->JntToCoriolis(jntPosConfigPlusJntVelConfig_q1.q, jntPosConfigPlusJntVelConfig_q1.qdot, C1_);
    id_dyn_solver2->JntToGravity(jntPosConfigPlusJntVelConfig_q2.q, G2_);
    id_dyn_solver2->JntToMass(jntPosConfigPlusJntVelConfig_q2.q, M2_);
    id_dyn_solver2->JntToCoriolis(jntPosConfigPlusJntVelConfig_q2.q, jntPosConfigPlusJntVelConfig_q2.qdot, C2_);

    /* execute solver for Jacobian based on velocities */
    //calculate jacobian and jacobian dot
    jnt_to_jac_solver1->JntToJac(jntPosConfigPlusJntVelConfig_q1.q, jac1_, kdl_chain1_.getNrOfSegments());
    jnt_to_jac_dot_solver1->JntToJacDot(jntPosConfigPlusJntVelConfig_q1, jac_dot1_, kdl_chain1_.getNrOfSegments());
    jnt_to_jac_solver2->JntToJac(jntPosConfigPlusJntVelConfig_q2.q, jac2_, kdl_chain2_.getNrOfSegments());
    jnt_to_jac_dot_solver2->JntToJacDot(jntPosConfigPlusJntVelConfig_q2, jac_dot2_, kdl_chain2_.getNrOfSegments());

    // forward kinematics: calculate cartesian position based on joint angles
    jnt_to_cart_pos_solver1->JntToCart(jntPosConfigPlusJntVelConfig_q1.q, cartPosFrame1, kdl_chain1_.getNrOfSegments());
    jnt_to_cart_vel_solver1->JntToCart(jntPosConfigPlusJntVelConfig_q1, cartVelFrame1, kdl_chain1_.getNrOfSegments());
    jnt_to_cart_pos_solver2->JntToCart(jntPosConfigPlusJntVelConfig_q2.q, cartPosFrame2, kdl_chain2_.getNrOfSegments());
    jnt_to_cart_vel_solver2->JntToCart(jntPosConfigPlusJntVelConfig_q2, cartVelFrame2, kdl_chain2_.getNrOfSegments());

    M_.data.topLeftCorner(kdl_chain1_.getNrOfJoints(),kdl_chain1_.getNrOfJoints()) = M1_.data;
    M_.data.bottomRightCorner(kdl_chain2_.getNrOfJoints(),kdl_chain2_.getNrOfJoints()) = M2_.data;
    G_.data.head(kdl_chain1_.getNrOfJoints()) = G1_.data;
    G_.data.tail(kdl_chain2_.getNrOfJoints()) = G2_.data;
    C_.data.head(kdl_chain1_.getNrOfJoints()) = C1_.data;
    C_.data.tail(kdl_chain2_.getNrOfJoints()) = C2_.data;

    this->castEigenMatrixDtoF(M_.data, inertia);
    this->castEigenVectorDtoF(G_.data, gravity);
    this->castEigenVectorDtoF(C_.data, coriolis);
    this->castEigenMatrixDtoF(jac1_.data, jacobian1);
    this->castEigenMatrixDtoF(jac_dot1_.data, jacobianDot1);
    this->castEigenMatrixDtoF(jac2_.data, jacobian2);
    this->castEigenMatrixDtoF(jac_dot2_.data, jacobianDot2);

    jacobian.setZero();
    jacobian.block<3,7>(0,0) = jacobian1.topRows<3>();
    jacobian.block<3,7>(3,7) = jacobian2.topRows<3>();
    jacobian.block<3,7>(6,0) = jacobian1.bottomRows<3>();
    jacobian.block<3,7>(9,7) = jacobian2.bottomRows<3>();

    jacobianDot.setZero();
    jacobianDot.block<3,7>(0,0) = jacobianDot1.topRows<3>();
    jacobianDot.block<3,7>(3,7) = jacobianDot2.topRows<3>();
    jacobianDot.block<3,7>(6,0) = jacobianDot1.bottomRows<3>();
    jacobianDot.block<3,7>(9,7) = jacobianDot2.bottomRows<3>();

//    jacobian.setZero();
//    jacobian.topLeftCorner(6,kdl_chain1_.getNrOfJoints()) = jacobian1;
//    jacobian.bottomRightCorner(6,kdl_chain2_.getNrOfJoints()) = jacobian2;
//    jacobianDot.setZero();
//    jacobianDot.topLeftCorner(6,kdl_chain1_.getNrOfJoints()) = jacobianDot1;
//    jacobianDot.bottomRightCorner(6,kdl_chain2_.getNrOfJoints()) = jacobianDot2;

    cartPos(0) = cartPosFrame1.p.x();
    cartPos(1) = cartPosFrame1.p.y();
    cartPos(2) = cartPosFrame1.p.z();
    cartPos(3) = cartPosFrame2.p.x();
    cartPos(4) = cartPosFrame2.p.y();
    cartPos(5) = cartPosFrame2.p.z();

    cartPos(6) = cartPosFrame1.M.GetRot().x();
    cartPos(7) = cartPosFrame1.M.GetRot().y();
    cartPos(8) = cartPosFrame1.M.GetRot().z();
    cartPos(9) = cartPosFrame2.M.GetRot().x();
    cartPos(10)= cartPosFrame2.M.GetRot().y();
    cartPos(11)= cartPosFrame2.M.GetRot().z();

    cartVel(0) = cartVelFrame1.GetTwist().vel.x(); //similar to velFrame.p.v.x();
    cartVel(1) = cartVelFrame1.GetTwist().vel.y(); //similar to velFrame.p.v.y();
    cartVel(2) = cartVelFrame1.GetTwist().vel.z(); //similar to velFrame.p.v.z();
    cartVel(3) = cartVelFrame2.GetTwist().vel.x(); //similar to velFrame.p.v.x();
    cartVel(4) = cartVelFrame2.GetTwist().vel.y(); //similar to velFrame.p.v.y();
    cartVel(5) = cartVelFrame2.GetTwist().vel.z(); //similar to velFrame.p.v.z();

    cartVel(6) = cartVelFrame1.GetTwist().rot.x();
    cartVel(7) = cartVelFrame1.GetTwist().rot.y();
    cartVel(8) = cartVelFrame1.GetTwist().rot.z();
    cartVel(9) = cartVelFrame2.GetTwist().rot.x();
    cartVel(10)= cartVelFrame2.GetTwist().rot.y();
    cartVel(11)= cartVelFrame2.GetTwist().rot.z();

    //TODO: compare with: cartVel = jacobian * robotstatus.angles
}


void WrapperKDLBoris::displayCurrentState() {
    std::cout << "############## WrapperKDLBoris State begin " << std::endl;
    std::cout << " tree NrOfJoints " << kdl_tree.getNrOfJoints() << std::endl;
    std::cout << " tree getNrOfSegments " << kdl_tree.getNrOfSegments() << std::endl;
    std::cout << " chain1 NrOfJoints " << kdl_chain1_.getNrOfJoints() << std::endl;
    std::cout << " chain1 getNrOfSegments " << kdl_chain1_.getNrOfSegments() << std::endl;
    std::cout << " chain2 NrOfJoints " << kdl_chain2_.getNrOfJoints() << std::endl;
    std::cout << " chain2 getNrOfSegments " << kdl_chain2_.getNrOfSegments() << std::endl;

    std::cout << " angles \n" << in_robotstatus_var.angles << std::endl;
    std::cout << " velocities \n" << in_robotstatus_var.velocities << std::endl;
    std::cout << " inertia \n" << out_inertia_var << std::endl;
    std::cout << " gravity \n" << out_gravity_var << std::endl;
    std::cout << " coriolis \n" << out_coriolis_var << std::endl;
    std::cout << " coriolisAndGravity \n" << out_coriolisAndGravity_var << std::endl;
    std::cout << " cartPos \n" << out_cartPos_var << std::endl;
    std::cout << " cartVel \n" << out_cartVel_var << std::endl;
    std::cout << " cartAcc \n" << out_cartAcc_var << std::endl;
    std::cout << " jacobian \n" << out_jacobian_var << std::endl;
    std::cout << " jacobianDot \n" << out_jacobianDot_var << std::endl;
    std::cout << "############## WrapperKDLBoris State end " << std::endl;
}


bool WrapperKDLBoris::exists_test(const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}


void WrapperKDLBoris::castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f) {
    f = d.cast <float> ();
}

void WrapperKDLBoris::castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d) {
    d = f.cast <double> ();
}

void WrapperKDLBoris::castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f) {
    f = d.cast <float> ();
}

void WrapperKDLBoris::castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d) {
    d = f.cast <double> ();
}

//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(WrapperKDLBoris)
