/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description:
 */

#include "TaskDescriberDualArm.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TaskDescriberDualArm::TaskDescriberDualArm(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setDOFsize", &TaskDescriberDualArm::setDOFsize, this).doc("set DOF size");
    addOperation("setContactTime", &TaskDescriberDualArm::setContactTime, this).doc("set ContactTime");
    addOperation("loadModel", &TaskDescriberDualArm::loadModel, this).doc("load model");
    addOperation("displayCurrentState", &TaskDescriberDualArm::displayCurrentState, this).doc("print current state");

    //other stuff
    contact_time = 0.0;
    currentMode=false;
    portsArePrepared = false;
}

bool TaskDescriberDualArm::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run

    //check conncetion of input port, the output ports must not necessarily be connected
//    if (!in_robotstatus_port.connected())
//        return false;
//    else
        return true;
}

bool TaskDescriberDualArm::startHook() {
    // this method starts the component

    std::string ConstrainedAuxiliaries_string = "caux";
    std::string PositionController_string = "positioncontroller";
    std::string TorqueSuperimposer_string = "torquesuperimposer";

    ConstrainedAuxiliaries_ptr = getPeer(ConstrainedAuxiliaries_string);
    PositionController_ptr = getPeer(PositionController_string);
    TorqueSuperimposer_ptr = getPeer(TorqueSuperimposer_string);

    setMode_ConstrainedAuxiliaries = ConstrainedAuxiliaries_ptr->getOperation("setConstrainedVersionMode");
    setMode_PositionController = PositionController_ptr->getOperation("setConstrainedVersionMode");
    setMode_TorqueSuperimposer = TorqueSuperimposer_ptr->getOperation("setConstrainedVersionMode");

    std::string NullspaceController_string = "nullspacecontroller";
    NullspaceController_ptr = getPeer(NullspaceController_string);
    setGains_NullspaceController = NullspaceController_ptr->getOperation("setGains");
    return true;
}

void TaskDescriberDualArm::updateHook() {
    // this is the actual body of a component. it is called on each cycle

    current_time = this->getSimulationTime();
    time_diff = this->current_time - this->contact_time;
//    if(time_diff > -2.0 && currentMode==false){
//        this->setGains_NullspaceController(0,0);
//    }
//    if(time_diff > 0.0 && currentMode==false){
//        currentMode = true;
//        this->setMode_ConstrainedAuxiliaries(true);
//        this->setMode_PositionController(true);
//        this->setMode_TorqueSuperimposer(true);
//    }

    if (in_robotstatus_port.connected()) {
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
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
    if ( (in_robotstatus_flow == RTT::NewData || in_robotstatus_flow == RTT::OldData) &&
         (in_jacobian_flow == RTT::NewData || in_jacobian_flow == RTT::OldData) &&
         (in_jacobianDot_flow == RTT::NewData || in_jacobianDot_flow == RTT::OldData) ){

        assert(in_robotstatus_var.angles.rows()==DOFsize);
        assert(in_robotstatus_var.angles.cols()==1);
        assert(in_robotstatus_var.velocities.rows()==DOFsize);
        assert(in_robotstatus_var.velocities.cols()==1);
        assert(in_robotstatus_var.torques.rows()==DOFsize);
        assert(in_robotstatus_var.torques.cols()==1);

        assert(in_jacobian_var.rows()==12);
        assert(in_jacobian_var.cols()==DOFsize);

        assert(in_jacobianDot_var.rows()==12);
        assert(in_jacobianDot_var.cols()==DOFsize);

        // specify jacobian full
        out_jacobianFull_var = in_jacobian_var;
        out_jacobianDotFull_var = in_jacobianDot_var;

        
        // specify jacobian task
        out_jacobianTask_var = in_jacobian_var;
        out_jacobianDotTask_var = in_jacobianDot_var;


        // specify jacobian constrained
//        out_jacobianCstr_var = in_jacobian_var;
//        out_jacobianDotCstr_var = in_jacobianDot_var;

        jntPosConfigPlusJntVelConfig_q.q.data.setZero();
        jntPosConfigPlusJntVelConfig_q.qdot.data.setZero();
        this->castEigenVectorFtoD(in_robotstatus_var.angles, jntPosConfigPlusJntVelConfig_q.q.data);
        this->castEigenVectorFtoD(in_robotstatus_var.velocities, jntPosConfigPlusJntVelConfig_q.qdot.data);

        jntPosConfigPlusJntVelConfig_q.q.data(0) = in_robotstatus_var.angles(6);//reverse order of left arm joint values
        jntPosConfigPlusJntVelConfig_q.q.data(1) = in_robotstatus_var.angles(5);
        jntPosConfigPlusJntVelConfig_q.q.data(2) = in_robotstatus_var.angles(4);
        jntPosConfigPlusJntVelConfig_q.q.data(3) = in_robotstatus_var.angles(3);
        jntPosConfigPlusJntVelConfig_q.q.data(4) = in_robotstatus_var.angles(2);
        jntPosConfigPlusJntVelConfig_q.q.data(5) = in_robotstatus_var.angles(1);
        jntPosConfigPlusJntVelConfig_q.q.data(6) = in_robotstatus_var.angles(0);

        jntPosConfigPlusJntVelConfig_q.qdot.data(0) = in_robotstatus_var.velocities(6);//reverse order of left arm joint values
        jntPosConfigPlusJntVelConfig_q.qdot.data(1) = in_robotstatus_var.velocities(5);
        jntPosConfigPlusJntVelConfig_q.qdot.data(2) = in_robotstatus_var.velocities(4);
        jntPosConfigPlusJntVelConfig_q.qdot.data(3) = in_robotstatus_var.velocities(3);
        jntPosConfigPlusJntVelConfig_q.qdot.data(4) = in_robotstatus_var.velocities(2);
        jntPosConfigPlusJntVelConfig_q.qdot.data(5) = in_robotstatus_var.velocities(1);
        jntPosConfigPlusJntVelConfig_q.qdot.data(6) = in_robotstatus_var.velocities(0);

        jac_.data.setZero();
        jac_dot_.data.setZero();
        jnt_to_jac_solver->JntToJac(jntPosConfigPlusJntVelConfig_q.q, jac_, kdl_chain_.getNrOfSegments());
        jnt_to_jac_dot_solver->JntToJacDot(jntPosConfigPlusJntVelConfig_q, jac_dot_, kdl_chain_.getNrOfSegments());

        this->castEigenMatrixDtoF(jac_.data, out_jacobianCstr_var);
        this->castEigenMatrixDtoF(jac_dot_.data, out_jacobianDotCstr_var);

        //compute constrained task space position and velocity
        jnt_to_cart_pos_solver->JntToCart(jntPosConfigPlusJntVelConfig_q.q, cartPosFrame, kdl_chain_.getNrOfSegments());
        jnt_to_cart_vel_solver->JntToCart(jntPosConfigPlusJntVelConfig_q, cartVelFrame, kdl_chain_.getNrOfSegments());

        out_cartPos_var(0) = cartPosFrame.p.x();
        out_cartPos_var(1) = cartPosFrame.p.y();
        out_cartPos_var(2) = cartPosFrame.p.z();
        out_cartPos_var(3) = cartPosFrame.M.GetRot().x();
        out_cartPos_var(4) = cartPosFrame.M.GetRot().y();
        out_cartPos_var(5) = cartPosFrame.M.GetRot().z();

        out_cartVel_var(0) = cartVelFrame.GetTwist().vel.x(); //similar to velFrame.p.v.x();
        out_cartVel_var(1) = cartVelFrame.GetTwist().vel.y(); //similar to velFrame.p.v.y();
        out_cartVel_var(2) = cartVelFrame.GetTwist().vel.z(); //similar to velFrame.p.v.z();
        out_cartVel_var(3) = cartVelFrame.GetTwist().rot.x();
        out_cartVel_var(4) = cartVelFrame.GetTwist().rot.y();
        out_cartVel_var(5) = cartVelFrame.GetTwist().rot.z();

    } else if ( (in_robotstatus_flow == RTT::NoData) || (in_jacobian_flow == RTT::NoData) || (in_jacobianDot_flow == RTT::NoData) ) {
        out_jacobianFull_var.setZero();
        out_jacobianDotFull_var.setZero();

        out_jacobianTask_var.setZero();
        out_jacobianDotTask_var.setZero();

        out_jacobianCstr_var.setZero();
        out_jacobianDotCstr_var.setZero();

        out_cartPos_var.setZero();
        out_cartVel_var.setZero();

    } else {
        // there should be something really wrong!
    }

    out_jacobianFull_port.write(out_jacobianFull_var);
    out_jacobianDotFull_port.write(out_jacobianDotFull_var);

    out_jacobianTask_port.write(out_jacobianTask_var);
    out_jacobianDotTask_port.write(out_jacobianDotTask_var);

    out_jacobianCstr_port.write(out_jacobianCstr_var);
    out_jacobianDotCstr_port.write(out_jacobianDotCstr_var);

    out_cartPos_port.write(out_cartPos_var);
    out_cartVel_port.write(out_cartVel_var);
}

void TaskDescriberDualArm::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TaskDescriberDualArm::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void TaskDescriberDualArm::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;

    jntPosConfigPlusJntVelConfig_q = KDL::JntArrayVel(kdl_chain_.getNrOfJoints());
    jac_ = KDL::Jacobian(kdl_chain_.getNrOfJoints());
    jac_dot_ = KDL::Jacobian(kdl_chain_.getNrOfJoints());

    this->preparePorts();
}

void TaskDescriberDualArm::setContactTime(float contactTime){
    this->contact_time = contactTime;
}

double TaskDescriberDualArm::getSimulationTime() {
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}

void TaskDescriberDualArm::loadModel(std::string modelname, std::string chain_root_link_name, std::string chain_tip_link_name){
    assert(modelname.length() > 0);
    assert(chain_root_link_name.length() > 0);
    assert(chain_tip_link_name.length() > 0);
    assert(exists_test(modelname) == true);

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
    //    kdl_chain_ = KDL::Chain();
    kdl_tree.getChain(chain_root_link_name, chain_tip_link_name, kdl_chain_);
    assert(kdl_chain_.getNrOfJoints() > 0);

    // initialize KDL solver
    jnt_to_jac_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    jnt_to_jac_dot_solver.reset(new KDL::ChainJntToJacDotSolver(kdl_chain_));

    std::cout << " tree NrOfJoints " << kdl_tree.getNrOfJoints() << std::endl;
    std::cout << " tree getNrOfSegments " << kdl_tree.getNrOfSegments() << std::endl;
    std::cout << " chain NrOfJoints " << kdl_chain_.getNrOfJoints() << std::endl;
    std::cout << " chain getNrOfSegments " << kdl_chain_.getNrOfSegments() << std::endl;

    cartPosFrame = KDL::Frame();
    cartVelFrame = KDL::FrameVel();
    jnt_to_cart_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_cart_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
}


void TaskDescriberDualArm::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("in_jacobian_port");
        ports()->removePort("in_jacobianDot_port");

        ports()->removePort("out_jacobianFull_port");
        ports()->removePort("out_jacobianDotFull_port");

        ports()->removePort("out_jacobianTask_port");
        ports()->removePort("out_jacobianDotTask_port");

        ports()->removePort("out_jacobianCstr_port");
        ports()->removePort("out_jacobianDotCstr_port");

        ports()->removePort("out_cartPos_port");
        ports()->removePort("out_cartVel_port");
    }

    //prepare input
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_jacobian_var = Eigen::MatrixXf(12,DOFsize);
    in_jacobian_var.setZero();
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for jacobian matrix");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    in_jacobianDot_var = Eigen::MatrixXf(12,DOFsize);
    in_jacobianDot_var.setZero();
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc("Input port for jacobianDot matrix");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;

    //prepare output
    out_jacobianFull_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobianFull_var.setZero();
    out_jacobianFull_port.setName("out_jacobianFull_port");
    out_jacobianFull_port.doc("Output port for jacobian matrix");
    out_jacobianFull_port.setDataSample(out_jacobianFull_var);
    ports()->addPort(out_jacobianFull_port);

    out_jacobianDotFull_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobianDotFull_var.setZero();
    out_jacobianDotFull_port.setName("out_jacobianDotFull_port");
    out_jacobianDotFull_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotFull_port.setDataSample(out_jacobianDotFull_var);
    ports()->addPort(out_jacobianDotFull_port);

    out_jacobianTask_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobianTask_var.setZero();
    out_jacobianTask_port.setName("out_jacobianTask_port");
    out_jacobianTask_port.doc("Output port for jacobian matrix");
    out_jacobianTask_port.setDataSample(out_jacobianTask_var);
    ports()->addPort(out_jacobianTask_port);

    out_jacobianDotTask_var = Eigen::MatrixXf(12,DOFsize);
    out_jacobianDotTask_var.setZero();
    out_jacobianDotTask_port.setName("out_jacobianDotTask_port");
    out_jacobianDotTask_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotTask_port.setDataSample(out_jacobianDotTask_var);
    ports()->addPort(out_jacobianDotTask_port);

    out_jacobianCstr_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianCstr_var.setZero();
    out_jacobianCstr_port.setName("out_jacobianCstr_port");
    out_jacobianCstr_port.doc("Output port for jacobian matrix");
    out_jacobianCstr_port.setDataSample(out_jacobianCstr_var);
    ports()->addPort(out_jacobianCstr_port);

    out_jacobianDotCstr_var = Eigen::MatrixXf(6,DOFsize);
    out_jacobianDotCstr_var.setZero();
    out_jacobianDotCstr_port.setName("out_jacobianDotCstr_port");
    out_jacobianDotCstr_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotCstr_port.setDataSample(out_jacobianDotCstr_var);
    ports()->addPort(out_jacobianDotCstr_port);

    out_cartPos_var = Eigen::VectorXf(6);
    out_cartPos_var.setZero();
    out_cartPos_port.setName("out_cartPos_port");
    out_cartPos_port.doc("Output port for cartesian position vector");
    out_cartPos_port.setDataSample(out_cartPos_var);
    ports()->addPort(out_cartPos_port);

    out_cartVel_var = Eigen::VectorXf(6);
    out_cartVel_var.setZero();
    out_cartVel_port.setName("out_cartVel_port");
    out_cartVel_port.doc("Output port for cartesian velocity vector");
    out_cartVel_port.setDataSample(out_cartVel_var);
    ports()->addPort(out_cartVel_port);

    portsArePrepared = true;
}

bool TaskDescriberDualArm::exists_test(const std::string& name) {
    if (FILE *file = fopen(name.c_str(), "r")) {
        fclose(file);
        return true;
    } else {
        return false;
    }
}

void TaskDescriberDualArm::castEigenVectorDtoF(Eigen::VectorXd const & d, Eigen::VectorXf & f) {
    f = d.cast <float> ();
}

void TaskDescriberDualArm::castEigenVectorFtoD(Eigen::VectorXf const & f, Eigen::VectorXd & d) {
    d = f.cast <double> ();
}

void TaskDescriberDualArm::castEigenMatrixDtoF(Eigen::MatrixXd const & d, Eigen::MatrixXf & f) {
    f = d.cast <float> ();
}

void TaskDescriberDualArm::castEigenMatrixFtoD(Eigen::MatrixXf const & f, Eigen::MatrixXd & d) {
    d = f.cast <double> ();
}

void TaskDescriberDualArm::displayCurrentState() {
    std::cout << "############## TaskDescriberDualArm State begin " << std::endl;
    std::cout << " angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " velocities " << in_robotstatus_var.velocities << std::endl;

    std::cout << " jacobian full " << out_jacobianFull_var << std::endl;
    std::cout << " jacobianDot full " << out_jacobianDotFull_var << std::endl;

    std::cout << " jacobian task " << out_jacobianTask_var << std::endl;
    std::cout << " jacobianDot task " << out_jacobianDotTask_var << std::endl;

    std::cout << " jacobian cstr " << out_jacobianCstr_var << std::endl;
    std::cout << " jacobianDot cstr " << out_jacobianDotCstr_var << std::endl;

    std::cout << " cartPos " << out_cartPos_var << std::endl;
    std::cout << " cartPos " << out_cartVel_var << std::endl;
    std::cout << "############## TaskDescriberDualArm State end " << std::endl;
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TaskDescriberDualArm)
