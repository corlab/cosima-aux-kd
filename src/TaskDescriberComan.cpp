/* Author: Niels Dehio
 * Date:   07 December 2016
 *
 * Description: 
 */

#include "TaskDescriberComan.hpp"
#include <rtt/Component.hpp> // needed for the macro at the end of this file


TaskDescriberComan::TaskDescriberComan(std::string const & name) : RTT::TaskContext(name) {
    //prepare operations
    addOperation("setConstrainedVersionMode", &TaskDescriberComan::setConstrainedVersionMode, this, RTT::ClientThread).doc("set constrained version mode");
    addOperation("setTranslationOnly", &TaskDescriberComan::setTranslationOnly, this, RTT::ClientThread).doc("set translation only, or use also orientation");
    addOperation("setDOFsize", &TaskDescriberComan::setDOFsize, this).doc("set DOF size");
    addOperation("displayCurrentState", &TaskDescriberComan::displayCurrentState, this).doc("print current state");

    //other stuff
    currentMode=false;
    this->setConstrainedVersionMode(false);
    this->setTranslationOnly(true);
    portsArePrepared = false;
}

bool TaskDescriberComan::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    return true;
}

bool TaskDescriberComan::startHook() {
    // this method starts the component
    return true;
}

void TaskDescriberComan::updateHook() {
    // this is the actual body of a component. it is called on each cycle

    if (in_jacobianL_port.connected()) {
        in_jacobianL_flow = in_jacobianL_port.read(in_jacobianL_var);
    } else {
        // handle the situation
    }

    if (in_jacobianDotL_port.connected()) {
        in_jacobianDotL_flow = in_jacobianDotL_port.read(in_jacobianDotL_var);
    } else {
        // handle the situation
    }

    if (in_jacobianR_port.connected()) {
        in_jacobianR_flow = in_jacobianR_port.read(in_jacobianR_var);
    } else {
        // handle the situation
    }

    if (in_jacobianDotR_port.connected()) {
        in_jacobianDotR_flow = in_jacobianDotR_port.read(in_jacobianDotR_var);
    } else {
        // handle the situation
    }

    if (in_cartPosL_port.connected()) {
        in_cartPosL_flow = in_cartPosL_port.read(in_cartPosL_var);
    } else {
        // handle the situation
    }

    if (in_cartVelL_port.connected()) {
        in_cartVelL_flow = in_cartVelL_port.read(in_cartVelL_var);
    } else {
        // handle the situation
    }

    if (in_cartPosR_port.connected()) {
        in_cartPosR_flow = in_cartPosR_port.read(in_cartPosR_var);
    } else {
        // handle the situation
    }

    if (in_cartVelR_port.connected()) {
        in_cartVelR_flow = in_cartVelR_port.read(in_cartVelR_var);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if ( (in_jacobianL_flow == RTT::NewData || in_jacobianL_flow == RTT::OldData) &&
         (in_jacobianDotL_flow == RTT::NewData || in_jacobianDotL_flow == RTT::OldData) &&
         (in_jacobianR_flow == RTT::NewData || in_jacobianR_flow == RTT::OldData) &&
         (in_jacobianDotR_flow == RTT::NewData || in_jacobianDotR_flow == RTT::OldData) &&
         (in_cartPosL_flow == RTT::NewData || in_cartPosL_flow == RTT::OldData) &&
         (in_cartVelL_flow == RTT::NewData || in_cartVelL_flow == RTT::OldData) &&
         (in_cartPosR_flow == RTT::NewData || in_cartPosR_flow == RTT::OldData) &&
         (in_cartVelR_flow == RTT::NewData || in_cartVelR_flow == RTT::OldData)
         ){
        assert(in_jacobianL_var.rows()==6);
        assert(in_jacobianL_var.cols()==DOFsize);

        assert(in_jacobianDotL_var.rows()==6);
        assert(in_jacobianDotL_var.cols()==DOFsize);

        assert(in_jacobianR_var.rows()==6);
        assert(in_jacobianR_var.cols()==DOFsize);

        assert(in_jacobianDotR_var.rows()==6);
        assert(in_jacobianDotR_var.cols()==DOFsize);

        assert(in_cartPosL_var.rows()==6);
        assert(in_cartPosL_var.cols()==1);

        assert(in_cartVelL_var.rows()==6);
        assert(in_cartVelL_var.cols()==1);

        assert(in_cartPosR_var.rows()==6);
        assert(in_cartPosR_var.cols()==1);

        assert(in_cartVelR_var.rows()==6);
        assert(in_cartVelR_var.cols()==1);

//        current_time = this->getSimulationTime();
//        time_diff = this->current_time - this->contact_time;
////        if(time_diff > -2.0 && currentMode==false){
////            this->setGains_NullspaceController(0,0);
////        }
//        if(time_diff > 0.0 && currentMode==false){
//            currentMode = true;
//            this->setMode_ConstrainedAuxiliaries(true);
//            this->setMode_PositionController(true);
//            this->setMode_TorqueSuperimposer(true);
//        }

        // specify jacobian full
        out_jacobianFull_var.setZero();
        out_jacobianDotFull_var.setZero();
        if(sendTranslationOnly){
            out_jacobianFull_var.topLeftCorner(3,DOFsize) = in_jacobianL_var.topLeftCorner(3,DOFsize);
            out_jacobianDotFull_var.topLeftCorner(3,DOFsize) = in_jacobianDotL_var.topLeftCorner(3,DOFsize);
            out_jacobianFull_var.bottomLeftCorner(3,DOFsize) = in_jacobianR_var.topLeftCorner(3,DOFsize);
            out_jacobianDotFull_var.bottomLeftCorner(3,DOFsize) = in_jacobianDotR_var.topLeftCorner(3,DOFsize);
        }
        else{
            out_jacobianFull_var.topLeftCorner(6,DOFsize) = in_jacobianL_var;
            out_jacobianDotFull_var.topLeftCorner(6,DOFsize) = in_jacobianDotL_var;
            out_jacobianFull_var.bottomLeftCorner(6,DOFsize) = in_jacobianR_var;
            out_jacobianDotFull_var.bottomLeftCorner(6,DOFsize) = in_jacobianDotR_var;
        }

        // specify jacobian task
        out_jacobianTask_var = selectorTask * out_jacobianFull_var;
        out_jacobianDotTask_var = selectorTask * out_jacobianDotFull_var;

        // specify jacobian constrained
        out_jacobianCstr_var = selectorCstr * out_jacobianFull_var;
        out_jacobianDotCstr_var = selectorCstr * out_jacobianDotFull_var;

        if(sendTranslationOnly){
            out_cartPos_var.head<3>() = in_cartPosL_var.head<3>();
            out_cartPos_var.tail<3>() = in_cartPosR_var.head<3>();
            out_cartVel_var.head<3>() = in_cartVelL_var.head<3>();
            out_cartVel_var.tail<3>() = in_cartVelR_var.head<3>();
        }
        else{
            out_cartPos_var.head<6>() = in_cartPosL_var;
            out_cartPos_var.tail<6>() = in_cartPosR_var;
            out_cartVel_var.head<6>() = in_cartVelL_var;
            out_cartVel_var.tail<6>() = in_cartVelR_var;
        }

    } else if ( (in_jacobianL_flow == RTT::NoData) ||
                (in_jacobianDotL_flow == RTT::NoData) ||
                (in_jacobianR_flow == RTT::NoData) ||
                (in_jacobianDotR_flow == RTT::NoData) ||
                (in_cartPosL_flow == RTT::NoData) ||
                (in_cartVelL_flow == RTT::NoData) ||
                (in_cartPosR_flow == RTT::NoData) ||
                (in_cartVelR_flow == RTT::NoData)
                ) {
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

void TaskDescriberComan::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void TaskDescriberComan::cleanupHook() {
    // cleaning the component data
    portsArePrepared = false;
}

void TaskDescriberComan::setConstrainedVersionMode(bool useConstrainedVersion){
    this->useConstrainedVersion = useConstrainedVersion;
}

void TaskDescriberComan::setTranslationOnly(const bool translationOnly) {
    sendTranslationOnly = translationOnly;
    if(sendTranslationOnly){
        TaskSpaceDimension = 6;
        CstrSpaceDimension = 6;
    }
    else{
        TaskSpaceDimension = 12;
        CstrSpaceDimension = 12;
    }
    selectorTask = Eigen::MatrixXf::Identity(TaskSpaceDimension,TaskSpaceDimension);
    selectorCstr = Eigen::MatrixXf::Identity(CstrSpaceDimension,CstrSpaceDimension);

    if (useConstrainedVersion){
    //define push in Z direction
////    selectorTask(0,0) = 0.0;
////    selectorTask(1,1) = 0.0;
//    selectorTask(2,2) = 0.0;
//    selectorTask(3,3) = 0.0;
//    selectorTask(4,4) = 0.0;
////    selectorTask(5,5) = 0.0;

////    selectorTask(6,6) = 0.0;
////    selectorTask(7,7) = 0.0;
//    selectorTask(8,8) = 0.0;
//    selectorTask(9,9) = 0.0;
//    selectorTask(10,10) = 0.0;
////    selectorTask(11,11) = 0.0;

//    selectorCstr(0,0) = 0.0;
//    selectorCstr(1,1) = 0.0;
////    selectorCstr(2,2) = 0.0;
////    selectorCstr(3,3) = 0.0;
////    selectorCstr(4,4) = 0.0;
//    selectorCstr(5,5) = 0.0;

//    selectorCstr(6,6) = 0.0;
//    selectorCstr(7,7) = 0.0;
////    selectorCstr(8,8) = 0.0;
////    selectorCstr(9,9) = 0.0;
////    selectorCstr(10,10) = 0.0;
//    selectorCstr(11,11) = 0.0;

    //define push in X direction
    if(sendTranslationOnly){
        selectorTask(0,0) = 0.0;
    //    selectorTask(1,1) = 0.0;
    //    selectorTask(2,2) = 0.0;

        selectorTask(3,3) = 0.0;
    //    selectorTask(4,4) = 0.0;
    //    selectorTask(5,5) = 0.0;

    //    selectorCstr(0,0) = 0.0;
        selectorCstr(1,1) = 0.0;
        selectorCstr(2,2) = 0.0;

    //    selectorCstr(3,3) = 0.0;
        selectorCstr(4,4) = 0.0;
        selectorCstr(5,5) = 0.0;
    }
    else{
        selectorTask(0,0) = 0.0;
    //    selectorTask(1,1) = 0.0;
    //    selectorTask(2,2) = 0.0;
    //    selectorTask(3,3) = 0.0;
        selectorTask(4,4) = 0.0;
        selectorTask(5,5) = 0.0;

        selectorTask(6,6) = 0.0;
    //    selectorTask(7,7) = 0.0;
    //    selectorTask(8,8) = 0.0;
    //    selectorTask(9,9) = 0.0;
        selectorTask(10,10) = 0.0;
        selectorTask(11,11) = 0.0;

    //    selectorCstr(0,0) = 0.0;
        selectorCstr(1,1) = 0.0;
        selectorCstr(2,2) = 0.0;
        selectorCstr(3,3) = 0.0;
    //    selectorCstr(4,4) = 0.0;
    //    selectorCstr(5,5) = 0.0;

    //    selectorCstr(6,6) = 0.0;
        selectorCstr(7,7) = 0.0;
        selectorCstr(8,8) = 0.0;
        selectorCstr(9,9) = 0.0;
    //    selectorCstr(10,10) = 0.0;
    //    selectorCstr(11,11) = 0.0;
    }

    }//end if useConstrainedVersion
}

void TaskDescriberComan::setDOFsize(unsigned int DOFsize){
    assert(DOFsize > 0);
    this->DOFsize = DOFsize;

    this->preparePorts();
}


void TaskDescriberComan::preparePorts(){
    if (portsArePrepared){
        ports()->removePort("in_jacobianL_port");
        ports()->removePort("in_jacobianDotL_port");
        ports()->removePort("in_jacobianR_port");
        ports()->removePort("in_jacobianDotR_port");
        ports()->removePort("in_cartPosL_port");
        ports()->removePort("in_cartVelL_port");
        ports()->removePort("in_cartPosR_port");
        ports()->removePort("in_cartPosR_port");
        ports()->removePort("out_jacobianFull_port");
        ports()->removePort("out_jacobianDotFull_port");
        ports()->removePort("out_jacobianTask_port");
        ports()->removePort("out_jacobianDotTask_port");
        ports()->removePort("out_jacobianCstr_port");
        ports()->removePort("out_jacobianDotCstr_port");
    }

    //prepare input
    in_jacobianL_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianL_var.setZero();
    in_jacobianL_port.setName("in_jacobianL_port");
    in_jacobianL_port.doc("Input port for jacobianL matrix");
    ports()->addPort(in_jacobianL_port);
    in_jacobianL_flow = RTT::NoData;

    in_jacobianDotL_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianDotL_var.setZero();
    in_jacobianDotL_port.setName("in_jacobianDotL_port");
    in_jacobianDotL_port.doc("Input port for jacobianDotL matrix");
    ports()->addPort(in_jacobianDotL_port);
    in_jacobianDotL_flow = RTT::NoData;

    in_jacobianR_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianR_var.setZero();
    in_jacobianR_port.setName("in_jacobianR_port");
    in_jacobianR_port.doc("Input port for jacobianR matrix");
    ports()->addPort(in_jacobianR_port);
    in_jacobianR_flow = RTT::NoData;

    in_jacobianDotR_var = Eigen::MatrixXf(6,DOFsize);
    in_jacobianDotR_var.setZero();
    in_jacobianDotR_port.setName("in_jacobianDotR_port");
    in_jacobianDotR_port.doc("Input port for jacobianDotR matrix");
    ports()->addPort(in_jacobianDotR_port);
    in_jacobianDotR_flow = RTT::NoData;

    in_cartPosL_var = Eigen::VectorXf(6);
    in_cartPosL_var.setZero();
    in_cartPosL_port.setName("in_cartPosL_port");
    in_cartPosL_port.doc("Input port for cartPosL vector");
    ports()->addPort(in_cartPosL_port);
    in_cartPosL_flow = RTT::NoData;

    in_cartVelL_var = Eigen::VectorXf(6);
    in_cartVelL_var.setZero();
    in_cartVelL_port.setName("in_cartVelL_port");
    in_cartVelL_port.doc("Input port for cartPosL vector");
    ports()->addPort(in_cartVelL_port);
    in_cartVelL_flow = RTT::NoData;

    in_cartPosR_var = Eigen::VectorXf(6);
    in_cartPosR_var.setZero();
    in_cartPosR_port.setName("in_cartPosR_port");
    in_cartPosR_port.doc("Input port for cartPosR vector");
    ports()->addPort(in_cartPosR_port);
    in_cartPosR_flow = RTT::NoData;

    in_cartVelR_var = Eigen::VectorXf(6);
    in_cartVelR_var.setZero();
    in_cartVelR_port.setName("in_cartVelR_port");
    in_cartVelR_port.doc("Input port for cartPosR vector");
    ports()->addPort(in_cartVelR_port);
    in_cartVelR_flow = RTT::NoData;

    //prepare output
    out_jacobianFull_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    out_jacobianFull_var.setZero();
    out_jacobianFull_port.setName("out_jacobianFull_port");
    out_jacobianFull_port.doc("Output port for jacobian matrix");
    out_jacobianFull_port.setDataSample(out_jacobianFull_var);
    ports()->addPort(out_jacobianFull_port);

    out_jacobianDotFull_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    out_jacobianDotFull_var.setZero();
    out_jacobianDotFull_port.setName("out_jacobianDotFull_port");
    out_jacobianDotFull_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotFull_port.setDataSample(out_jacobianDotFull_var);
    ports()->addPort(out_jacobianDotFull_port);

    out_jacobianTask_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    out_jacobianTask_var.setZero();
    out_jacobianTask_port.setName("out_jacobianTask_port");
    out_jacobianTask_port.doc("Output port for jacobian matrix");
    out_jacobianTask_port.setDataSample(out_jacobianTask_var);
    ports()->addPort(out_jacobianTask_port);

    out_jacobianDotTask_var = Eigen::MatrixXf(TaskSpaceDimension,DOFsize);
    out_jacobianDotTask_var.setZero();
    out_jacobianDotTask_port.setName("out_jacobianDotTask_port");
    out_jacobianDotTask_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotTask_port.setDataSample(out_jacobianDotTask_var);
    ports()->addPort(out_jacobianDotTask_port);

    out_jacobianCstr_var = Eigen::MatrixXf(CstrSpaceDimension,DOFsize);
    out_jacobianCstr_var.setZero();
    out_jacobianCstr_port.setName("out_jacobianCstr_port");
    out_jacobianCstr_port.doc("Output port for jacobian matrix");
    out_jacobianCstr_port.setDataSample(out_jacobianCstr_var);
    ports()->addPort(out_jacobianCstr_port);

    out_jacobianDotCstr_var = Eigen::MatrixXf(CstrSpaceDimension,DOFsize);
    out_jacobianDotCstr_var.setZero();
    out_jacobianDotCstr_port.setName("out_jacobianDotCstr_port");
    out_jacobianDotCstr_port.doc("Output port for jacobianDot matrix");
    out_jacobianDotCstr_port.setDataSample(out_jacobianDotCstr_var);
    ports()->addPort(out_jacobianDotCstr_port);

    out_cartPos_var = Eigen::VectorXf(TaskSpaceDimension);
    out_cartPos_var.setZero();
    out_cartPos_port.setName("out_cartPos_port");
    out_cartPos_port.doc("Output port for cartPos vector");
    out_cartPos_port.setDataSample(out_cartPos_var);
    ports()->addPort(out_cartPos_port);

    out_cartVel_var = Eigen::VectorXf(TaskSpaceDimension);
    out_cartVel_var.setZero();
    out_cartVel_port.setName("out_cartVel_port");
    out_cartVel_port.doc("Output port for cartVel vector");
    out_cartVel_port.setDataSample(out_cartVel_var);
    ports()->addPort(out_cartVel_port);


    portsArePrepared = true;
}


void TaskDescriberComan::displayCurrentState() {
    std::cout << "############## TaskDescriberComan State begin " << std::endl;
    std::cout << " jacobian full \n" << out_jacobianFull_var << std::endl;
    std::cout << " jacobianDot full \n" << out_jacobianDotFull_var << std::endl;

    std::cout << " jacobian task \n" << out_jacobianTask_var << std::endl;
    std::cout << " jacobianDot task \n" << out_jacobianDotTask_var << std::endl;

    std::cout << " jacobian cstr \n" << out_jacobianCstr_var << std::endl;
    std::cout << " jacobianDot cstr \n" << out_jacobianDotCstr_var << std::endl;

    std::cout << " cartPos \n" << out_cartPos_var << std::endl;
    std::cout << " cartVel \n" << out_cartVel_var << std::endl;
    std::cout << "############## TaskDescriberComan State end " << std::endl;
}


//this macro should appear only once per library
//ORO_CREATE_COMPONENT_LIBRARY()

// This macro, as you can see, creates the component. Every component should have this!
ORO_LIST_COMPONENT_TYPE(TaskDescriberComan)
