/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "extended-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>


ExtendedExample::ExtendedExample(std::string const & name) : RTT::TaskContext(name) {
    // constructor:
    addOperation("setKdGain", &ExtendedExample::setKpGain, this, RTT::ClientThread);
    addOperation("setKvGain", &ExtendedExample::setKvGain, this, RTT::ClientThread);
    addOperation("setKdGainByIndex", &ExtendedExample::setKpGainByIndex, this, RTT::ClientThread);
    addOperation("setKvGainByIndex", &ExtendedExample::setKvGainByIndex, this, RTT::ClientThread);
    addOperation("setDOFsize", &ExtendedExample::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("printCurrentState", &ExtendedExample::printCurrentState, this, RTT::ClientThread).doc("print current state");

    magnitude = 1.0;
    addProperty("trajectory_magnitude", magnitude).doc("Magnitude of sinusoidal trajectory");
}

bool ExtendedExample::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:
    this->setKpGain(100);
    this->setKvGain(10);

    if (!(out_torques_port.connected() && (in_robotstatus_port.connected())))
        return false;
    else
        return true;
}

bool ExtendedExample::startHook() {
    // this method starts the component
    return true;
}

void ExtendedExample::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        // read into "currJntPos" and save state of data into "currJntPos_Flow", which can be "NewData", "OldData" or "NoData".
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if (in_robotstatus_flow == RTT::NewData){

    } else if (in_robotstatus_flow == RTT::OldData) {

    } else if (in_robotstatus_flow == RTT::NoData){

    } else {
        // there should be something really wrong!
    }

    // actual controller!
    for(int i=0; i<DOFsize; ++i){
        q_des.angles(i)        = magnitude*sin(getSimulationTime());
        qDot_des.velocities(i) = magnitude*cos(getSimulationTime());
    }

    out_torques_var.torques = Kp*(q_des.angles - in_robotstatus_var.angles) - Kv*qDot_des.velocities;

    // write it to port
    out_torques_port.write(out_torques_var);
}

void ExtendedExample::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ExtendedExample::cleanupHook() {
    // cleaning the component data
}

double ExtendedExample::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void ExtendedExample::setKpGain(float value) {
    this->Kp = Eigen::MatrixXf::Identity(DOFsize,DOFsize)*value;
}

void ExtendedExample::setKvGain(float value) {
    this->Kv = Eigen::MatrixXf::Identity(DOFsize,DOFsize)*value;
}

void ExtendedExample::setKpGainByIndex(int idx, float value) {
    this->Kp(idx, idx) = value;
}

void ExtendedExample::setKvGainByIndex(int idx, float value) {
    this->Kv(idx, idx) = value;
}

void ExtendedExample::setDOFsize(unsigned int DOFsize){
    this->DOFsize = DOFsize;

    // prepare input ports:
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output ports:
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    // initializing data
    q_des = rstrt::kinematics::JointAngles(DOFsize);
    q_des.angles.setZero();

    qDot_des = rstrt::kinematics::JointVelocities(DOFsize);
    qDot_des.velocities.setZero();
}

void ExtendedExample::printCurrentState(){
    std::cout << "############## ExtendedExample State begin " << std::endl;
    std::cout << " feedback angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " feedback velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " feedback torques " << in_robotstatus_var.torques << std::endl;
    std::cout << " command torques " << out_torques_var.torques << std::endl;
    std::cout << "############## ExtendedExample State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(ExtendedExample)
