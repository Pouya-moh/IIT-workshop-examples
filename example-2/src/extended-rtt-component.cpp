/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "extended-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>


ExampleRightArm::ExampleRightArm(std::string const & name) : RTT::TaskContext(name) {
    // constructor:
    // output port and coommand value:
    joint_torque_right_arm_command = rstrt::dynamics::JointTorques(COMAN_RIGHT_ARM_DOF_SIZE);
    joint_torque_right_arm_command.torques.setZero();

    joint_torque_right_arm_output_port.setName("JointPositionOutputPort_right_arm");
    joint_torque_right_arm_output_port.setDataSample(joint_torque_right_arm_command);

    ports()->addPort(joint_torque_right_arm_output_port).doc("Output port for sending right arm refrence torque values");

    // input ports:
    robot_feedback_port.setName("RobotStateFeedback");
    robot_feedback_flow = RTT::NoData;

    ports()->addPort(robot_feedback_port).doc("This port receives the state feedback from robot.");

    // adding operations
    addOperation("setKdGainByIndex", &ExampleRightArm::setKpGainByIndex, this, RTT::ClientThread);
    addOperation("setKdGainByIndex", &ExampleRightArm::setKpGainByIndex, this, RTT::ClientThread);
}

bool ExampleRightArm::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:
    this->Kp = Eigen::Matrix<float, COMAN_RIGHT_ARM_DOF_SIZE,COMAN_RIGHT_ARM_DOF_SIZE>::Identity()*Kp_gain;
    this->Kv = Eigen::Matrix<float, COMAN_RIGHT_ARM_DOF_SIZE,COMAN_RIGHT_ARM_DOF_SIZE>::Identity()*Kv_gain;


    if (!(joint_torque_right_arm_output_port.connected() && (robot_feedback_port.connected())))
        return false;
    else
        return true;
}

bool ExampleRightArm::startHook() {
    // this method starts the component
    return true;
}

void ExampleRightArm::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (robot_feedback_port.connected()) {
        // read into "currJntPos" and save state of data into "currJntPos_Flow", which can be "NewData", "OldData" or "NoData".
        robot_feedback_flow = robot_feedback_port.read(robot_state);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if (robot_feedback_flow == RTT::NewData){

    } else if (robot_feedback_flow == RTT::OldData) {

    } else if (robot_feedback_flow == RTT::NoData){

    } else {
        // there should be something really wrong!
    }

    // actual controller!
    q_fb.angles  = robot_state.angles;
    q_dot        = cos(getSimulationTime());
    q_des        = sin(getSimulationTime());

    joint_torque_right_arm_command.torques = Kp*(q_des.angles - q_fb.angles) + Kv*q_dot.velocities;

    // write it to port
    joint_torque_right_arm_output_port.write(joint_torque_right_arm_command);
}

void ExampleRightArm::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ExampleRightArm::cleanupHook() {
    // cleaning the component data
}

double ExampleRightArm::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void ExampleRightArm::setKpGainByIndex(int idx, float value) {
    this->Kp(idx, idx) = value;
}

void ExampleRightArm::setKvGainByIndex(int idx, float value) {
    this->Kv(idx, idx) = value;
}

// This macro, as you can see, creates the component. Every component should have this!
// talk about the way it differs from the first component
ORO_LIST_COMPONENT_TYPE(ExampleRightArm)
