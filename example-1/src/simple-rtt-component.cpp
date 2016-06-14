/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "simple-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>


ExampleLeftArm::ExampleLeftArm(std::string const & name) : RTT::TaskContext(name) {
    // constructor:
    joint_position_left_arm_command = rstrt::kinematics::JointAngles(COMAN_LEFT_ARM_DOF_SIZE);
    joint_position_left_arm_command.angles.setZero();

    joint_position_left_arm_output_port.setName("JointPositionOutputPort_left_arm");
    joint_position_left_arm_output_port.setDataSample(joint_position_left_arm_command);

    ports()->addPort(joint_position_left_arm_output_port).doc("Output port for sending left arm refrence joint values");

    magnitude = 1.0;
    addProperty("trajectory_magnitude", magnitude).doc("Magnitude of sinusoidal trajectory");
}

bool ExampleLeftArm::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    if (!joint_position_left_arm_output_port.connected())
        return false;
    else
        return true;
}

bool ExampleLeftArm::startHook() {
    // this method starts the component
    return true;
}

void ExampleLeftArm::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    for(int i=0; i<COMAN_LEFT_ARM_DOF_SIZE; ++i)
        joint_position_left_arm_command.angles(i) = magnitude*sin(getSimulationTime());

    joint_position_left_arm_output_port.write(joint_position_left_arm_command);
}

void ExampleLeftArm::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void ExampleLeftArm::cleanupHook() {
    // cleaning the component data
}

double ExampleLeftArm::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(ExampleLeftArm)
