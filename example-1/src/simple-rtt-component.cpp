/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#include "simple-rtt-component.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>


SimpleExample::SimpleExample(std::string const & name) : RTT::TaskContext(name) {
    // constructor:
    addOperation("setDOFsize", &SimpleExample::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("printCurrentState", &SimpleExample::printCurrentState, this, RTT::ClientThread).doc("print current state");

    magnitude = 1.0;
    addProperty("trajectory_magnitude", magnitude).doc("Magnitude of sinusoidal trajectory");
}

bool SimpleExample::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
}

bool SimpleExample::startHook() {
    // this method starts the component
    if (!out_angles_port.connected())
        return false;
    else
        return true;
    return true;
}

void SimpleExample::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    for(int i=0; i<DOFsize; ++i)
        out_angles_var.angles(i) = magnitude*sin(getSimulationTime());

    out_angles_port.write(out_angles_var);
}

void SimpleExample::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void SimpleExample::cleanupHook() {
    // cleaning the component data
}

double SimpleExample::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void SimpleExample::setDOFsize(unsigned int DOFsize){
    this->DOFsize = DOFsize;

    // prepare input ports:
    // none...

    //prepare output ports:
    out_angles_var = rstrt::kinematics::JointAngles(DOFsize);
    out_angles_var.angles.setZero();
    out_angles_port.setName("out_angles_port");
    out_angles_port.doc("Output port for sending torque values");
    out_angles_port.setDataSample(out_angles_var);
    ports()->addPort(out_angles_port);
}

void SimpleExample::printCurrentState(){
    std::cout << "############## SimpleExample State begin " << std::endl;
    std::cout << " command angles " << out_angles_var.angles << std::endl;
    std::cout << " magnitude " << magnitude << std::endl;
    std::cout << "############## SimpleExample State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(SimpleExample)
