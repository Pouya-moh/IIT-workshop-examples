/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#pragma once

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

// Joint value datatype:
#include <rst-rt/kinematics/JointAngles.hpp>

class SimpleExample: public RTT::TaskContext {
public:
    SimpleExample(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setDOFsize(unsigned int DOFsize);
    void printCurrentState();

private:
    // Declare ports and their datatypes
    RTT::OutputPort<rstrt::kinematics::JointAngles> out_angles_port;

    // Actuall joint command to be sent over port:
    rstrt::kinematics::JointAngles out_angles_var;

    // helpers:
    double getSimulationTime();
    unsigned int DOFsize;
    double magnitude;
};

