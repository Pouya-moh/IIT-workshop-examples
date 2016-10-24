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
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>


class ExtendedExample: public RTT::TaskContext {
public:
    ExtendedExample(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    // Declare ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;

    // Actuall joint command to be sent over port:
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::dynamics::JointTorques out_torques_var;
    rstrt::kinematics::JointAngles q_des;
    rstrt::kinematics::JointVelocities qDot_des;

    unsigned int DOFsize;
    double magnitude;

    // some gains:
    Eigen::MatrixXf Kp;
    Eigen::MatrixXf Kv;

    // operations:
    void setKpGain(float value);
    void setKvGain(float value);
    void setKpGainByIndex(int idx, float value);
    void setKvGainByIndex(int idx, float value);
    void setDOFsize(unsigned int DOFsize);
    void printCurrentState();

    // helpers:
    double getSimulationTime();
};

