/* Author: Pouya Mohammadi
 * Date:   09/06/2016
 *
 * Description: This is a simple orocos/rtt component template. It should be
 *              modified and extended by users to accomodate their needs.
 */

#ifndef EXTENDEDRTTCOMPONENT_HPP
#define EXTENDEDRTTCOMPONENT_HPP

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>

// Joint value datatype:
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>


#define COMAN_RIGHT_ARM_DOF_SIZE 7

class ExampleRightArm: public RTT::TaskContext {
public:
    ExampleRightArm(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

private:
    // Declare ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> joint_torque_right_arm_output_port;
    RTT::InputPort<rstrt::robot::JointState> robot_feedback_port; //Input!

    // Data flow:
    RTT::FlowStatus robot_feedback_flow;

    // Actuall joint command to be sent over port:
    rstrt::dynamics::JointTorques joint_torque_right_arm_command;
    rstrt::kinematics::JointVelocities q_dot;
    rstrt::kinematics::JointAngles q_des;
    rstrt::kinematics::JointAngles q_fb;

    rstrt::robot::JointState robot_state;

    // helpers:
    double getSimulationTime();

    // some gains:
    Eigen::MatrixXf Kp;
    Eigen::MatrixXf Kv;
    float Kp_gain;
    float Kv_gain;

    // operations:
    void setKpGainByIndex(int idx, float value);
    void setKvGainByIndex(int idx, float value);


};

#endif // SIMPLERTTCOMPONENT_HPP
