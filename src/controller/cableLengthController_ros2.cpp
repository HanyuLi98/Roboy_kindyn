/*
    BSD 3-Clause License
    Copyright (c) 2018, Roboy
            All rights reserved.
    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    author: Simon Trendel ( st@gi.ai ), 2018
    description: A Cable length controller for joint position targets using PD control
*/


#include <type_traits>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "kindyn/robot.hpp"
#include "kindyn/controller/cardsflow_state_interface.hpp"
#include <roboy_simulation_msgs/ControllerType.h>
#include <math.h>
#include <cmath>
#include <std_msgs/Float32.h>
#include <roboy_control_msgs/SetControllerParameters.h>


#include "rclcpp/rclcpp.hpp"







#define M_2PI 2*M_PI

using namespace std;

double wrap_pos_neg_pi(double angle)
{
    return fmod(angle + M_PI, M_2PI) - M_PI;
}

class CableLengthController : public controller_interface::Controller<hardware_interface::CardsflowCommandInterface> {
public:
    /**
     * Constructor
     */
    CableLengthController() {};

    /**
     * Initializes the controller. Will be call by controller_manager when loading this controller
     * @param hw pointer to the hardware interface
     * @param node the nodehandle
     * @return success
     */                                                         //ros::NodeHandle &n                
    bool init(hardware_interface::CardsflowCommandInterface *hw, rclcpp::Node::SharedPtr node) {
        nh = node;
        // get joint name from the parameter server
        //.  !nh.get_parameter
        if (!nh->get_parameter("joint", joint_name)) {
            //ROS_ERROR   
            RCLCPP_ERROR(nh->get_logger(), "No joint given (namespace: %s)", nh->get_namespace().c_str());
            return false;
        }
        
        //not needed 
        // In ROS2, spinners are replaced by executors
        // Executors should be created outside of the class and spin should be called in a separate thread if needed

        // spinner.reset(new ros::AsyncSpinner(0));
        // spinner->start();

        controller_state = nh->create_publisher<roboy_simulation_msgs::msg::ControllerType>("/controller_type", 1);
        rclcpp::Rate r(10);

        while(controller_state->getNumSubscribers()==0) // we wait until the controller state is available
            r.sleep();
        joint = hw->getHandle(joint_name); // throws on failure
        joint_index = joint.getJointIndex();
        last_update = nh->now();
//        joint_command = nh.subscribe((joint_name+"/target").c_str(),1,&CableLengthController::JointPositionCommand, this);
//      if needed, 
//      joint_command = nh->create_subscription<std_msgs::msg::Float32>(joint_name + "/target", 10, 
//                  std::bind(&CableLengthController::JointPositionCommand, this, std::placeholders::_1));
        return true;
    }

    /**
     * Called regularily by controller manager. The length change of the cables wrt to a PD controller on the joint target
     * position is calculated.
     * @param time current time
     * @param period period since last control
     */
    void update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        double q = joint.getPosition();
        double q_target = joint.getJointPositionCommand();
        MatrixXd L = joint.getL();
        VectorXd Kp = *joint.Kp_;
        VectorXd Kd = *joint.Kd_;

        double p_error = wrap_pos_neg_pi(q - q_target);
        // we use the joint_index column of the L matrix to calculate the result for this joint only
        VectorXd ld = L.col(joint_index) * (Kd[joint_index] * (p_error - p_error_last)/period.toSec() + Kp[joint_index] * p_error);
        joint.setMotorCommand(ld);
        p_error_last = p_error;
        last_update = time;
    }

    /**
     * Called by controller manager when the controller is about to be started
     * @param time current time
     */
    void starting(const rclcpp::Time& time) {
        RCLCPP_WARN(nh->get_logger(), "cable length controller started for %s with index %d", joint_name.c_str(), joint_index);
        roboy_simulation_msgs::ControllerType msg;
        msg.joint_name = joint_name;
        msg.type = CARDSflow::ControllerType::cable_length_controller;
        controller_state->publish(msg);
    }
    /**
     * Called by controller manager when the controller is about to be stopped
     * @param time current time
     */
    void stopping(const rclcpp::Time& time) {
        RCLCPP_WARN(nh->get_logger(), "cable length controller stopped for %s", joint_name.c_str());
    }

    /**
     * Joint position command callback for this joint
     * @param msg joint position target in radians
     */
    void JointPositionCommand(const std_msgs::Float32ConstPtr &msg){
        joint.setJointPositionCommand(msg->data);
    }

private:
    double p_error_last = 0; /// last error
    rclcpp::Node::SharedPtr nh; /// ROS2 node

    //ros::Publisher controller_state; /// publisher for controller state
    rclcpp::Publisher<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_state;/// publisher for controller state
   
    //ros::ServiceServer controller_parameter_srv; /// service for controller parameters
                    //type not sure, because controller_parameter_srv has not been used so far 
    rclcpp::Service<rclcpp::srv::SetParameters>::SharedPtr controller_parameter_srv; /// service for controller parameters


    // spinner is intergrated into ROS2 Node
    //boost::shared_ptr<ros::AsyncSpinner> spinner; /// ROS async spinner

    
    hardware_interface::CardsflowHandle joint; /// cardsflow joint handle for access to joint/cable model state
    
    //ros::Subscriber joint_command; /// joint command subscriber
                        //type of template parameters not sure, here is consistent with JointPositionCommand
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_command; /// joint command subscriber
    
    string joint_name; /// name of the controlled joint
    int joint_index; /// index of the controlled joint in the robot model
    rclcpp::Time last_update; /// time of last update
};
PLUGINLIB_EXPORT_CLASS(CableLengthController, controller_interface::ControllerBase);
