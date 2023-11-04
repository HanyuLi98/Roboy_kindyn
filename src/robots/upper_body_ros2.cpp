// #include "kindyn/vrpuppet.hpp"
#include "kindyn/robot.hpp"
#include <thread>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/RoboyState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <roboy_middleware_msgs/SetStrings.h>
#include <roboy_middleware_msgs/SystemStatus.h>
#include <roboy_middleware_msgs/BodyPart.h>
#include <roboy_simulation_msgs/Tendon.h>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <std_srvs/Empty.h>
// #include <tf/tf.h>
// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;

class UpperBody: public cardsflow::kindyn::Robot{
private:
    //ros::NodeHandlePtr nh; /// ROS nodehandle
    rclcpp::Node::SharedPtr node_;  // ROS 2 node

    // ros::Publisher motor_command, system_status_pub, tendon_motor_pub, joint_target_pub; /// motor command publisher
    rclcpp::Publisher<roboy_middleware_msgs::MotorCommand>::SharedPtr motor_command;
    rclcpp::Publisher<roboy_middleware_msgs::SystemStatus>::SharedPtr system_status_pub;
    rclcpp::Publisher<roboy_simulation_msgs::Tendon>::SharedPtr tendon_motor_pub;
    rclcpp::Publisher<sensor_msgs::JointState>::SharedPtr joint_target_pub;

    // ros::Subscriber motor_state_sub, motor_info_sub, roboy_state_sub;
    rclcpp::Subscription<roboy_middleware_msgs::MotorState::ConstPtr>::SharedPtr motor_state_sub;
    rclcpp::Subscription<roboy_middleware_msgs::RoboyState::ConstPtr>::SharedPtr roboy_state_sub;
    rclcpp::Subscription<roboy_middleware_msgs::MotorInfo::ConstPtr>::SharedPtr motor_info_sub;


    //vector<ros::ServiceServer> init_poses;

    //ros::ServiceServer init_pose;
    rclcpp::Service<?????>::SharedPtr init_pose;
    
    //ros::AsyncSpinner *spinner;
    // ros::ServiceClient motor_control_mode, motor_config, control_mode;

    map<int,int> pos, initial_pos;
    map<string, rclcpp::Client<roboy_middleware_msgs::srv::MotorConfigService>::SharedPtr> motor_config;
    map<string, rclcpp::Client<roboy_middleware_msgs::srv::MotorConfigService>::SharedPtr> motor_control_mode;
    map<string, rclcpp::Client<roboy_middleware_msgs::srv::ControlMode>::SharedPtr> control_mode;


    map<string, bool> motor_status_received;
    map<int, bool> communication_established; // keeps track of communication quality for each motor
    map<int,float> l_offset, position, tendon_length;
    VectorXd l_current;
    map<string, vector<float>> integral, error_last;
    boost::shared_ptr<tf2_ros::TransformListener> listener;
    std::vector<string> body_parts = {"shoulder_right", "shoulder_left","head", "wrist_right","wrist_left"};//, "shoulder_left"};//}, "elbow_left"};
    map<string, bool> init_called;
    boost::shared_ptr<std::thread> system_status_thread;
    rclcpp::Time prev_roboy_state_time;
    enum BulletPublish {zeroes, current};
public:
    /**
     * Constructor
     * @param urdf path to urdf
     * @param cardsflow_xml path to cardsflow xml
     */
    UpperBody(string urdf, string cardsflow_xml, string robot_model, bool debug): node_(node),debug_(debug)
    
    {

        // if (!ros::isInitialized()) {
        //     int argc = 0;
        //     char **argv = NULL;
        //     ros::init(argc, argv, robot_model + "upper_body");
        // }

        // //debug_ = debug;

        // nh = ros::NodeHandlePtr(new ros::NodeHandle);
        // spinner = new ros::AsyncSpinner(0);
        // spinner->start();

        vector<string> joint_names;
        // nh->getParam("joint_names", joint_names);
        // nh->getParam("external_robot_state", external_robot_state);
        node_->declare_parameter("joint_names");
        node_->declare_parameter("external_robot_state");
        node_->get_parameter("joint_names", joint_names);
        node_->get_parameter("external_robot_state", external_robot_state);

        RCLCPP_INFO(node_->get_logger(), "External robot state: %s", external_robot_state.c_str());
        topic_root = "/roboy/" + robot_model + "/";

        init(urdf,cardsflow_xml,joint_names);
//        listener.reset(new tf::TransformListener);
        l_current.resize(kinematics.number_of_cables);
        l_current.setZero();

        update();


        motor_state_sub = node_->create_subscription<roboy_middleware_msgs::MotorState::ConstPtr>(topic_root + "middleware/MotorState", 1, std::bind(&UpperBody::MotorState, this, std::placeholders::_1));
        roboy_state_sub = node_->create_subscription<roboy_middleware_msgs::RoboyState::ConstPtr>(topic_root + "middleware/RoboyState", 1, std::bind(&UpperBody::RoboyState, this, std::placeholders::_1));
        motor_info_sub = node_->create_subscription<roboy_middleware_msgs::MotorInfo::ConstPtr>(topic_root + "middleware/MotorInfo", 1, std::bind(&UpperBody::MotorInfo, this, std::placeholders::_1));

        for (auto body_part: body_parts) {
            init_called[body_part] = false;
            motor_config[body_part] = node_->create_client<roboy_middleware_msgs::srv::MotorConfigService>(topic_root + "middleware/" + body_part + "/MotorConfig");
            control_mode[body_part] = node_->create_client<roboy_middleware_msgs::srv::ControlMode>(topic_root + "middleware/ControlMode");//+body_part+"ControlMode");
        }

        //motor_command = nh->advertise<roboy_middleware_msgs::MotorCommand>(topic_root + "middleware/MotorCommand",1);
        motor_command = node_->create_publisher<roboy_middleware_msgs::MotorCommand>(topic_root + "middleware/MotorCommand", 1);
        //init_pose = nh->advertiseService(topic_root + "init_pose", &UpperBody::initPose,this);
        init_pose = node_->create_service</*ServiceType*/>(topic_root + "init_pose", std::bind(&UpperBody::initPose, this, std::placeholders::_1, std::placeholders::_2));
        //joint_target_pub = nh->advertise<sensor_msgs::JointState>(topic_root + "simulation/joint_targets",1);
        joint_target_pub = node_->create_publisher<sensor_msgs::JointState>(topic_root + "simulation/joint_targets", 1);
        //system_status_pub = nh->advertise<roboy_middleware_msgs::SystemStatus>(topic_root + "control/SystemStatus",1);
        system_status_pub = node_->create_publisher<roboy_middleware_msgs::SystemStatus>(topic_root + "control/SystemStatus", 1);
        

        system_status_thread = boost::shared_ptr<std::thread>(new std::thread(&UpperBody::SystemStatusPublisher, this));
        system_status_thread->detach();

        //tendon_motor_pub = nh->advertise<roboy_simulation_msgs::Tendon>(topic_root + "control/tendon_state_motor", 1);
        tendon_motor_pub = node_->create_publisher<roboy_simulation_msgs::Tendon>(topic_root + "control/tendon_state_motor", 1);

        //nh->setParam("initialized", init_called);
        node_->set_parameter(rclcpp::Parameter("initialized", init_called));
        
        RCLCPP_INFO(node_->get_logger(), "Finished setup");
    };

    ~UpperBody() {
        if (system_status_thread->joinable())
            system_status_thread->join();
    }

    void SystemStatusPublisher() {
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) {
            auto msg = roboy_middleware_msgs::SystemStatus();
            msg.header.stamp = rclcpp::Clock().now();
            auto body_part = roboy_middleware_msgs::BodyPart();
            for (auto part: body_parts) {
                body_part.name = part;
                body_part.status = !init_called[part];
                msg.body_parts.push_back(body_part);
            }
            system_status_pub->publish(msg);
            rate.sleep();
        }

    }


    bool initPose(roboy_middleware_msgs::SetStrings::Request &req,
                  roboy_middleware_msgs::SetStrings::Response &res){
        res.result = true;

        if (find(req.strings.begin(), req.strings.end(), "all") != req.strings.end()) {
            req.strings = body_parts;
        }

        for (string body_part: req.strings) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(),"init called for: " << body_part);
            init_called[body_part] = false;
            auto r = initBodyPart(body_part);
            res.result = r*res.result;
        }

        return res.result;
    }


    bool initBodyPart(string name) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger(),"initBodyPart: " << name);

        std::vector<int> motor_ids;
        try {
            node_->getParam(name+"/motor_ids", motor_ids);
        }
        catch (const std::exception&) {
            RCLCPP_ERROR(rclcpp::get_logger(),"motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", name);
            return false;
        }
        int pwm;
        try {
            if (name == "wrist_left" || name == "wrist_right") {
                node_->getParam("m3_pwm", pwm);
            } else {
                node_->getParam("pwm",pwm);
            }
        }
        catch (const std::exception&) {
           RCLCPP_ERROR(rclcpp::get_logger(),"rosparam pwm or init_m3_displacement is not set. will not init.");
            return false;
        }


        RCLCPP_INFO(rclcpp::get_logger(),"changing control mode of motors to PWM with %d",pwm);
        roboy_middleware_msgs::ControlMode msg;


        // if (name == "wrist_left" || name == "wrist_right") {
        //     msg.request.control_mode = DISPLACEMENT;
        // } else {
            msg.request.control_mode = DIRECT_PWM;
        // }
        // TODO: fix in plexus PWM direction for the new motorboard
        std::vector<float> set_points(motor_ids.size(), pwm);
        for (auto m: motor_ids) msg.request.global_id.push_back(m);
        msg.request.set_points = set_points;

        stringstream str1;
        for(int i=0;i<msg.request.set_points.size();i++) {
            int motor_id = motor_ids[i];

            str1 << msg.request.global_id[i] << "\t|\t" << msg.request.set_points[i] << endl;
        }


        RCLCPP_INFO_STREAM(rclcpp::get_logger(), str1.str());


        if (!control_mode[name].call(msg)) {
            RCLCPP_ERROR(rclcpp::get_logger(), "Changing control mode for %s didnt work", name);
            return false;
        }


        rclcpp::Time t0;
        t0 = rclcpp::Clock().now();
        double timeout = 0;
        node_->getParam("timeout",timeout);
        if(timeout==0) {
            int seconds = 5;
            while (( rclcpp::Clock().now() - t0).toSec() < 5) {
                //ROS_INFO_THROTTLE(1, "waiting %d", seconds--);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(1), "waiting %d", seconds--);
            }
        }else{
            int seconds = timeout;
            while ((ros::Time::now() - t0).toSec() < timeout) {
                // ROS_INFO_THROTTLE(1, "waiting %d", seconds--);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(1), "waiting %d", seconds--);
            }
        }
        motor_status_received[name] = true;
        if(!motor_status_received[name]) {
            RCLCPP_ERROR(rclcpp::get_logger(),"did not receive motor status for %s, try again", name);
            return false;
        }

        stringstream str;
        str << "saving position offsets:" << endl << "motor id  |   position offset [ticks]  | length_sim[m] | length offset[m]" << endl;

        // for (int i = 0; i<motor_ids.size();i++) str << motor_ids[i] << ": " << position[motor_ids[i]] << ", ";
        // str << endl;


        // Make sure we get current actual joint state
        rclcpp::Time t0;
        t0 = rclcpp::Clock().now();
        int seconds = 1;
        while (( rclcpp::Clock().now(); - t0).toSec() < 1) {
            ROS_INFO_THROTTLE(1, "waiting %d for external joint state", seconds--);
        }


        // Get current tendon length
        kinematics.setRobotState(q, qd);
        kinematics.getRobotCableFromJoints(l_current);

        for (int i = 0; i < motor_ids.size(); i++) {
            int motor_id = motor_ids[i];
            RCLCPP_WARN_STREAM(rclcpp::get_logger(),name << " info print");
            l_offset[motor_id] = l_current[motor_id] + position[motor_id];
            str << motor_id << "\t|\t" << position[motor_id] << "\t|\t" << l_current[motor_id] << "\t|\t"
                << l_offset[motor_id] << endl;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger(),str.str());

        RCLCPP_INFO_STREAM(rclcpp::get_logger(),"changing control mode of %s to POSITION" << name);

        roboy_middleware_msgs::ControlMode msg1;
        msg1.request.control_mode = ENCODER0_POSITION;
        for (int id: motor_ids) {
            msg1.request.global_id.push_back(id);
            msg1.request.set_points.push_back(position[id]);
        }

        if (!control_mode[name].call(msg1)) {
            RCLCPP_ERROR(rclcpp::get_logger(),"Changing control mode for %s didnt work" << name);
            return false;
        }

        vector<float> _integral(motor_ids.size(), 0);
        vector<float> _error(motor_ids.size(), 0);
        integral[name] = _integral;
        error_last[name] = _error;

        update();


        if(this->external_robot_state) {
            // Set current state to bullet
            publishBulletTarget(name, BulletPublish::current);

            t0 = rclcpp::Clock().now();
            int seconds = 3;
            while ((t0 = rclcpp::Clock().now(); - t0).toSec() < 3) {
                //ROS_INFO_THROTTLE(1, "waiting %d for setting bullet", seconds--);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(1), "waiting %d for setting bullet", seconds--);
            }

            // Move back to zero position
            publishBulletTarget(name, BulletPublish::zeroes);
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger(),"%s pose init done" << name);
        init_called[name] = true;
        node_->setParam("initialized", init_called);

        return true;


    }

    /**
     * Publish Target point to bullet
     * @param body_part
     * @param zeroes_or_current will publish either "zeroes" or "current" as targets to Bullet
     */
    void publishBulletTarget(string body_part, BulletPublish zeroes_or_current){

        sensor_msgs::JointState target_msg;

        // set respecitve body part joint targets to 0
        string endeffector;
        node_->getParam(body_part+"/endeffector", endeffector);
        if ( !endeffector.empty() ) {
            vector<string> ik_joints;
            node_->getParam((endeffector + "/joints"), ik_joints);
            if (ik_joints.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger(),
                        "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                        body_part.c_str());
            }
            else {

                for (auto joint: ik_joints) {
                    int joint_index = kinematics.GetJointIdByName(joint);
                    if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                        target_msg.name.push_back(joint);

                        if(zeroes_or_current == BulletPublish::zeroes) {
                            target_msg.position.push_back(0);
                            RCLCPP_WARN_STREAM(rclcpp::get_logger(),"Set target 0 for " << joint);
                        }else if(zeroes_or_current == BulletPublish::current){
                            target_msg.position.push_back(q[joint_index]);
                            RCLCPP_WARN_STREAM(rclcpp::get_logger(),"Set target " << q[joint_index] << " for " << joint);
                        }
                    }
                }

            }
        }

        joint_target_pub->publish(target_msg);
    }


    string findBodyPartByMotorId(int id) {
        string ret = "unknown";
        for (auto body_part: body_parts) {
            std::vector<int> motor_ids;
            try {
//                mux.lock();
                node_->getParam(body_part+"/motor_ids", motor_ids);
//                mux.unlock();
            }
            catch (const std::exception&) {
                RCLCPP_ERROR(rclcpp::get_logger(),"motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
                return ret;
            }
            if (find(motor_ids.begin(), motor_ids.end(), id) != motor_ids.end()) {
                return body_part;
            }
        }
        RCLCPP_WARN_ONCE(rclcpp::get_logger(), "Seems like motor with id %d does not belong to any body part", id);
        return ret;
    }

    void MotorState(const roboy_middleware_msgs::MotorState::ConstPtr &msg){
        prev_roboy_state_time = rclcpp::Clock().now();

        int i=0;
        for (auto id:msg->global_id) {
            position[id] = msg->encoder0_pos[i];
            tendon_length[id] = l_offset[id] - position[id];
            i++;
        }

//        roboy_simulation_msgs::Tendon tendon_msg;
//        for (int j=0; j < tendon_length.size(); j++){
//            tendon_msg.l.push_back(tendon_length[j]);
//        }
//        tendon_motor_pub.publish(tendon_msg);
    }

    void RoboyState(const roboy_middleware_msgs::RoboyState::ConstPtr &msg) {
//        prev_roboy_state_time = ros::Time::now();
    }

    void MotorInfo(const roboy_middleware_msgs::MotorInfo::ConstPtr &msg){
        for (int i=0;i<msg->global_id.size();i++) {
            auto id = int(msg->global_id[i]);
            auto body_part = findBodyPartByMotorId(id);
//            ROS_INFO_STREAM(body_part);
            if (body_part != "unknown") {
                if (msg->communication_quality[i] > 0 ) {
                    motor_status_received[body_part] = true;
                    //            communication_established[id] = true;
                }
                else {
                    if (body_part != "wrist_left" && body_part != "wrist_right")
                    {
                    //            communication_established[id] = false;
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(10),"Did not receive motor status for motor with id: %d. %s Body part is disabled.", (id, body_part));

                        // TODO fix triceps
                        //if (id != 18 && body_part != "shoulder_right") {
                        if(init_called[body_part]) {
                            init_called[body_part] = false;
                            node_->setParam("initialized", init_called);

                            // set respecitve body part joint targets to 0
                            string endeffector;
                            node_->getParam(body_part+"/endeffector", endeffector);
                            if ( !endeffector.empty() ) {
                                vector<string> ik_joints;
                                nh->getParam((endeffector + "/joints"), ik_joints);
                                if (ik_joints.empty()) {
                                    RCLCPP_ERROR(rclcpp::get_logger(),
                                            "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                                            body_part.c_str());
                                }
                                else {

                                    for (auto joint: ik_joints) {
                                        int joint_index = kinematics.GetJointIdByName(joint);
                                        if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                                            q_target(joint_index) = 0;
                                            RCLCPP_WARN_STREAM(rclcpp::get_logger(),"Set target 0 for " << joint);
                                        }
                                    }

                                }
                            }

                        }                            
                    }
                }
            }

        }
//      int i=0;
//      for (auto id:msg->global_id) {
//           ROS_INFO_STREAM("ID: " << id);
//        auto body_part = findBodyPartByMotorId(id);
//        if (msg->communication_quality[i] > 0 && body_part != "unknown") {
//            motor_status_received[body_part] = true;
////            communication_established[id] = true;
//        }
//        else {
////            communication_established[id] = false;
//            ROS_WARN_THROTTLE(1, "Did not receive %s's motor status for motor with id: %d. Body part is disabled.", (body_part.c_str(), id));
//            init_called[body_part] = false;
//        }
//        i++;
//      }
    }


    /**
     * Updates the robot model
     */
    void read(){
        update();
    };
    /**
     * Sends motor commands to the real robot
     */
    void write(){
        // check if plexus is alive
        auto diff = ros::Time::now() - prev_roboy_state_time;
        if (diff.toSec() > 1) {
            for (auto body_part: body_parts) {
                init_called[body_part] = false;
                nh->setParam("initialized", init_called);
                // reset the joint targets
                q_target.setZero();
            }
             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(5), "No messages from roboy_plexus. Will not be sending MotorCommand...");
            return;
        }

        for (auto body_part: body_parts) {
            if (!init_called[body_part]) {
                 RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), std::chrono::seconds(10), body_part << " was not initialized. skipping");
            } else {
//             if(body_part == "shoulder_right"){
                std::vector<int> motor_ids;
                try {
                    node_->getParam(body_part+"/motor_ids", motor_ids); }
                catch (const std::exception&) {
                    RCLCPP_ERROR(rclcpp::get_logger(),"motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part);
                }

                stringstream str;
                roboy_middleware_msgs::MotorCommand msg;
                msg.global_id = {};
                msg.setpoint = {};

                for (int i = 0; i < motor_ids.size(); i++) {
                    msg.global_id.push_back(motor_ids[i]);
                    auto setpoint = -l_next[motor_ids[i]] + l_offset[motor_ids[i]];
                    msg.setpoint.push_back(setpoint);
                }
                motor_command.publish(msg);
                if(!str.str().empty())
                    RCLCPP_WARN_STREAM(rclcpp::get_logger(),str.str());
            }
        }
    };

};

/**
 * controller manager update thread. Here you can define how fast your controllers should run
 * @param cm pointer to the controller manager
 */
void update(controller_manager::ControllerManager *cm) {
    //ros::Time prev_time = ros::Time::now();
    rclcpp::Time prev_time;
    prev_time = = rclcpp::Clock().now();
    rclcpp::Rate rate(500); // changing this value affects the control speed of your running controllers
    while (rclcpp::ok()) {
        const rclcpp::Time time = rclcpp::Clock::now();
        const rclcpp::Duration period = time - prev_time;
        cm->update(time, period);
        prev_time = time;
        rate.sleep();
    }
}

int main(int argc, char *argv[]) {

    string robot_model(argv[1]);
    bool debug(argv[2]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "launching " << robot_model);

    auto node = std::make_shared<rclcpp::Node>(robot_model + "_upper_body");

    // not need in ROS2
    // if (!ros::isInitialized()) {
    //     int argc = 0;
    //     char **argv = NULL;
    //     ros::init(argc, argv, robot_model + "_upper_body");
    // }
    //ros::NodeHandle nh;

    string urdf, cardsflow_xml;

    // if(nh.hasParam("urdf_file_path") && nh.hasParam("cardsflow_xml")) {
    //     nh.getParam("urdf_file_path", urdf);
    //     nh.getParam("cardsflow_xml", cardsflow_xml);
    // }else {
    //     ROS_FATAL("USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
    //     return 1;
    // }
    // ROS_INFO("\nurdf file path: %s\ncardsflow_xml %s", urdf.c_str(), cardsflow_xml.c_str());

    if(node->has_parameter("urdf_file_path") && node->has_parameter("cardsflow_xml")) {
        node->get_parameter("urdf_file_path", urdf);
        node->get_parameter("cardsflow_xml", cardsflow_xml);
    } else {
        RCLCPP_FATAL(rclcpp::get_logger(), "USAGE: ros2 run kindyn test_robot path_to_urdf path_to_viapoints_xml");
        retrun 1;
    }
    RCLCPP_INFO(rclcpp::get_logger(), "\nurdf file path: %s\ncardsflow_xml: %s", urdf.c_str(), cardsflow_xml.c_str());
    UpperBody robot(urdf, cardsflow_xml,robot_model, debug);
    controller_manager::ControllerManager cm(&robot);

    if (node->hasParam("simulated")) {
      node->getParam("simulated", robot.simulated);
    }

    thread update_thread(update, &cm);
    update_thread.detach();

    rclcpp::Rate rate(200);
    while(rclcpp::ok()){
        robot.read();
        if (!robot.simulated)
          robot.write();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    RCLCPP_INFO(rclcpp::get_logger(),"TERMINATING...");
    update_thread.join();

    return 0;  
  
  
  }


    



