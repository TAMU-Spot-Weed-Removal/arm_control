#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#include <CustomUnitreeArm.h>

double WEED_OBSERVATION_HEIGHT = 0;  // relative to the arm base

class ArmController : public CustomUnitreeArm
{
public:
    ArmController(ros::NodeHandle &nh) : nh(nh) {
        scene_observation_sub = nh.subscribe("/scene_observation_pose", 1, &ArmController::SceneObservationCallback, this);
        weed_position_sub = nh.subscribe("/weed_position", 1, &ArmController::WeedObservationCallback, this);
        weed_actuation_sub = nh.subscribe("/actuation_position", 1, &ArmController::WeedingActuationCallback, this);
        BackToStartCallback_sub = nh.subscribe("/back_to_start", 1, &ArmController::BackToStartCallback, this);

        scene_observation_status_pub = nh.advertise<std_msgs::UInt8>("scene_observation_status", 1);
        weed_observation_status_pub = nh.advertise<std_msgs::UInt8>("weed_observation_status", 1);
        weed_actuation_status_pub = nh.advertise<std_msgs::UInt8>("weed_actuation_status", 1);
        BackToStartCallback_pub = nh.advertise<std_msgs::UInt8>("back_to_start_status", 1);

        torch_control_pub = nh.advertise<std_msgs::String>("torch_command", 1);
    };

    ~ArmController(){};

    void SceneObservationCallback(geometry_msgs::Point32ConstPtr look_at_position)
    {
        // move camera by finding a suitable position
        bool succeed= false;
        Eigen::Vector3d target_pos(look_at_position->x, look_at_position->y, look_at_position->z);
        for (double x = 0.2; x <= 0.4; x += 0.01)
        {   
            for (double z = -0.2; z <= 0.2; z += 0.01)
            {   
                Eigen::Isometry3d base_T_cam;
                base_T_cam.translation() = Eigen::Vector3d(x, 0, z);
                Eigen::Vector3d x_axis, y_axis, z_axis;
                z_axis = (target_pos - base_T_cam.translation()).normalized();
                x_axis = z_axis.cross(Eigen::Vector3d::UnitZ()).normalized();
                y_axis = z_axis.cross(x_axis).normalized();
                base_T_cam.linear().block(0, 0, 3, 1) = x_axis;
                base_T_cam.linear().block(0, 1, 3, 1) = y_axis;
                base_T_cam.linear().block(0, 2, 3, 1) = z_axis;
            
                // get arm endpoint's position and rotation
                Eigen::Isometry3d base_T_arm_end = base_T_cam * arm_end_T_cam.inverse();

                Eigen::VectorXd jointAngles = Eigen::VectorXd::Zero(6);
                if(InverseKinematicsIKFast(base_T_arm_end.translation(), base_T_arm_end.linear(), jointAngles))
                {
                    succeed = MoveToPosRot(base_T_arm_end.translation(), base_T_arm_end.linear());
                }

                if (succeed)
                {
                    break;
                }
            }

            if (succeed)
            {
                break;
            }
        }

        std_msgs::UInt8 status;
        if (succeed)
        {
            status.data = 0;  // successfully moved to the observation position
        } 
        else 
        {
            ROS_WARN("no feasible solution found for scene observation");
            status.data = 1;  // the position is not reachable
        }
        scene_observation_status_pub.publish(status);
    }

    void WeedObservationCallback(geometry_msgs::Point32ConstPtr weed_position)
    {
        // get camera's position and rotation
        Eigen::Isometry3d base_T_cam;
        base_T_cam.translation() = Eigen::Vector3d(weed_position->x, weed_position->y, WEED_OBSERVATION_HEIGHT);  // observe from above the weed
        base_T_cam.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix() * 
                              Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // get arm endpoint's position and rotation
        Eigen::Isometry3d base_T_arm_end = base_T_cam * arm_end_T_cam.inverse();

        bool succeed = MoveToPosRot(base_T_arm_end.translation(), base_T_arm_end.linear());

        std_msgs::UInt8 status;
        if (succeed)
        {
            status.data = 0;  // successfully moved to the observation position
        } 
        else 
        {
            ROS_WARN("no feasible solution found for weed observation");
            status.data = 1;  // the position is not reachable
        }
        weed_observation_status_pub.publish(status);
    }

    void WeedingActuationCallback(geometry_msgs::Point32ConstPtr weed_position)
    {
        std_msgs::UInt8 status;

        // get torch's position and rotation
        Eigen::Isometry3d base_T_torch;
        base_T_torch.translation() = Eigen::Vector3d(weed_position->x, weed_position->y, weed_position->z);  // observe from above the weed
        base_T_torch.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix() * 
                                Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // get arm endpoint's position and rotation
        Eigen::Isometry3d base_T_arm_end = base_T_torch * arm_end_T_torch.inverse();

        bool succeed = MoveToPosRot(base_T_arm_end.translation(), base_T_arm_end.linear());

        if (!succeed)
        {
            ROS_WARN("no feasible solution found for weeding actuation");
            status.data = 1;  // the position is not reachable
            weed_actuation_status_pub.publish(status);
            return;
        } 

        std_msgs::String torch_command;
        torch_command.data = "fire on";
        torch_control_pub.publish(torch_command);
        ros::Duration(10.0).sleep();
        torch_command.data = "fire off";
        torch_control_pub.publish(torch_command);

        status.data = 0;  // succeed
        weed_actuation_status_pub.publish(status);
    }

    void BackToStartCallback(geometry_msgs::Point32ConstPtr weed_position)
    {
        this->backToStart();
        std_msgs::UInt8 status;
        BackToStartCallback_pub.publish(status);
    }

    void PublishEndPointTF()
    {
        Eigen::Matrix3d endPointRot;
        Eigen::Vector3d endPointPos;
        Eigen::VectorXd jointAngles = Eigen::VectorXd::Zero(6);
        GetJointAngles(jointAngles);

        // get the rotation and translation of the arm's end point 
        ForwardKinematics(jointAngles, endPointPos, endPointRot);

        // publish tf
        Eigen::Quaterniond q(endPointRot);
        static tf::TransformBroadcaster tb;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(endPointPos.x(), endPointPos.y(), endPointPos.z()));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        tb.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "link00", "arm_end_point"));
    }


private:
    ros::NodeHandle nh;

    ros::Subscriber scene_observation_sub;
    ros::Subscriber weed_position_sub;
    ros::Subscriber weed_actuation_sub;
    ros::Subscriber BackToStartCallback_sub;

    ros::Publisher scene_observation_status_pub;
    ros::Publisher weed_observation_status_pub;
    ros::Publisher weed_actuation_status_pub;
    ros::Publisher BackToStartCallback_pub;

    ros::Publisher torch_control_pub;

public:
    Eigen::Isometry3d arm_end_T_cam;
    Eigen::Isometry3d arm_end_T_torch;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh;

    ArmController armController(nh);
    armController.backToStart();

    // get arm join angles
    LoopFunc loop_JointState("printJointState", 0.1, boost::bind(&ArmController::PublishEndPointTF, &armController));
    loop_JointState.start();

    // get tf
    tf::TransformListener listener;
    tf::StampedTransform transform;
    if (listener.waitForTransform("arm_end_point", "camera", ros::Time(0), ros::Duration(1.0)))
    {
        listener.lookupTransform("arm_end_point", "camera", ros::Time(0), transform);
    } 
    else
    {
        ROS_ERROR("transformation between camera and arm end point is not available");
        return 1;
    }
    armController.arm_end_T_cam.translation() = Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    armController.arm_end_T_cam.linear() = 
        Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z()).toRotationMatrix();

    if (listener.waitForTransform("arm_end_point", "torch", ros::Time(0), ros::Duration(1.0)))
    {
        listener.lookupTransform("arm_end_point", "torch", ros::Time(0), transform);
    } 
    else
    {
        ROS_ERROR("transformation between torch and arm end point is not available");
        return 1;
    }
    armController.arm_end_T_torch.translation() = Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    armController.arm_end_T_torch.linear() = 
        Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z()).toRotationMatrix();

    ros::spin();
}
