#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#include <opencv2/highgui/highgui.hpp>

#include <CustomUnitreeArm.h>

class KeyboardController : public CustomUnitreeArm
{
public:
    KeyboardController(ros::NodeHandle &nh) : nh(nh) {};

    ~KeyboardController(){};

    char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
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

    void KeyboardControl()
    {
        tf::TransformListener tl;
        tf::StampedTransform transform;
        Eigen::Matrix<double, 6, 1> posture;
        Eigen::Matrix<double, 6, 1> postureDefault;

        postureDefault << 0.0, M_PI / 2, 0.0, 0.2, 0.0, 0.0;
        posture = postureDefault;
        MoveToRPYXYZ(posture.head(3), posture.tail(3), 0.25);

        std::cout << "please use the keyboard to control the arm" << std::endl;
        std::cout << "q/a: x; w/s: y; e/d: z; r/f: roll; t/g: pitch; y/h: yaw" << std::endl;
        while (1)
        {
            auto postureXYZ = posture.tail(3);
            auto postureRPY = posture.head(3);

            char c = getch();

            if (c == '0') {
                backToStart();
                continue;
            } else if (c == '1') {
                posture = postureDefault;
                MoveToRPYXYZ(posture.head(3), posture.tail(3), 0.25);
                continue;
            }

            // xyz
            if (c == 'q') {
                postureXYZ.x() -= 0.05; 
            } else if (c == 'a') {
                postureXYZ.x() += 0.05; 
            } else if (c == 'w') {
                postureXYZ.y() -= 0.05; 
            } else if (c == 's') {
                postureXYZ.y() += 0.05; 
            } else if (c == 'e') {
                postureXYZ.z() -= 0.05; 
            } else if (c == 'd') {
                postureXYZ.z() += 0.05; 
            } 
            // roll pitch yaw
            else if (c == 'r') {
                postureRPY.x() -= M_PI / 12; 
            } else if (c == 'f') {
                postureRPY.x() += M_PI / 12;
            } else if (c == 't') {
                postureRPY.y() -= M_PI / 12;
            } else if (c == 'g') {
                postureRPY.y() += M_PI / 12;
            } else if (c == 'y') {
                postureRPY.z() -= M_PI / 12;
            } else if (c == 'h') {
                postureRPY.z() += M_PI / 12;
            }

            MoveToRPYXYZ(postureRPY, postureXYZ);

        }
    }

private:
    ros::NodeHandle nh;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_control");
    ros::NodeHandle nh("~");

    KeyboardController armController(nh);
    armController.backToStart();

    // get arm join angles
    LoopFunc loop_JointState("printJointState", 0.1, boost::bind(&KeyboardController::PublishEndPointTF, &armController));
    loop_JointState.start();

    // keyboard
    armController.KeyboardControl();
    ros::spin();
}
