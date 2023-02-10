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

#include "CustomUnitreeArm.h"


class CamArmCalibration : public CustomUnitreeArm
{
public:
    CamArmCalibration(ros::NodeHandle &nh) : nh(nh) {};
    ~CamArmCalibration(){};

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

    void SetCalibTargetPos(double x, double y, double z)
    {
        calibTargetPos << x, y, z;
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

    void Calibrate()
    {
        backToStart();
        sleep(1);

        tf::TransformListener tl;
        tf::StampedTransform transform;

        sensor_msgs::ImageConstPtr imagePtr;
        Eigen::Matrix<double, 6, 1> posture;
        Eigen::Matrix<double, 6, 1> postureDefault;

        postureDefault << 0.0, 0.0, 0.0, 0.5, 0.0, 0.2;
        posture = postureDefault;
        MoveToRPYXYZ(posture.head(3), posture.tail(3), 0.25);

        std::cout << "please use the keyboard to control the arm" << std::endl;
        std::cout << "q/a: x; w/s: y; e/d: z" << std::endl;
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
            } else if (c == '2') {
                ros::Time timeNow = ros::Time::now();
                tl.waitForTransform("link00", "arm_end_point", timeNow, ros::Duration(3.0));
                tl.lookupTransform("link00", "arm_end_point", timeNow, transform);

                const tf::Vector3 t = transform.getOrigin();
                const tf::Quaternion q = transform.getRotation();
                imagePtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/image_raw", nh);
                const auto cvImage = cv_bridge::toCvShare(imagePtr, "bgr8")->image;
                std::string imageName = std::to_string(q.w()) + '_' + 
                                        std::to_string(q.x()) + '_' +
                                        std::to_string(q.y()) + '_' +
                                        std::to_string(q.z()) + '_' +
                                        std::to_string(t.x()) + '_' +
                                        std::to_string(t.y()) + '_' +
                                        std::to_string(t.z()) + ".png";
                cv::imwrite(string(getenv("HOME")) + "/Pictures/" + imageName, cvImage);
                std::cout << "image saved to " << string(getenv("HOME")) + "/Pictures/" << imageName << std::endl;

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

        /*
        for (int i = 0; i < 5; ++i)
        {
            for (int j = -2; j < 3; ++j)
            {
                Eigen::Vector3d postureXYZ(0.3 + i * 0.05, j * 0.1, 0.1);
                auto rot = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), calibTargetPos - postureXYZ);
                Eigen::Vector3d postureRPY = rot.toRotationMatrix().eulerAngles(2, 1, 0);

                bool succeed = MoveToRPYXYZ(postureRPY, postureXYZ);
                if (succeed)
                {
                    // capture image
                    sleep(5);
                    ros::Time timeNow = ros::Time::now();
                    tl.waitForTransform("link00", "arm_end_point", timeNow, ros::Duration(3.0));
                    tl.lookupTransform("link00", "arm_end_point", timeNow, transform);

                    const tf::Vector3 t = transform.getOrigin();
                    const tf::Quaternion q = transform.getRotation();
                    imagePtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/image_raw", nh);
                    const auto cvImage = cv_bridge::toCvShare(imagePtr, "bgr8")->image;
                    std::string imageName = std::to_string(q.w()) + '_' + 
                                            std::to_string(q.x()) + '_' +
                                            std::to_string(q.y()) + '_' +
                                            std::to_string(q.z()) + '_' +
                                            std::to_string(t.x()) + '_' +
                                            std::to_string(t.y()) + '_' +
                                            std::to_string(t.z()) + ".png";
                    cv::imwrite(string(getenv("HOME")) + "/Pictures/" + imageName, cvImage);
                    std::cout << "image saved to " << string(getenv("HOME")) + "/Pictures/" << imageName << std::endl;
                }
            }
        }

        // z = -0.2
        for (int i = 0; i < 5; ++i)
        {
            for (int j = -2; j < 3; ++j)
            {
                Eigen::Vector3d postureXYZ(0.3 + i * 0.05, j * 0.1, 0.3);
                auto rot = Eigen::Quaterniond().setFromTwoVectors(Eigen::Vector3d::UnitX(), calibTargetPos - postureXYZ);
                Eigen::Vector3d postureRPY = rot.toRotationMatrix().eulerAngles(2, 1, 0);

                bool succeed = MoveToRPYXYZ(postureRPY, postureXYZ);
                if (succeed)
                {
                    sleep(5);
                    // capture image
                    ros::Time timeNow = ros::Time::now();
                    tl.waitForTransform("link00", "arm_end_point", timeNow, ros::Duration(3.0));
                    tl.lookupTransform("link00", "arm_end_point", timeNow, transform);

                    const tf::Vector3 t = transform.getOrigin();
                    const tf::Quaternion q = transform.getRotation();
                    imagePtr = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/image_raw", nh);
                    const auto cvImage = cv_bridge::toCvShare(imagePtr, "bgr8")->image;
                    std::string imageName = std::to_string(q.w()) + '_' + 
                                            std::to_string(q.x()) + '_' +
                                            std::to_string(q.y()) + '_' +
                                            std::to_string(q.z()) + '_' +
                                            std::to_string(t.x()) + '_' +
                                            std::to_string(t.y()) + '_' +
                                            std::to_string(t.z()) + ".png";
                    cv::imwrite(string(getenv("HOME")) + "/Pictures/" + imageName, cvImage);
                    std::cout << "image saved to " << string(getenv("HOME")) + "/Pictures/" << imageName << std::endl;
                }
            }
        }

        backToStart();
        */
    }

private:
    ros::NodeHandle nh;
    Eigen::Vector3d calibTargetPos;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_control");
    ros::NodeHandle nh("~");

    CamArmCalibration armController(nh);

    std::string calib_target_pos_param;
    if (nh.getParam("calib_target_pos", calib_target_pos_param))
    {
        const char delim = ' ';
        std::vector<std::string> split;
        tokenize(calib_target_pos_param, delim, split);
        assert(split.size() == 3);
        armController.SetCalibTargetPos(stod(split[0]), stod(split[1]), stod(split[2]));
    }

    armController.backToStart();

    // get arm join angles
    LoopFunc loop_printJointState("printJointState", 0.1, boost::bind(&CamArmCalibration::PublishEndPointTF, &armController));
    loop_printJointState.start();

    // calibrate
    armController.Calibrate();

    ros::spin();
}
