#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#include "CustomUnitreeArm.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>

template<typename TYPE>
struct CubicVolume {
    std::vector<TYPE> values_;
    Eigen::Vector3d volumeOrigin_;
    double volumeResolution_;
    Eigen::Vector3i volumeSize_;

    CubicVolume(const Eigen::Vector3i& volumeSize, const Eigen::Vector3d& volumeOrigin, double volumeResolution)
    {
        volumeOrigin_ = volumeOrigin;
        volumeResolution_ = volumeResolution;
        volumeSize_ = volumeSize;
        values_.resize(volumeSize.x() * volumeSize.y() * volumeSize.z());
    }

    TYPE& at(int i, int j, int k)
    {
        assert (i >= 0 && i < volumeSize_.x());
        assert (j >= 0 && j < volumeSize_.y());
        assert (k >= 0 && k < volumeSize_.z());
        return values_[i * volumeSize_[1] * volumeSize_[2] + j * volumeSize_[2] + k];
    }

    const TYPE& at(int i, int j, int k) const 
    {
        assert (i >= 0 && i < volumeSize_.x());
        assert (j >= 0 && j < volumeSize_.y());
        assert (k >= 0 && k < volumeSize_.z());
        return values_[i * volumeSize_[1] * volumeSize_[2] + j * volumeSize_[2] + k];
    }

    Eigen::Vector3d indexToPos(int i, int j, int k) const
    {
        return volumeOrigin_ + Eigen::Vector3d(i * volumeResolution_, j * volumeResolution_, k * volumeResolution_);
    }

    Eigen::Vector3i getSize() const
    {
        return volumeSize_;
    }
};

void getPointFromReachability(const CubicVolume<int>& v, sensor_msgs::PointCloud2& cloudMsg)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    Eigen::Vector3i size = v.getSize();

    for (int i = 0; i < size.x(); ++i)
    {
        for (int j = 0; j < size.y(); ++j)
        {
            for (int k = 0; k < size.z(); ++k)
            {
                if (v.at(i, j, k) == 0) continue;

                Eigen::Vector3d pos = v.indexToPos(i, j, k);
                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = pos(2);
                pt.intensity = v.at(i, j, k);
                cloud.push_back(pt);
            }
        }
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = "link00";

    pcl::toROSMsg(cloud, cloudMsg);
}

void saveToFile(std::string fileName, const CubicVolume<int>& v)
{
    std::ofstream file;
    file.open(fileName);
  
    Eigen::Vector3i size = v.getSize();
    for (int i = 0; i < size.x(); ++i)
    {
        for (int j = 0; j < size.y(); ++j)
        {
            for (int k = 0; k < size.z(); ++k)
            {
                if (v.at(i, j, k) == 0) continue;

                Eigen::Vector3d pos = v.indexToPos(i, j, k);
                file << pos.x() << ' ' << pos.y() << ' ' << pos.z() << std::endl;
            }
        }
    }

    file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reachability");
    ros::NodeHandle nh;
    ros::Publisher reachabilityPub = nh.advertise<sensor_msgs::PointCloud2>("reachability", 10);

    CustomUnitreeArm arm;

    // get tf
    tf::TransformListener listener;
    tf::StampedTransform transform;
    Eigen::Isometry3d arm_end_T_torch;
    if (listener.waitForTransform("arm_end_point", "torch", ros::Time(0), ros::Duration(1.0)))
    {
        listener.lookupTransform("arm_end_point", "torch", ros::Time(0), transform);
    } 
    else
    {
        ROS_ERROR("transformation between torch and arm end point is not available");
        return 1;
    }
    arm_end_T_torch.translation() = Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    arm_end_T_torch.linear() = 
        Eigen::Quaterniond(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z()).toRotationMatrix();

    // construct the rotation list
    std::vector<Eigen::Matrix3d> rotationList;
    Eigen::Matrix3d rotMatrix = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix() * 
                                Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();;
    rotationList.push_back(rotMatrix);

    // loop over each voxel of the target area
    Eigen::Vector3d volumeOrigin(0, -0.5, -1.0);
    double volumeResolution = 0.01;
    Eigen::Vector3i volumeSize(100, 100, 100);
    CubicVolume<int> v(volumeSize, volumeOrigin, volumeResolution);
    Eigen::VectorXd jointAngles(6);
    for (int i = 0; i < volumeSize(0); ++i)
    {
        std::cout << "calculating" << std::endl;
        for (int j = 0; j < volumeSize(1); ++j)
        {
            for (int k = 0; k < volumeSize(2); ++k)
            {
                Eigen::Vector3d pos = v.indexToPos(i, j, k);
                for (const auto& rot : rotationList)
                {
                    Eigen::Isometry3d base_T_torch, base_T_arm_end;
                    base_T_torch.linear() = rot;
                    base_T_torch.translation() = pos;
                    base_T_arm_end = base_T_torch * arm_end_T_torch.inverse();

                    bool succeed = arm.InverseKinematicsIKFast(base_T_arm_end.translation(), base_T_arm_end.linear(), jointAngles);
                    if (succeed)
                    {
                        v.at(i, j, k)++;
                    }
                }
            }
        }
    }

    ros::Rate rate(10);
    sensor_msgs::PointCloud2 cloudMsg;
    getPointFromReachability(v, cloudMsg);

    saveToFile("./reachability.txt", v);
    std::cout << "Rechability map saved to ./reachability.txt" << std::endl;
    std::cout << "Open RVIZ to visulize the reachability" << std::endl;
    while (ros::ok())
    {
        reachabilityPub.publish(cloudMsg);
        rate.sleep();
    }
}
