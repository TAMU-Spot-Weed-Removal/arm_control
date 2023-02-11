#ifndef __CUSTOMUNITREEARM_H
#define __CUSTOMUNITREEARM_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#include <unistd.h>
#include <termios.h>

#include <ikfast/ikfast_z1.h>

#include <control/unitreeArm.h>

const int NUM_JOINTS = 6;

const Eigen::Vector3d ROTATION_AXIS[NUM_JOINTS] = {
    Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitY(),
    Eigen::Vector3d::UnitZ(),
    Eigen::Vector3d::UnitX(),
};

const Eigen::Vector3d LINKS[NUM_JOINTS + 1] = {
    Eigen::Vector3d(0.0, 0.0, 0.065),    // between base and joint1
    Eigen::Vector3d(0.0, 0.0, 0.04),     // between joint1 and joint2
    Eigen::Vector3d(-0.35, 0.0, 0.0),     // between joint2 and joint3
    Eigen::Vector3d(0.218, 0.0, 0.057),     // between joint3 and joint4
    Eigen::Vector3d(0.072, 0.0, 0.0),     // between joint4 and joint5
    Eigen::Vector3d(0.1, 0.0, 0.0),     // between joint5 and joint6
    Eigen::Vector3d(0.0, 0.0, 0.0)     // between joint6 and end effector
};

const std::pair<double, double> JOINT_LIMITS[NUM_JOINTS] = {
    std::pair<double, double>(-150.0 / 180.0 * M_PI,	150.0 / 180.0 * M_PI),
    std::pair<double, double>(  0.0 / 180.0 * M_PI,	    180.0 / 180.0 * M_PI),
    std::pair<double, double>(-165.0 / 180.0 * M_PI,	  0.0 / 180.0 * M_PI),
    std::pair<double, double>( -80.0 / 180.0 * M_PI,	 80.0 / 180.0 * M_PI),
    std::pair<double, double>( -85.0 / 180.0 * M_PI,	 85.0 / 180.0 * M_PI),
    std::pair<double, double>(-160.0 / 180.0 * M_PI,	160.0 / 180.0 * M_PI)
};

const double ANGULAR_SPEED_LIMIT[NUM_JOINTS] = {0.5, 0.75, 1.0, 1.0, 1.0, 1.0};

void tokenize(std::string const &str, const char delim,
            std::vector<std::string> &out)
{
    std::stringstream ss(str);
 
    std::string s;
    while (std::getline(ss, s, delim)) {
        out.push_back(s);
    }
}

class CustomUnitreeArm : public unitreeArm
{
public:
    CustomUnitreeArm():unitreeArm(new CtrlComponents(0.002)){
        sendRecvThread->start();
    };

    CustomUnitreeArm(CtrlComponents * ctrlComp):unitreeArm(ctrlComp){
        sendRecvThread->start();
    };

    ~CustomUnitreeArm(){
        sendRecvThread->shutdown();
    };

    void GetJointAngles(Eigen::VectorXd &jointAngles)
    {
        // get joint angles
        jointAngles = _ctrlComp->lowstate->getQ();
    }

    void ForwardKinematics(
        const Eigen::VectorXd &jointAngles,
        Eigen::Vector3d &endPointPos,
        Eigen::Matrix3d &endPointRot) 
    {
        endPointPos = LINKS[NUM_JOINTS];
        endPointRot = Eigen::Matrix3d::Identity();
        for (int i = NUM_JOINTS - 1; i >= 0; --i)
        {
            auto jointRot = Eigen::AngleAxisd(jointAngles(i), ROTATION_AXIS[i]);
            endPointPos = jointRot * endPointPos + LINKS[i];
            endPointRot = jointRot * endPointRot;
        }
    }

    bool InverseKinematicsIKFast(
        const Eigen::Vector3d &endPointPos,
        const Eigen::Matrix3d &endPointRot, 
        Eigen::VectorXd &jointAngles)
    {
        IkReal eerot[9],eetrans[3];
        eerot[0] = endPointRot(0, 0);
        eerot[1] = endPointRot(0, 1);
        eerot[2] = endPointRot(0, 2);
        eerot[3] = endPointRot(1, 0);
        eerot[4] = endPointRot(1, 1);
        eerot[5] = endPointRot(1, 2);
        eerot[6] = endPointRot(2, 0);
        eerot[7] = endPointRot(2, 1);
        eerot[8] = endPointRot(2, 2);
        eetrans[0] = endPointPos(0);
        eetrans[1] = endPointPos(1);
        eetrans[2] = endPointPos(2);

        IkSolutionList<IkReal> solutions;
        bool succeed = ComputeIk(eetrans, eerot, nullptr, solutions);

        if(!succeed) {
            return false;
        }

        assert(NUM_JOINTS == GetNumJoints());
        std::vector<IkReal> solvalues(NUM_JOINTS);
        bool hasValidSolution = false;
        for (int i = 0; i < solutions.GetNumSolutions(); ++i) {
            const IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
            sol.GetSolution(&solvalues[0], nullptr);
            bool jointWithinLimit = true;
            for(int j = 0; j < NUM_JOINTS; ++j)
            {
                if (solvalues[j] < JOINT_LIMITS[j].first || solvalues[j] > JOINT_LIMITS[j].second)
                {
                    jointWithinLimit = false;
                    break;
                }
            }
            if (jointWithinLimit)
            {
                hasValidSolution = true;
                break;
            }
        }
        
        if (!hasValidSolution)
        {
            return false;
        }

        jointAngles << solvalues[0], solvalues[1], solvalues[2], solvalues[3], solvalues[4], solvalues[5];
        return true;
    }

    void MoveJoints(const Eigen::VectorXd &jointAngles, double speed = 1.0) 
    {
        startTrack(ArmFSMState::JOINTCTRL);
        Eigen::VectorXd qInit = _ctrlComp->lowstate->getQ();
        Eigen::VectorXd dq = jointAngles - qInit;

        // make sure the angular speed does not exceed the limit of each joint
        double run_time_with_curr_speed = std::numeric_limits<double>::min();
        double run_time_with_scaled_speed = std::numeric_limits<double>::min();
        double speed_scale_factor = 1;
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            run_time_with_curr_speed = std::max(run_time_with_curr_speed, std::abs(dq[i]) / speed);
        }
        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            double temp_speed = std::abs(dq[i]) / run_time_with_curr_speed;
            speed_scale_factor = std::max(speed_scale_factor, temp_speed / ANGULAR_SPEED_LIMIT[i]);
        }
        speed /= speed_scale_factor;
        if (speed_scale_factor > 1)
        {
            std::cout << "angular velocity scaled by a factor: " << speed_scale_factor << std::endl;
        }

        for (int i = 0; i < NUM_JOINTS; ++i)
        {
            run_time_with_scaled_speed = std::max(run_time_with_scaled_speed, std::abs(dq[i]) / speed);
        }
        Eigen::VectorXd speed_each_joint = dq / run_time_with_scaled_speed;

        unsigned long stepCount = 0;
        while (1) 
        {
            if ((_ctrlComp->lowstate->getQ() - jointAngles).norm() < 0.05)
            {
                break;
            }

            q = qInit + stepCount * speed_each_joint * _ctrlComp->dt;    // max speed 1 rad/s
            stepCount++;
            usleep(_ctrlComp->dt * 1000000);
        }

        _ctrlComp->sendCmd.track = false;
    }

    bool MoveToPosRot(
        const Eigen::Vector3d &endPointPos,
        const Eigen::Matrix3d &endPointRot,
        double speed = 1.0)
    {
        Eigen::VectorXd jointAngles = Eigen::VectorXd::Zero(6);
        bool succeed = InverseKinematicsIKFast(endPointPos, endPointRot, jointAngles);

        if (succeed) 
        {
            Vec6 posture;
            posture.head(3) = endPointRot.eulerAngles(2, 1, 0);
            posture.tail(3) = endPointPos;
            MoveJoints(jointAngles, speed);
            return true;
        } else {
            std::cout << "The target position is not reachable" << std::endl;
            return false;
        }
    }

    bool MoveToRPYXYZ(
        const Eigen::Vector3d &endPointRPY,
        const Eigen::Vector3d &endPointXYZ,
        double speed = 1.0)
    {
        Eigen::Matrix3d endPointRot = (Eigen::AngleAxisd(endPointRPY.z(), Eigen::Vector3d::UnitZ()) * 
                                       Eigen::AngleAxisd(endPointRPY.y(), Eigen::Vector3d::UnitY()) * 
                                       Eigen::AngleAxisd(endPointRPY.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        Eigen::Vector3d endPointPos = endPointXYZ;
        return MoveToPosRot(endPointPos, endPointRot, speed);
    }

};

#endif