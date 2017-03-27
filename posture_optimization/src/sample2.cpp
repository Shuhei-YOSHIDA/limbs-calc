//use roboptim-capsule to calc collision-distance
#include <ros/ros.h>
#include "sample_util.h"
#include <RBDynUrdf/Reader.h> //github repository:jorisv/RBDynUrdf
#include <RBDyn/FK.h>
#include "problem_define.h"
#include "capsule_set.h"
#include <random>

using namespace std;
using namespace rbdyn_urdf;
using namespace roboptim;

Urdf robotData;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample2");
    ros::NodeHandle n;

    string urdf_string;
    if (n.getParam("/robot_description", urdf_string)) {
        robotData = readUrdf(urdf_string);
    }
    else {
        cerr << "urdf was not loaded from parameter " << endl;
        return -1;
    }

    sensor_msgs::JointState jmsg; 
    visualization_msgs::MarkerArray amsg;
    
    ros::Publisher j_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);

    rbd::MultiBody mb = robotData.mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbcin(mb);


    test1(amsg);
    while (ros::ok()) {
        a_pub.publish(amsg);
        auto stamp = ros::Time::now();
        for (auto&& var : amsg.markers) {
            var.header.stamp = stamp;
        }
        ros::Duration(1.0).sleep();
    }

   
    return 0;
}
