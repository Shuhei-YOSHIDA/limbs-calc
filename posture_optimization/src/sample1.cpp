#include <ros/ros.h>
#include "sample_util.h"
#include <RBDynUrdf/Reader.h> //github repository:jorisv/RBDynUrdf
#include <RBDyn/FK.h>
#include "problem_define.h"

using namespace std;
using namespace rbdyn_urdf;

Urdf robotData;

void optim()
{

}

void sample1(sensor_msgs::JointState &jmsg, visualization_msgs::MarkerArray &amsg)
{
    rbd::MultiBody mb = robotData.mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbc(mb);
    mbc.zero(mb);



    
    rbd::forwardKinematics(mb, mbc);
    JointStateFromMBC(mb, mbc, jmsg);
    MarkerArrayFromMBC(mb, mbc, amsg);
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sample1");
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

    while(ros::ok()) {
        sample1(jmsg, amsg);
        j_pub.publish(jmsg);
        a_pub.publish(amsg);

        ros::Duration(1.0).sleep();
    }
    
    return 0;
}
