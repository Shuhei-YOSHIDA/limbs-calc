//use roboptim-capsule to calc collision-distance
#include <ros/ros.h>
#include "sample_util.h"
#include <RBDynUrdf/Reader.h> //github repository:jorisv/RBDynUrdf
#include <RBDyn/FK.h>

#include "problem_define.h"
#include "capsule_set.h"
#include "sch_set.h"
#include <random>

using namespace std;
using namespace rbdyn_urdf;
using namespace roboptim;

Urdf robotData;
//1. urdf to rbdyn
//2. urdf to optimized capsule
//3. rbdyn to collision of capsules

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
    visualization_msgs::MarkerArray allowmsg;
    
    ros::Publisher j_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Publisher a_pub = n.advertise<visualization_msgs::MarkerArray>("markers", 1);
    ros::Publisher a_pub2 = n.advertise<visualization_msgs::MarkerArray>("allows", 1);

    //==========================================
    // urdf to capsule (MESH in urdf is ignored)
    rbd::MultiBody mb = robotData.mbg.makeMultiBody("base_link", rbd::Joint::Fixed);
    rbd::MultiBodyConfig mbcin(mb);
    mbcin.zero(mb);
    rbd::forwardKinematics(mb, mbcin);
    rbd::forwardVelocity(mb, mbcin);

    std::map<std::string, std::vector<double>> capParam, capParamW;
    std::vector<double> distance;
    std::vector<Point3> p1, p2;
    //std::map<std::string, Point3> p1, p2;
    
    capsulesFromModel(urdf_string, capParam);
    calcColDistance(mb, mbcin, capParam, capParamW, distance, p1, p2);

    std::random_device rnd;
    std::mt19937 mt(rnd());
    std::uniform_int_distribution<int> ranj(0,100);
    int id = 0;
    for (auto&& var : capParamW) {
        visualization_msgs::Marker msg;
        Vector4d color(ranj(mt)/100., ranj(mt)/100., ranj(mt)/100., 0.6);
        capsuleMarker(point_t(var.second[0], var.second[1], var.second[2]), 
                      point_t(var.second[3], var.second[4], var.second[5]), 
                      var.second[6],id, amsg, color); 
        id += 3;
    }
    id = 0;
    for (int i = 0; i < p1.size(); i++) {
        visualization_msgs::Marker sph1, allow1, box2;
        point_t pd; pd << p1[i][0] - p2[i][0], p1[i][1] - p2[i][1], p1[i][2] - p2[i][2]; 
        if (pd == point_t(0,0,0)) pd << 1,0,0;
        Quaterniond q = Quaterniond::FromTwoVectors(point_t(1.,0.,0.), pd);
        std_msgs::Header header; header.frame_id = "base_link";
        sph1.header = allow1.header = box2.header = header;
        sph1.id = id; allow1.id = id+1; box2.id = id+2; id+=3;
        sph1.type = visualization_msgs::Marker::SPHERE;
        allow1.type = visualization_msgs::Marker::ARROW;
        box2.type = visualization_msgs::Marker::CUBE;
        sph1.action = allow1.action = box2.action = visualization_msgs::Marker::ADD;
        sph1.pose.position.x = p1[i][0]; sph1.pose.position.y = p1[i][1]; sph1.pose.position.z = p1[i][2]; 
        allow1.pose.position.x = p2[i][0]; allow1.pose.position.y = p2[i][1]; allow1.pose.position.z = p2[i][2]; 
        box2.pose.position.x = p2[i][0]; box2.pose.position.y = p2[i][1]; box2.pose.position.z = p2[i][2]; 
        sph1.pose.orientation.w = q.w();
        sph1.pose.orientation.x = q.x();
        sph1.pose.orientation.y = q.y();
        sph1.pose.orientation.z = q.z();
        allow1.pose.orientation = box2.pose.orientation = sph1.pose.orientation;
        sph1.scale.x = sph1.scale.y = sph1.scale.z = 0.015;
        box2.scale.x = box2.scale.y = box2.scale.z = 0.010;
        allow1.scale.x = pd.norm(); allow1.scale.y = allow1.scale.z = 0.005;
        std_msgs::ColorRGBA color;
        color.r = ranj(mt)/100.; color.g = ranj(mt)/100.; color.b = ranj(mt)/100.; 
        color.a = 1.0;
        sph1.color = allow1.color = box2.color = color;
        allowmsg.markers.push_back(sph1);
        allowmsg.markers.push_back(allow1);
        allowmsg.markers.push_back(box2);
    }

    //==========================================
    // capsule-test
    //test1(amsg);

    //==========================================
    // capsule-test2
    //test2(amsg, allowmsg);

    while (ros::ok()) {
        auto stamp = ros::Time::now();
        for (auto&& var : amsg.markers) {
            var.header.stamp = stamp;
        }
        for (auto&& var : allowmsg.markers) {
            var.header.stamp = stamp;
        }
        a_pub.publish(amsg);
        a_pub2.publish(allowmsg);
        ros::Duration(1.0).sleep();
    }

   
    return 0;
}
