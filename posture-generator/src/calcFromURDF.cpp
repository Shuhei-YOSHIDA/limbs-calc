#include <ros/ros.h>
#include <urdf/model.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>
#include <RBDyn/FD.h>
#include <RBDyn/CoM.h>

using namespace urdf;
Model model;

bool readUrdf()
{
    if (!model.initParam("robot_description")) {
        return false;
    }
    //TODO automatically read graph from urdf::Model
    return true;
}

void treeParse(LinkConstSharedPtr link, int level = 0)
{
    level+=2;//for indent
    int count = 0;
    for (std::vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++) {
        if (*child) {
            for (int j=0;j<level;j++) std::cout <<"  ";//indent
            std::cout << "child(" << (count++)+1 << "): " << (*child)->name << std::endl;
            // next generation
            treeParse(*child, level);
        }
        else { // In case of end link, this part does not processed?
            for (int j=0;j<level;j++) std::cout <<"  ";//indent
            std::cout << "root link: " << link->name << "has no child " << *child << std::endl;
        }
    }

}

rbd::MultiBodyGraph mbd;
bool setMultiBodyGraph()
{
    //TODO automatically read graph from urdf::Model
    
    
    return true;  
}










int main (int argc, char** argv)
{
    ros::init(argc, argv, "multiLinkCalc");
    ros::NodeHandle n;
    
    readUrdf();
    LinkConstSharedPtr root_link = model.getRoot();
    treeParse(root_link);

    ros::spin();

    return 0;
}
