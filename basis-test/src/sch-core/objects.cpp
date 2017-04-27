#include "ros/ros.h"

#include <sch/S_Object/S_Superellipsoid.h>
#include <sch/S_Object/S_Sphere.h>
#include <sch/CD/CD_Scene.h>

using namespace sch;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "objects");
    ros::NodeHandle n;

    sch::CD_Scene sObj;

    // radius 1.0 sphere
    S_Superellipsoid *super1 = new S_Superellipsoid(1.0, 1.0, 1.0, 1, 1);
    S_Superellipsoid *super2 = new S_Superellipsoid(1.0, 1.0, 1.0, 1, 1);
    S_Superellipsoid *super3 = new S_Superellipsoid(1.0, 1.0, 1.0, 1, 1);
    //S_Sphere * super1 = new S_Sphere(1.0);
    //S_Sphere * super2 = new S_Sphere(1.0);
    //S_Sphere * super3 = new S_Sphere(1.0);


    sch::Quaternion q = Quaternion(0.0,0.0,0.,1.);
    super1->setPosition(0.0, 0.0, 0.0);
    //super1->setOrientation(q);
    super1->setOrientation(0,0,0);
    super2->setPosition(3.0, 0.0, 0.0);
    //super2->setOrientation(q);
    super2->setOrientation(0,0,0);
    super3->setPosition(-2.001, 0.0, 0.0);
    //super3->setOrientation(q);
    super3->setOrientation(0,0,0);

    sObj.addObject(super1);
    sObj.addObject(super2);
    sObj.addObject(super3);

    std::vector<std::string> objName;
    objName.push_back("super1");
    objName.push_back("super2");
    objName.push_back("super3");

    int collisionNbr = sObj.sceneProximityQuery();
    std::cout << "collisionNbr " << collisionNbr << std::endl;

    for (int i = 0; i < sObj.size(); i++) {
        for (int j = 0; j < i; j++) {
            Point3 p1, p2;
            //Warning: this does not recompute the distance, even if one object
            //moved after the scene proximity query. 
            sObj.getWitnessPoints(i, j, p1, p2);
            std::cout << objName[i] << ", " << i << ", " << p1 << ", " << objName[j] << ", " << j << ", " << p2 << std::endl;

            //This will recompute the distance if the objects moved.
            CD_Pair *ptr = sObj(i, j);
            //ptr->getDistanceWithoutPenetrationDepth();
            Point3 p1_, p2_;
            double dis = ptr->getClosestPoints(p1_, p2_);
            std::cout << objName[i] <<", " << i << ", " << p1_ << ", " << objName[j] << ", " << j << ", " << p2_ << ", dis " << dis << std::endl;
        }
    }

    std::cout << "directly use pair " << std::endl;
    CD_Pair pair1(super1, super2);
    CD_Pair pair2(super1, super3);
    CD_Pair pair3(super2, super3);

    Point3 p11, p12;
    std::cout << "dis 1 " << pair1.getDistance() << std::endl;
    pair1.getClosestPoints(p11, p12);
    std::cout << p11 << ", " << p12 << std::endl;

    Point3 p21, p22;
    std::cout << "dis 2 " << pair2.getDistance() << std::endl;
    pair1.getClosestPoints(p21, p22);
    std::cout << p21 << ", " << p22 << std::endl;

    Point3 p31, p32;
    std::cout << "dis 3 " << pair3.getDistance() << std::endl;
    pair1.getClosestPoints(p32, p32);
    std::cout << p31 << ", " << p32 << std::endl;

    std::cout << "end " << std::endl;
    delete super1;
    delete super2;
    delete super3;
    return 0;
}

//Warning
/*
 * Pay attention to witness point. 
 * Check which geometry each point belongs to. 
 * And, if two geometry contacts just one position with mathmatical view, 
 * GJK algorithm may failed. 
 */
