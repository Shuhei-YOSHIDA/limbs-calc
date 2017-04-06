//urdf to optimized capsule
/*
 *This uses sch-core. 
 *FCL Library may have bug about Collision of Sphere to Box
 */
#pragma once

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
// -------------------------
// Includes for 3D objects
// First guess :urdf::geometry
// optimization :roboptim::capsule
// collision :sch-core::capsule
//#include <sch/S_Object/S_Sphere.h>
//#include <sch/S_Object/S_Box.h>
//#include <sch/S_Object/S_Superellipsoid.h>
#include <sch/S_Object/S_Capsule.h>
#include "capsule_set.h"

// -------------------------
// Includes for proximity queries
#include <sch/CD/CD_Pair.h>

// -------------------------
// Includes for RBDyn
#include <RBDyn/MultiBodyConfig.h>

using namespace sch;
using namespace Eigen;


// Read geometry info from urdf, and Generate 
void capsulesFromModel(const std::string& urdf,
                       //const boost::shared_ptr<urdf::ModelInterface>& model, 
                       //double capParam[7])
                       std::map<std::string, std::vector<double>> &capParams)
{
    const boost::shared_ptr<urdf::ModelInterface>& model = urdf::parseURDF(urdf);
    if(!model) {
        throw std::runtime_error("Urdf file is not valid");
    }
    capParams.clear();

    for(const auto& it: model->links_) {
        if (it.second->collision) {
            const urdf::Collision& colUrdf = *it.second->collision;

            // check geometry:Sphere, Box, Cylinder, Mesh
            double boxHalfEdge[3];
            std::string linkname = it.first;
            switch (colUrdf.geometry->type) {
                case urdf::Geometry::SPHERE:
                {
                    boost::shared_ptr<urdf::Sphere> ge = 
                        boost::dynamic_pointer_cast<urdf::Sphere>(colUrdf.geometry);
                    for (int i = 0; i < 3; i++)
                        boxHalfEdge[i] = ge->radius;
                }
                break;
                case urdf::Geometry::BOX:
                {
                    boost::shared_ptr<urdf::Box> ge = 
                        boost::dynamic_pointer_cast<urdf::Box>(colUrdf.geometry);
                    boxHalfEdge[0] = ge->dim.x/2.;
                    boxHalfEdge[1] = ge->dim.y/2.;
                    boxHalfEdge[2] = ge->dim.z/2.;
                }
                break;
                case urdf::Geometry::CYLINDER:
                {
                    boost::shared_ptr<urdf::Cylinder> ge = 
                        boost::dynamic_pointer_cast<urdf::Cylinder>(colUrdf.geometry);
                    boxHalfEdge[0] = ge->radius;
                    boxHalfEdge[1] = ge->radius;
                    boxHalfEdge[2] = ge->length/2.;
                }
                break;
                case urdf::Geometry::MESH:
                    std::cerr << "Please implement MESH in capsuleFromModel" << std::endl;
                    continue;
                break;
                default:
                    std::cerr << "can't generate capsule, unknown geometry type" << std::endl;
                    continue;
                break;
            }
            double param[7];
            fittingCapsule(boxHalfEdge, param);
            std::vector<double> tmparray(param, param+7);
            capParams[linkname] = tmparray;
        }
    }
}

// Calc Collision-distance from RBDyn::MultiBodyConfig and Optimized capsules
void calcColDistance(rbd::MultiBody mb, rbd::MultiBodyConfig mbc, 
                     std::map<std::string, std::vector<double>> capParams,
                     std::map<std::string, std::vector<double>> &capParamsW,
                     std::vector<double> &distance,
                     std::vector<Point3> &points1,
                     std::vector<Point3> &points2)
                     //std::map<std::string, Point3> &points1,
                     //std::map<std::string, Point3> &points2)
{
    capParamsW.clear();
    points1.clear();
    points2.clear();
    std::vector<S_Capsule> capsules;//Pay attention to move address
    std::vector<std::string> capsuleNames;
    
    int count = 0;
    for (auto itr = mbc.bodyPosW.begin(); itr < mbc.bodyPosW.end(); itr++) {
        std::string linkname = mb.body(count).name();
        count++;
        if (capParams[linkname].empty()) continue;//std::map<,double> default/ maybe ok
        Vector3d wp1 = itr->translation() + itr->rotation().transpose() * Vector3d(capParams[linkname][0], capParams[linkname][1], capParams[linkname][2]);
        Vector3d wp2 = itr->translation() + itr->rotation().transpose() * Vector3d(capParams[linkname][3], capParams[linkname][4], capParams[linkname][5]);
        capsules.push_back(S_Capsule(Point3(wp1(0),wp1(1),wp1(2)), Point3(wp2(0),wp2(1),wp2(2)), capParams[linkname][6]));
        capsuleNames.push_back(linkname);

        //resultant data
        std::vector<double> tmp = {wp1(0),wp1(1),wp1(2),wp2(0),wp2(1),wp2(2), capParams[linkname][6]};
        capParamsW[linkname] = tmp;
    }

    //make CD_Pair considering with never colliding pair
    int combCount = 0;
    if (capsuleNames.empty()) {
        std::cerr << "capsuleNames is empty!" << std::endl;
    }
    //combination
    for (int i = 0; i < capsuleNames.size()-1; i++) {
        int j = i;
        for (auto itr = capsuleNames.begin()+i+1; itr != capsuleNames.end()-1; itr++) {
            std::string first = capsuleNames[i];
            std::string second = *itr;
            CD_Pair pair(&capsules[i], &capsules[j]);// Usually, CD_Pair class uses previous infomation?
            int sign = 1;
            if (pair.getDistance() < 0) sign = -1;
            std::cout << "Distance " << sign*sqrt(fabs(pair.getDistance())) << ", " << first << ", " << second << std::endl;
            std::cout << "Collision " << (pair.isInCollision() ? "True ": "False ") << std::endl;
            Point3 p1,p2;
            pair.getClosestPoints(p1, p2);
            std::cout << "Witness points " << std::endl;
            std::cout << " P1: " << p1 << std::endl;
            std::cout << " P2: " << p2 << std::endl;
            if (pair.isInCollision()) {
                points1.push_back(p1);
                points2.push_back(p2);
                //points1[first] = p1;
                //points2[second] = p2;
            }

            j++;
        }
    }

}

void test2(visualization_msgs::MarkerArray &capsule, 
           visualization_msgs::MarkerArray &allow)
{
    double box1[3] = {0.5, 0.5, 0.5};
    double box2[3] = {0.5, 0.5, 0.5};
    double param1[7], param2[7];

    fittingCapsule(box1, param1);
    fittingCapsule(box2, param2);

    //not in collision
    double pos1[3] = {+1.0, 0, 0};
    double pos2[3] = {-1.0, 0, 0};
    //in collision
    //double pos1[3] = {+0.5, 0, 0};
    //double pos2[3] = {-0.5, 0, 0};

    S_Capsule cap1(Point3(pos1[0]+param1[0], 
                          pos1[1]+param1[1], 
                          pos1[2]+param1[2]), 
                   Point3(pos1[0]+param1[3], 
                          pos1[1]+param1[4], 
                          pos1[2]+param1[5]), 
                   param1[6]);
    S_Capsule cap2(Point3(pos2[0]+param2[0], 
                          pos2[1]+param2[1], 
                          pos2[2]+param2[2]), 
                   Point3(pos2[0]+param2[3], 
                          pos2[1]+param2[4], 
                          pos2[2]+param2[5]), 
                   param2[6]);

    capsuleMarker(point_t(pos1[0]+param1[0], 
                          pos1[1]+param1[1], 
                          pos1[2]+param1[2]), 
                  point_t(pos1[0]+param1[3], 
                          pos1[1]+param1[4], 
                          pos1[2]+param1[5]), 
                  param1[6], 0, capsule, Vector4d(0.5, 0, 0, 0.8));
    capsuleMarker(point_t(pos2[0]+param2[0], 
                          pos2[1]+param2[1], 
                          pos2[2]+param2[2]), 
                  point_t(pos2[0]+param2[3], 
                          pos2[1]+param2[4], 
                          pos2[2]+param2[5]), 
                  param2[6], 3, capsule, Vector4d(0, 0.5, 0, 0.8));

    CD_Pair pair(&cap1, &cap2);
    Point3 p1, p2;
    pair.getClosestPoints(p1, p2);
    visualization_msgs::Marker sph1, sph2;
    sph1.header.frame_id = sph2.header.frame_id = "base_link";
    sph1.id = 0; sph2.id = 1;
    sph1.type = sph2.type = visualization_msgs::Marker::SPHERE;
    sph1.scale.x = sph1.scale.y = sph1.scale.z = 0.015;
    sph2.scale.x = sph2.scale.y = sph2.scale.z = 0.015;
    sph1.color.r = 0.4; sph1.color.a = 1.;
    sph2.color.g = 0.4; sph2.color.a = 1.;
    sph1.pose.position.x = p1(0); sph1.pose.position.y = p1(1); sph1.pose.position.z = p1(2);
    sph2.pose.position.x = p2(0); sph2.pose.position.y = p2(1); sph2.pose.position.z = p2(2);
    sph1.pose.orientation.w = sph2.pose.orientation.w = 1;
    allow.markers.push_back(sph1); allow.markers.push_back(sph2);

}

