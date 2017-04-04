//urdf to optimized capsule
/*
 *This uses sch-core. 
 *FCL Library may have bug about Collision of Sphere to Box
 */
#pragma once

#include <urdf_model/model.h>
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
void capsulesFromModel(const boost::shared_ptr<urdf::ModelInterface>& model, 
                       //double capParam[7])
                       std::map<std::string, std::vector<double>> capParams)
{
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
                     std::vector<double> distance)
{
    //size_t size = capParams.size();
    //boost::shared_ptr<S_Capsule> capsules;
    std::vector<S_Capsule> capsules;//Pay attention to move addressa
    std::vector<std::string> capsuleNames;
    
    int count = 0;
    for (auto itr = mbc.bodyPosW.begin(); itr < mbc.bodyPosW.end(); itr++) {
        std::string linkname = mb.body(count).name();
        if (capParams[linkname].empty()) continue;//std::map<,double> default/ maybe ok
        Vector3d wp1 = itr->translation() + itr->rotation().transpose() * Vector3d(capParams[linkname][0], capParams[linkname][1], capParams[linkname][2]);
        Vector3d wp2 = itr->translation() + itr->rotation().transpose() * Vector3d(capParams[linkname][3], capParams[linkname][4], capParams[linkname][5]);
        capsules.push_back(S_Capsule(Point3(wp1(0),wp1(1),wp1(2)), Point3(wp2(0),wp2(1),wp2(2)), capParams[linkname][6]));
        capsuleNames.push_back(linkname);

        count++;
    }

    //make CD_Pair considering with never colliding pair
    int combCount = 0;
    if (capsuleNames.empty()) {
        std::cerr << "capsuleNames is empty!" << std::endl;
    }
    while (true) {
        for (auto itr = capsuleNames.begin()+combCount; itr != capsuleNames.end()-1; itr++) {
            std::string first = *itr;
            std::string second = *(itr+1);
            CD_Pair pair(&capsules[combCount], &capsules[combCount+1]);
            std::cout << "Distance " << sqrt(fabs(pair.getDistance())) <<first << ", " << second << std::endl;
        }
        combCount++;
    }

}


