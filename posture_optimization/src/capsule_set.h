#pragma once
#include "roboptim/capsule/volume.hh"
#include "roboptim/capsule/util.hh"
#include "roboptim/capsule/fitter.hh"
#include <visualization_msgs/MarkerArray.h>

using namespace roboptim::capsule;
using namespace Eigen;
//Read from cad-model
//Set capsule

//Marker of Capsule from cylinder and sphear
void capsuleMarker(point_t p1, point_t p2, value_type radius,
                   int id_3, //n, n+1, n+2
                   visualization_msgs::MarkerArray &msg, 
                   Vector4d colorRGBA = Vector4d(0.5, 0, 0, 0.6), 
                   std::string frame_id = "base_link")
{
    visualization_msgs::Marker cylinder, sph1, sph2;
    point_t pd = p1 - p2; pd = pd/pd.norm();
    Quaterniond q = Quaterniond::FromTwoVectors(pd, point_t(1., 0., 0.));
    q = q*Quaterniond(AngleAxisd(M_PI/2.0, Vector3d::UnitY()));
    q = q.normalized();
    auto pos = 0.5*(p1 + p2);
    auto stamp = ros::Time::now();

    std_msgs::Header header; header.frame_id = frame_id; header.stamp = stamp;
    cylinder.header = sph1.header = sph2.header = header;
    cylinder.id = id_3; sph1.id = id_3 + 1; sph2.id = id_3 + 2;
    cylinder.type = visualization_msgs::Marker::CYLINDER;
    sph1.type = visualization_msgs::Marker::SPHERE;
    sph2.type = visualization_msgs::Marker::SPHERE;
    cylinder.action = sph1.action = sph2.action = visualization_msgs::Marker::ADD;
    cylinder.pose.position.x = pos[0]; cylinder.pose.position.y = pos[1]; cylinder.pose.position.z = pos[2];
    sph1.pose.position.x = p1[0]; sph1.pose.position.y = p1[1]; sph1.pose.position.z = p1[2];
    sph2.pose.position.x = p2[0]; sph2.pose.position.y = p2[1]; sph2.pose.position.z = p2[2];
    cylinder.pose.orientation.w = q.w();
    cylinder.pose.orientation.x = q.x();
    cylinder.pose.orientation.y = q.y();
    cylinder.pose.orientation.z = q.z();
    sph1.pose.orientation.w = sph2.pose.orientation.w = 1.;
    cylinder.scale.x = cylinder.scale.y = 2*radius;
    cylinder.scale.z = (p1 - p2).norm();
    sph1.scale.x = sph1.scale.y = sph1.scale.z = 2*radius;
    sph2.scale.x = sph2.scale.y = sph2.scale.z = 2*radius;
    std_msgs::ColorRGBA color;
    color.r = colorRGBA(0); color.g = colorRGBA(1); color.b = colorRGBA(2); color.a = colorRGBA(3); 
    cylinder.color = sph1.color = sph2.color = color;

    msg.markers.push_back(cylinder);
    msg.markers.push_back(sph1);
    msg.markers.push_back(sph2);
}

void test1(visualization_msgs::MarkerArray &msg)
{
    polyhedron_t polyhedron;

    //Build a cubic polyhedron centerd in (0,0,0)
    value_type halfLength = 0.5;
    //polyhedron.push_back (point_t (-halfLength, -halfLength, -halfLength));
    //polyhedron.push_back (point_t (-halfLength, -halfLength, halfLength));
    //polyhedron.push_back (point_t (-halfLength, halfLength, -halfLength));
    //polyhedron.push_back (point_t (-halfLength, halfLength, halfLength));
    //polyhedron.push_back (point_t (halfLength, -halfLength, -halfLength));
    //polyhedron.push_back (point_t (halfLength, -halfLength, halfLength));
    //polyhedron.push_back (point_t (halfLength, halfLength, -halfLength));
    //polyhedron.push_back (point_t (halfLength, halfLength, halfLength));

    // !! thin geometry makes capsule-volume big.
    // thin box
    //polyhedron.push_back (point_t (-halfLength, -halfLength, -halfLength*0.1));
    //polyhedron.push_back (point_t (-halfLength, -halfLength, halfLength*0.1));
    //polyhedron.push_back (point_t (-halfLength, halfLength, -halfLength*0.1));
    //polyhedron.push_back (point_t (-halfLength, halfLength, halfLength*0.1));
    //polyhedron.push_back (point_t (halfLength, -halfLength, -halfLength*0.1));
    //polyhedron.push_back (point_t (halfLength, -halfLength, halfLength*0.1));
    //polyhedron.push_back (point_t (halfLength, halfLength, -halfLength*0.1));
    //polyhedron.push_back (point_t (halfLength, halfLength, halfLength*0.1));

    // thin regular hexagonal prism
    Vector3d axis(0,0,1);
    int max = 6;
    double ref = halfLength*sqrt(tan(M_PI/max)*tan(M_PI/max) + 1);
    for (int i = 0; i < max; i++) {
        auto rotz = AngleAxisd(2.*M_PI/max*i, axis);
        //auto rotx = AngleAxisd(2.*M_PI, Vector3d(1,0,0));
        //auto roty = AngleAxisd(2.*M_PI, Vector3d(0,1,0));
        //pattern1
        auto p0 = rotz * point_t(ref, 0, -halfLength*0.1);
        auto p1 = rotz * point_t(ref, 0, +halfLength*0.1);
        //pattern2
        //auto p0 = rotx * rotz * point_t(ref, 0, -halfLength*0.1);
        //auto p1 = rotx * rotz * point_t(ref, 0, +halfLength*0.1);
        //pattern3
        //auto p0 = roty * rotz * point_t(ref, 0, -halfLength*0.1);
        //auto p1 = roty * rotz * point_t(ref, 0, +halfLength*0.1);
        std::cout << "p0 " << p0 << std::endl;
        std::cout << "p1 " << p1 << std::endl;
        polyhedron.push_back(p0);
        polyhedron.push_back(p1);
    }

    polyhedrons_t polyhedrons;
    polyhedrons.push_back(polyhedron);

    // Define initial capsule parameters. The segment must be inside the
    // polyhedron, and the capsule must contain the polyhedron.

    // To do so, compute initial guess by finding a bounding capsule
    // (not the minimum one).

    // If needed, the convex hull of the polyhedron can be first computed
    // to reduce the number of constraints and accelerate the optimization
    // phase.

    polyhedrons_t convexPolyhedrons;
    computeConvexPolyhedron(polyhedrons, convexPolyhedrons);

    // Create fitter. It is used to find the best fitting capsule on the
    // polyhedron vector
    Fitter fitter_cube(convexPolyhedrons);

    point_t endPoint1 = point_t(0., 0., 0.);
    point_t endPoint2 = point_t(0., 0., 0.);
    value_type radius = 0.;
    computeBoundingCapsulePolyhedron(convexPolyhedrons, 
                                     endPoint1, endPoint2, radius);

    argument_t initParam(7);
    convertCapsuleToSolverParam(initParam, endPoint1, endPoint2, radius);
    cout << initParam << endl;

    // Compute best fitting capsule
    fitter_cube.computeBestFitCapsule(initParam);
    argument_t solutionParam = fitter_cube.solutionParam();
    cout << fitter_cube << endl;
    point_t resPoint1(solutionParam[0], solutionParam[1], solutionParam[2]);
    point_t resPoint2(solutionParam[3], solutionParam[4], solutionParam[5]);
    double resRadius = solutionParam[6];

    capsuleMarker(endPoint1, endPoint2, radius, 0, msg);
    Vector4d green = Vector4d(0., 0.5, 0., 0.6);
    capsuleMarker(resPoint1, resPoint2, resRadius, 3, msg, green);
}

void fittingCapsule(double xyz[3], double resParam[7])
{
    //For ease, set cubic for Sphere, Cylinder and Box
    //Build a cubic polyhedron centered in (0,0,0)
    polyhedron_t polyhedron;
    value_type halfX = xyz[0];
    value_type halfY = xyz[1];
    value_type halfZ = xyz[2];
    polyhedron.push_back (point_t (-halfX, -halfY, -halfZ));
    polyhedron.push_back (point_t (-halfX, -halfY,  halfZ));
    polyhedron.push_back (point_t (-halfX,  halfY, -halfZ));
    polyhedron.push_back (point_t (-halfX,  halfY,  halfZ));
    polyhedron.push_back (point_t ( halfX, -halfY, -halfZ));
    polyhedron.push_back (point_t ( halfX, -halfY,  halfZ));
    polyhedron.push_back (point_t ( halfX,  halfY, -halfZ));
    polyhedron.push_back (point_t ( halfX,  halfY,  halfZ));

    polyhedrons_t polyhedrons;
    polyhedrons.push_back(polyhedron);

    polyhedrons_t convexPolyhedrons;
    computeConvexPolyhedron(polyhedrons, convexPolyhedrons);

    Fitter fitter_cube(convexPolyhedrons);

    point_t endPoint1 = point_t(0., 0., 0.);
    point_t endPoint2 = point_t(0., 0., 0.);
    value_type radius = 0.;
    computeBoundingCapsulePolyhedron(convexPolyhedrons, 
                                     endPoint1, endPoint2, radius);

    argument_t initParam(7);
    convertCapsuleToSolverParam(initParam, endPoint1, endPoint2, radius);

    // Compute best fitting capsule
    fitter_cube.computeBestFitCapsule(initParam);
    argument_t solutionParam = fitter_cube.solutionParam();
    for (int i = 0; i < 7; i++) resParam[i] = solutionParam[i];
 
}


