/**
 * @file workspace_Database.h
 */
#pragma once

#include <vector>
#include <map>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/octree/octree_search.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

typedef vector<vector<double>> Config;
typedef vector<Config> Configs;
//typedef map<VectorXd, Configs> Database;
/// order of point cloud and that of config array are same 
//typedef pair<PointCloud<PointXYZ>, vector<Configs>> Database;
typedef pair<octree::OctreePointCloudSearch<PointXYZ>, vector<Configs>> Database;


// Voxel by Octree
void makeOctreePointCloud(PointCloud<PointXYZ>::Ptr in_cloud,
                          PointCloud<PointXYZ>::Ptr voxel_cloud)
                          //PointCloud<PointXYZ>::Ptr voxel_cloud,
                          //Configs configs, Database& database)
{
  float resolution = 0.10;
  octree::OctreePointCloudSearch<PointXYZ> octree(resolution); // : public OctreePointCloud?
  //octree::OctreePointCloud<PointXYZ> octree(resolution);
  octree.setInputCloud(in_cloud);
  octree.addPointsFromInputCloud(); // add points to octree

  octree::OctreePointCloud<PointXYZ>::AlignedPointXYZVector voxel_center_list_arg;
  int voxel_num = octree.getOccupiedVoxelCenters(voxel_center_list_arg);
  cout << "voxel_num of octree is " << voxel_num << endl;
  cout << "original num of points is " << in_cloud->size() << endl;;
  cout << "tree depth is " << octree.getTreeDepth() << endl;

  //IndicesConstPtr indices(octree.getIndices());
  //if (indices) cout << "has validity" << endl;
  //else cout << "no valid" << endl;
  //int ind = indices->size();
  //cout << "indices size is " << ind << endl;
  //for (int i = 0; i < indices->size(); i++)
  //{
  //  cout << indices->at(i) << ", ";
  //}cout << endl;

  voxel_cloud->clear();
  for (auto&& var : voxel_center_list_arg)
  {
    voxel_cloud->push_back(var);
  }

  int cnt = 0;
  //for (int i = 0; i < in_cloud->size(); i++)
  for (int i = 0; i < voxel_center_list_arg.size(); i++)
  {
    vector<int> point_idx_data;
    // return points' indices(in a voxel) for input cloud's index
    //if (octree.voxelSearch(i, point_idx_data))
    // return points' indices(in a voxel) for a voxel position
    if (octree.voxelSearch(voxel_center_list_arg[i], point_idx_data))
    {
      cnt += point_idx_data.size();
    }
  }
  cout << "points check " << cnt << " is same as next number" << endl;
  cout << in_cloud->size() << endl;

  //database.first = octree;
}
