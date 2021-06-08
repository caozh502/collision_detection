//
// Created by caleb on 01.06.21.
//

#include <iostream>

// Collision, Distance
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
//#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
//#include "fcl/broadphase/default_broadphase_callbacks.h"
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
//#include <fcl/common/types.h>
#include "test_fcl_utility.h"
// PCL lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>

using namespace fcl;

int main(int argc, char* argv[]){
    auto startTime = std::chrono::steady_clock::now();
    // load sample 3D model
    std::vector<Vector3f> vertices;
    std::vector<Triangle> triangles;
    fcl::test::loadOBJFile("../tree_mesh.obj", vertices, triangles);

    //build bvh
    typedef BVHModel<AABBf> Model;
    std::shared_ptr<Model> geom = std::make_shared<Model>();
    geom->beginModel();
    geom->addSubModel(vertices,triangles);
    geom->endModel();

    std::cout<<geom->getNumBVs()<<std::endl;
    std::cout<<geom->num_tris<<std::endl;
    std::cout<<geom->num_vertices<<std::endl;

    // set object transforms
    Transform3f pose1 = Transform3f::Identity();
    Transform3f pose2 = Transform3f::Identity();

    // collision objects can be set
    CollisionObjectf* obj1 = new CollisionObjectf(geom, pose1);
    double length=14;
    double width=2;
    double height=2;
    double m=5;
    //fcl支持锥形cone，圆柱形，胶囊体
    std::shared_ptr<fcl::CollisionGeometryf> box_geometry1(new fcl::Boxf(width,height,length));
    CollisionObjectf* box1 = new CollisionObjectf (box_geometry1,pose2);
    fcl::Vector3f trans1(0.,-height/2.-m,length/2.);
    box1->setTranslation(trans1);

    // set the collision request structure, here we just use the default setting
    CollisionRequestf request;
    CollisionResultf result;
    DistanceRequestf D_request;
    DistanceResultf D_result;
    D_request.enable_nearest_points = true;
    D_request.enable_signed_distance = true;
    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    D_request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;

//    fcl::detail::MeshCollisionTraversalNode<fcl::AABB<float>> traveral_node;
//    if (!fcl::detail::initialize(traveral_node, obj1, pose1, box1, pose2, request, result))
//        std::cout << "initialize error" << std::endl;
//    fcl::detail::collide(&traveral_node);

    // perform collision test
    collide(obj1, box1, request, result);
    distance(obj1, box1, D_request, D_result);
    std::cout<< result.numContacts() << std::endl;
    std::cout << "The min distance is "<< D_result.min_distance << std::endl;
    std::cout << D_result.nearest_points[0].transpose() << std::endl;
    std::cout << D_result.nearest_points[1].transpose() << std::endl;



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Processing took " << elapsedTime.count() << " ms" << std::endl;
}

