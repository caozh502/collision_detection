#include <iostream>
#include <Eigen/Core>

// Collision, Distance
#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include "fcl/broadphase/default_broadphase_callbacks.h"
// Distance Request & Result
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
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

#include "octree_viewer.cpp"
#include <pcl/filters/conditional_removal.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT>::Ptr PtCdtr;



// If you wonder the fcl::FCL_REAL, fcl::Transform3, and etc...//
// you should see <fcl/common/types.h>
// And you will understand it is equal to Eigen functions

//traversal_node_octree.h

Eigen::Matrix3f setRPY(const Eigen::Vector3f rot){
        Eigen::Matrix3f ret;
        ret = Eigen::AngleAxisf(rot.x(), Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(rot.y(), Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(rot.z(), Eigen::Vector3f::UnitZ());
        return ret;
}

fcl::CollisionObjectf createCollisionObject(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr)
{
    // octomap octree settings
    const float resolution = 0.1;
    const float prob_hit = 0.9;
    const float prob_miss = 0.1;
    const float clamping_thres_min = 0.12;
    const float clamping_thres_max = 0.98;
    octomap::point3d sensor_origin_3d (0.,0.,0.);

    std::shared_ptr<octomap::OcTree> octomap_octree = std::make_shared<octomap::OcTree>(resolution);
    octomap_octree->setProbHit(prob_hit);
    octomap_octree->setProbMiss(prob_miss);
    octomap_octree->setClampingThresMin(clamping_thres_min);
    octomap_octree->setClampingThresMax(clamping_thres_max);
    octomap::Pointcloud octoCloud;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = pointcloud_ptr->begin(); it < pointcloud_ptr->end(); ++it)
    {
        octoCloud.push_back(octomap::point3d(it->x, it->y,it->z));
    }
    octomap_octree->insertPointCloud(octoCloud, sensor_origin_3d, -1, true, true);
    octomap_octree->updateInnerOccupancy();
/*
    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;

#if defined(_OPENMP)
#pragma omp parallel
#endif
    {
#if defined(_OPENMP)
    auto thread_id = omp_get_thread_num();
    auto thread_num = omp_get_num_threads();
#else
        int thread_id = 0;
        int thread_num = 1;
#endif
        int start_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * thread_id;
        int end_idx = static_cast<int>(pointcloud_ptr->size() / thread_num) * (thread_id + 1);
        if (thread_id == thread_num - 1)
        {
            end_idx = pointcloud_ptr->size();
        }

        octomap::KeySet local_free_cells;
        octomap::KeySet local_occupied_cells;

        for (auto i = start_idx; i < end_idx; i++)
        {
            octomap::point3d point((*pointcloud_ptr)[i].x, (*pointcloud_ptr)[i].y, (*pointcloud_ptr)[i].z);
            octomap::KeyRay key_ray;
            if (octomap_octree->computeRayKeys(sensor_origin_3d, point, key_ray))
            {
                local_free_cells.insert(key_ray.begin(), key_ray.end());
            }

            octomap::OcTreeKey tree_key;
            if (octomap_octree->coordToKeyChecked(point, tree_key))
            {
                local_occupied_cells.insert(tree_key);
            }
        }

#if defined(_OPENMP)
#pragma omp critical
#endif
        {
            free_cells.insert(local_free_cells.begin(), local_free_cells.end());
            occupied_cells.insert(local_occupied_cells.begin(), local_occupied_cells.end());
        }
    }

    // free cells only if not occupied in this cloud
    for (auto it = free_cells.begin(); it != free_cells.end(); it++)
    {
        if (occupied_cells.find(*it) == occupied_cells.end())
        {
            octomap_octree->updateNode(*it, false);
        }
    }

    // occupied cells
    for (auto it = occupied_cells.begin(); it != occupied_cells.end(); it++)
    {
        octomap_octree->updateNode(*it, true);
    }
*/
    auto fcl_octree = std::make_shared<fcl::OcTreef> (octomap_octree);
    return fcl::CollisionObjectf (fcl_octree);
    //std::shared_ptr<fcl::CollisionGeometry<double>> fcl_geometry = fcl_octree;
    //return std::make_shared<fcl::CollisionObject<double>>(fcl_geometry);
}
int main(int argc, char* argv[]) {

    // Load input file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PtCdtr cloud_downSampled(new pcl::PointCloud<PointT>);
    PtCdtr cloud_filtered(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile("../tree.pcd", *cloud_xyzrgb) == -1)
    {
        cout << "点云数据读取失败！" << endl;
    }

    std::cout << "Orginal points number: " << cloud_xyzrgb->points.size() << std::endl;




    PtCdtr cloud_fz(new pcl::PointCloud<PointT>);
    PtCdtr cloud_fy(new pcl::PointCloud<PointT>);
    PtCdtr cloud_fx(new pcl::PointCloud<PointT>);
    PtCdtr cloud_condi(new pcl::PointCloud<PointT>);
    auto startTime = std::chrono::steady_clock::now();
    // 条件滤波
    // 创建条件对象
    pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT>);
    // 添加比较对象
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "z", pcl::ComparisonOps::GT, 0.0)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "z", pcl::ComparisonOps::LE, 15.0)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "x", pcl::ComparisonOps::GT, -5)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "x", pcl::ComparisonOps::LE, 5)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::GT, -7)));
    range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>(
            "y", pcl::ComparisonOps::LE, 0)));
    pcl::ConditionalRemoval<PointT> condRem;
    condRem.setInputCloud(cloud_xyzrgb);
    // 设置过滤条件
    condRem.setCondition(range_cond);
    // true：将条件之外的点还保留，但是设置NAN, 或者setUserFilterValue修改
    condRem.setKeepOrganized(false);
    condRem.filter(*cloud_condi);
/*
    //ROI filter
    //z轴直通滤波(深度）
    pcl::PassThrough<PointT> pass_z;//设置滤波器对象
    //参数设置
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    //z轴区间设置
    pass_z.setFilterLimits(0,15);
    //设置为保留还是去除(true为去除)
    //pass_z.setFilterLimitsNegative(true);
    pass_z.filter(*cloud_fz);

    //y轴（高度）
    pcl::PassThrough<PointT> pass_y;//设置滤波器对象
    pass_y.setInputCloud(cloud_fz);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-7, 0);
    //pass_y.setFilterLimitsNegative(false);
    pass_y.filter(*cloud_fy);
    //x轴（宽度）
    pcl::PassThrough<PointT> pass_x;//设置滤波器对象
    pass_x.setInputCloud(cloud_fy);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-5, 5);
    //pass_x.setFilterLimitsNegative(true);
    pass_x.filter(*cloud_fx);

*/

    // 下采样
    pcl::VoxelGrid<PointT> downSampled;  //创建滤波对象
    downSampled.setInputCloud (cloud_condi);            //设置需要过滤的点云给滤波对象
    downSampled.setLeafSize (0.3f, 0.3f, 0.3f);  //设置滤波时创建的体素体积为1cm的立方体
    downSampled.filter (*cloud_downSampled);           //执行滤波处理，存储输出
    std::cout<<"cloud_downSampled: "<<cloud_downSampled->size() <<std::endl;

    // 统计滤波
    pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       //创建滤波器对象
    statisOutlierRemoval.setInputCloud (cloud_downSampled);            //设置待滤波的点云
    statisOutlierRemoval.setMeanK (100);                                //设置在进行统计时考虑查询点临近点数
    statisOutlierRemoval.setStddevMulThresh (1.5);                     //设置判断是否为离群点的阀值:均值+1.0*标准差
    statisOutlierRemoval.filter (*cloud_filtered);                     //滤波结果存储到cloud_filtered
    std::cout<<"cloud_filtered: "<<cloud_filtered->size() <<std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Processing_1 took " << elapsedTime.count() << " ms" << std::endl;

    //tree
    pcl::copyPointCloud(*cloud_filtered, *cloud);
    auto octree_point = createCollisionObject(cloud);

    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Processing_2 took " << elapsedTime.count() << " ms" << std::endl;

    // Drone's size: 1668 mm × 1518 mm × 727 mm
    float length=2;
    float width=2;
    float height=1;
    float m=5;
    //fcl支持锥形cone，圆柱形，胶囊体
    std::shared_ptr<fcl::CollisionGeometryf> box_geometry1(new fcl::Boxf(width,height,length));
    fcl::CollisionObjectf box1(box_geometry1);


//    fcl::Vector3f trans1(0.,-height/2.,length/2);
    fcl::Vector3f trans1(0.,-height/2.-m,length/2.);
    fcl::Vector3f rot1(0.,0.,0.);
    fcl::Vector3f trans2(0.,0.,0.);
    fcl::Vector3f rot2(0.,0.,0.);
    box1.setTranslation(trans1);
    box1.setRotation(setRPY(rot1));
    octree_point.setTranslation(trans2);
    octree_point.setRotation(setRPY(rot2));

//    // DynamicAABBTree BroadPhase Managers
//    std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> group1 = std::make_shared<fcl::DynamicAABBTreeCollisionManager<float>>();
//
//    // Set Objects
//    group1->registerObject(&octree_point);
//    group1->setup();
//
//    // Data store
//    fcl::DefaultDistanceData<float> d_data1;
//    fcl::DefaultCollisionData<float> c_data1;
//
//
//    // 1. Contact Number between env and que
//    group1->collide(&box1, &c_data1, fcl::DefaultCollisionFunction);
//    bool res = c_data1.result.isCollision();
//    std::cout << res << std::endl;
//    pcl::PointXYZ p1 (d_data1.result.nearest_points[0][0],
//                      d_data1.result.nearest_points[0][1],
//                      d_data1.result.nearest_points[0][2]);
//    pcl::PointXYZ p2 (d_data1.result.nearest_points[1][0],
//                      d_data1.result.nearest_points[1][1],
//                      d_data1.result.nearest_points[1][2]);
//    std::cout << d_data1.result.nearest_points[0].transpose() << std::endl;
//    std::cout << d_data1.result.nearest_points[1].transpose() << std::endl;
    // Distance Request and Result
    fcl::DistanceRequestf request;
    fcl::DistanceResultf result;
    // Collision Request and Result
    fcl::CollisionRequestf C_request;
    fcl::CollisionResultf C_result;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    C_request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    //request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    //auto x1 =  fcl::GJKSolverType::GST_INDEP;


    // Calculate distance
    result.clear();

    fcl::collide(&box1, &octree_point,C_request,C_result);
    fcl::distance(&box1, &octree_point, request, result);


    // Show results

    auto ans((C_result.isCollision()) ? "There is obstacle in the range":"no thing in the range");
    std::cout << ans << ": " << C_result.numContacts() << std::endl;
    std::cout << "The min distance is "<< result.min_distance << std::endl;
    std::cout << result.nearest_points[0].transpose() << std::endl;
    std::cout << result.nearest_points[1].transpose() << std::endl;

    pcl::PointXYZ p1 (result.nearest_points[0][0],result.nearest_points[0][1],result.nearest_points[0][2]);
    pcl::PointXYZ p2 (result.nearest_points[1][0],result.nearest_points[1][1],result.nearest_points[1][2]);


    endTime = std::chrono::steady_clock::now();
    elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Processing_3 took " << elapsedTime.count() << " ms" << std::endl;

    // 显示网格化结果
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    OctreeViewer v(cloud, 0.0025);
    v.viz.setBackgroundColor(0, 0, 0);  //
    v.viz.addPointCloud(cloud_filtered,"pt");
    v.viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "pt");
    v.viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.8, 0.2,"pt");
    v.viz.addSphere(p1, 0.05, "s1", 0);
    v.viz.addSphere(p2, 0.05, "s2", 0);
    v.viz.addCube(-width/2., width/2, -height-m, 0-m, 0, length, 1, 0, 0, "cube");
    v.viz.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "cube");
    while (!v.viz.wasStopped())
    {
        v.viz.spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}

