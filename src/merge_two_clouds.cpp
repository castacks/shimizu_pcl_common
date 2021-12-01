
#include <functional>
#include <map>

#include "shimizu_ros_common/ros_common.hpp"

#include "shimizu_pcl_common/common.hpp"
#include "shimizu_pcl_common/io.hpp"

#include "file_system/file_system.hpp"

const char* DEFAULT_NODE_NAME = "merge_two_clouds_node";

namespace spc = shimizu_pcl_common;

template < typename PointType >
static void read_and_merge(
    const std::string& in_fn_0,
    const std::string& in_fn_1,
    const std::string& out_fn ) {
    
    // Read the point clouds.
    typename pcl::PointCloud<PointType>::Ptr cloud_0 = 
        spc::read_point_cloud<PointType>( in_fn_0 );

    typename pcl::PointCloud<PointType>::Ptr cloud_1 = 
        spc::read_point_cloud<PointType>( in_fn_1 );

    // Merge.
    typename pcl::PointCloud<PointType>::Ptr merged ( new pcl::PointCloud<PointType> );
    *merged += *cloud_0;
    *merged += *cloud_1;

    // Save.
    spc::write_point_cloud( out_fn, merged );
}

int main(int argc, char** argv) {
    // Prepare the function map.
    std::map< 
        std::string, 
        std::function<void(std::string, std::string, std::string)> 
    > merge_func_map;

    merge_func_map.emplace( "xyz",            read_and_merge<pcl::PointXYZ> );
    merge_func_map.emplace( "xyz_rgb",        read_and_merge<pcl::PointXYZRGB> );
    merge_func_map.emplace( "xyz_normal",     read_and_merge<pcl::PointNormal> );
    merge_func_map.emplace( "xyz_rgb_normal", read_and_merge<pcl::PointXYZRGBNormal> );
    
    ROS_INFO_STREAM("Hello, " << DEFAULT_NODE_NAME << "! ");

    // ROS init.
    ros::init(argc, argv, DEFAULT_NODE_NAME);

    // The node handle.
    ros::NodeHandle nh("~");

    // Get the input parameters.
    GET_PARAM(std::string, cloud_0, nh)
    GET_PARAM(std::string, cloud_1, nh)
    GET_PARAM(std::string, out_fn, nh)
    GET_PARAM(std::string, point_type, nh)

    // Prepare the output directory.
    test_directory_by_filename( out_fn );

    // Check if the point type is supported.
    if ( merge_func_map.find( point_type ) == merge_func_map.end() ) {
        ROS_ERROR_STREAM("The point type " << point_type << " is not supported. The supported types are: ");
        for ( const auto& func_map : merge_func_map ) {
            std::cout << func_map.first << "\n";
        }

        return 1;
    }

    // Merge.
    merge_func_map[point_type]( cloud_0, cloud_1, out_fn );

    return 0;
}
