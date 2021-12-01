//
// Created by yaoyu on 3/27/20.
//

#ifndef __SHIMIZU_PCL_COMMON_IO_HPP__
#define __SHIMIZU_PCL_COMMON_IO_HPP__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "shimizu_pcl_common/common.hpp"

namespace shimizu_pcl_common 
{

template<typename T>
typename pcl::PointCloud<T>::Ptr read_point_cloud(const std::string &fn) {
    // ========== Read the point cloud from the file. ==========
    // std::cout << "Loading points from " << fn << " ... \n";

    typename pcl::PointCloud<T>::Ptr pOutCloud ( new pcl::PointCloud<T> );

    if (pcl::io::loadPLYFile<T>(fn, *pOutCloud) == -1) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw (std::runtime_error(ss.str()));
    }

    return pOutCloud;
}

template<typename PCL_PC_T>
void read_point_cloud(
    const std::string &fn, 
    PCL_PC_T &outCloud) {
    // ========== Read the point cloud from the file. ==========
    // std::cout << "Loading points from " << fn << " ... \n";

    typedef typename PCL_PC_T::PointType PointType;

    if (pcl::io::loadPLYFile<PointType>(fn, outCloud) == -1) {
        std::stringstream ss;
        ss << "Failed to read: " << fn;
        throw (std::runtime_error(ss.str()));
    }
}

template < typename Point_T, typename Eigen_MatX_T >
void read_point_cloud_xyz_as_eigen_matrix( 
    const std::string &fn,
    Eigen_MatX_T &mat ) {
    typename pcl::PointCloud<Point_T>::Ptr pCloud = read_point_cloud<Point_T>(fn);
    convert_pcl_2_eigen_matrix(pCloud, mat);
}

template < typename PCL_Ptr_T >
void write_point_cloud( 
    const std::string& fn, 
    const PCL_Ptr_T& cloud, 
    bool flagBinary=true ) {

    pcl::PLYWriter writer;
    // std::cout << "Saving the filtered point cloud. \n";
    writer.write(fn, *cloud, flagBinary, false);
}

} // namespace shimizu_pcl_common.
#endif //__SHIMIZU_PCL_COMMON_IO_HPP__
