//
// Created by yaoyu on 3/25/20.
//

#ifndef __SHIMIZU_PCL_COMMON_COMMON_HPP__
#define __SHIMIZU_PCL_COMMON_COMMON_HPP__

#include <algorithm>    // std::sort
#include <cmath>
#include <iostream>
#include <numeric>      // std::iota
#include <sstream>
#include <string>
#include <vector>

#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

// TODO: Find a way to do this for all the other ROS packages.
#ifdef EIGEN_VERSION_AT_LEAST
#if !EIGEN_VERSION_AT_LEAST(3,4,0)

namespace Eigen
{

template < typename Type, int Size >
using Vector = Matrix<Type, Size, 1>;

template < typename Type >
using Matrix3 = Matrix<Type, 3, 3>;

template < typename Type >
using Matrix4 = Matrix<Type, 4, 4>;

} // Namespace Eigen

#endif
#endif

namespace shimizu_pcl_common
{

template < typename PCL_PC_T, typename Eigen_MatX_T >
void convert_pcl_2_eigen_matrix(
        const PCL_PC_T &cloud,
        Eigen_MatX_T &points ) {
    const std::size_t N = cloud.size();
    assert(N > 0);

    points.resize(3, N);
    for ( std::size_t i = 0; i < N; ++i) {
        const auto& point = cloud.at(i);
        points(0, i) = point.x;
        points(1, i) = point.y;
        points(2, i) = point.z;
    }
}

template < typename Point_T, typename Eigen_MatX_T >
typename pcl::PointCloud<Point_T>::Ptr convert_eigen_matrix_2_pcl_xyz(
    const Eigen_MatX_T &mat ) {
    
    const std::size_t N = mat.cols();

    assert( N > 0 );

    typename pcl::PointCloud<Point_T>::Ptr pOutput ( new pcl::PointCloud<Point_T> );
    pOutput->resize( N );

    for ( std::size_t i = 0; i < N; ++i ) {
        Point_T p;
        p.x = mat(0, i);
        p.y = mat(1, i);
        p.z = mat(2, i);
        pOutput->at(i) = p;
    }

    return pOutput;
}

template < typename Eigen_MatX_T >
pcl::PointCloud<pcl::PointXYZ>::Ptr convert_eigen_depth_img_2_pcl_xyz(
        const Eigen_MatX_T &img ) {
    const std::size_t height = img.rows();
    const std::size_t width  = img.cols();

    assert( height > 0 );
    assert( width  > 0 );

    pcl::PointCloud<pcl::PointXYZ>::Ptr pOutput ( new pcl::PointCloud<pcl::PointXYZ> );
    pOutput->clear();

    std::size_t count = 0;

    for ( std::size_t i = 0; i < height; ++i ) {
        for ( std::size_t j = 0; j < width; ++j ) {
            if ( img(i, j) <= 0 ) {
                continue;
            }

            pcl::PointXYZ p;
            p.x = j;
            p.y = i;
            p.z = img(i, j);
            pOutput->push_back(p);
            count++;
        }
    }

    return pOutput;
}

template < typename pT, typename rT >
Eigen::Vector<rT, 3> create_eigen_vector3_from_xyz(const pT &point) {
    EIGEN_ALIGN16 Eigen::Vector<rT, 3> v;
    v << point.x, point.y, point.z;
    return v;
}

template < typename pT, typename rT >
Eigen::Vector<rT, 4> create_eigen_vector4_from_xyz(const pT &point) {
    EIGEN_ALIGN16 Eigen::Vector<rT, 4> v;
    v << point.x, point.y, point.z, static_cast<rT>(1.0);
    return v;
}

template < typename pT, typename rT >
Eigen::Vector<rT, 3> create_eigen_vector3_from_normal( const pT &point ) {
    EIGEN_ALIGN16 Eigen::Vector<rT, 3> v;
    v << point.normal_x, point.nomrl_y, point.normal_z;
    return v;
}

template < typename pT, typename rT >
Eigen::Vector<rT, 4> create_eigen_vector4_from_normal( const pT &point ) {
    EIGEN_ALIGN16 Eigen::Vector<rT, 4> v;
    v << point.normal_x, point.nomrl_y, point.normal_z, static_cast<rT>(1.0);
    return v;
}

template < typename pT, typename rT >
Eigen::Matrix4<rT> create_eigen_transform_matrix_0(
        const pT &translation,
        const Eigen::Matrix3<rT> &rotation ) {
    Eigen::Matrix4<rT> transMat = Eigen::Matrix4<rT>::Identity();

    transMat.block( 0,0,3,3 ) = rotation;
//    transMat( Eigen::seq(0,2), Eigen::seq(0,2) ) = rotation;
    transMat( 0, 3 ) = translation.x;
    transMat( 1, 3 ) = translation.y;
    transMat( 2, 3 ) = translation.z;

    return transMat;
}

template < typename rT >
Eigen::Matrix4<rT> create_eigen_transform_matrix_1(
        const pcl::PointXYZ &translation,
        const Eigen::Matrix3<rT> &rotation ) {
    Eigen::Matrix4<rT> transMat = Eigen::Matrix4<rT>::Identity();

    Eigen::Vector<rT, 3> transVec;
    transVec << translation.x, translation.y, translation.z;

    transMat.block(0,0,3,3) = rotation.transpose();
    // transMat( Eigen::seq(0,2), 3 ) = -rotation.transpose() * transVec;
    transMat.bloc(0,3,3,1) = -rotation.transpose() * transVec;

    return transMat;
}

template < typename pT0, typename pT1 >
float distance_two_points( const pT0& p0, const pT1& p1 ) {
    const float d0 = p0.x - p1.x;
    const float d1 = p0.y - p1.y;
    const float d2 = p0.z - p1.z;

    return std::sqrt( d0 * d0 + d1 * d1 + d2 * d2 );
}

template < typename Vector_T >
pcl::PointIndices::Ptr convert_vector_2_pcl_indices( const Vector_T &v) {
    const auto N = v.size();

    if ( N == 0 ) {
        std::stringstream ss;
        ss << "Vector is empty. Cannot convert to pcl::PointIndices. ";
        throw( std::runtime_error( ss.str() ) );
    }

    pcl::PointIndices::Ptr pclIndices (new pcl::PointIndices() );

    pclIndices->indices.resize( N );
    std::copy(v.begin(), v.end(), pclIndices->indices.begin() );

    return pclIndices;
}

template < typename Vector_T >
void convert_vector_2_pcl_indices( const Vector_T& v, pcl::PointIndices::Ptr& indices ) {
    const auto N = v.size();

    if ( N == 0 ) {
        std::stringstream ss;
        ss << "Vector is empty. Cannot convert to pcl::PointIndices. ";
        throw( std::runtime_error( ss.str() ) );
    }

    indices->indices.resize(N);

    std::copy( v.begin(), v.end(), indices->indices.begin() );
}

}

#endif //__SHIMIZU_PCL_COMMON_COMMON_HPP__
