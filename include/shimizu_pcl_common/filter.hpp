
#ifndef __SHIMIZU_PCL_COMMON_FILTER_HPP__
#define __SHIMIZU_PCL_COMMON_FILTER_HPP__

#include "shimizu_pcl_common/common.hpp"
#include "shimizu_pcl_common/extraction.hpp"

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

namespace shimizu_pcl_common
{

template < typename Vector_T >
void append_2_vector( 
    const Vector_T& from, 
    Vector_T& to ) {
    
    const auto nFrom = from.size();

    if ( 0 == nFrom ) {
        return;
    }

    const auto nTo = to.size();

    to.resize( nTo + nFrom );

    std::copy( from.begin(), from.end(), to.begin() + nTo );
}

template < typename PCL_Ptr_T >
PCL_Ptr_T cluster_filter( 
    const PCL_Ptr_T& pInput,
    double tolerance, int minSize, int maxSize ) {

    assert( tolerance > 0 );
    assert( minSize > 0 );
    assert( maxSize > minSize );

    typedef typename PCL_Ptr_T::element_type::PointType PointType;

    // Tree.
    typename pcl::search::KdTree<PointType>::Ptr tree ( new pcl::search::KdTree<PointType> );

    // The indices.
    std::vector< pcl::PointIndices > clusterIndices;

    // The extractor.
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pInput);
    ec.extract(clusterIndices);

    const auto N = clusterIndices.size();

    if ( 0 == N ) {
        std::stringstream ss;
        ss << "No clusters found. "
           << "tolerance: " << tolerance << ", "
           << "minSize: " << minSize << ", "
           << "maxSize: " << maxSize << ". ";
        throw( std::runtime_error( ss.str() ) );
    }

    pcl::PointIndices::Ptr indices ( new pcl::PointIndices );

    for ( const auto& ids : clusterIndices ) {
        if ( ids.indices.size() <= minSize ) {
            append_2_vector( ids.indices, indices->indices );
        }
    }

    std::cout << N << " small clusters with less than " << minSize << " points found. "
              << indices->indices.size() << " points to be removed. \n";

    // Remove the points.
    PCL_Ptr_T pOutput ( new pcl::PointCloud<PointType> );
    extract_points( pInput, pOutput, indices, true );
    return pOutput;
}

template < typename PCL_Ptr_T >
PCL_Ptr_T stat_outlier_removal(
    const PCL_Ptr_T& in_cloud,
    int mean_k,
    double std_dev_factor ) {
    
    typedef typename PCL_Ptr_T::element_type::PointType PointType;

    PCL_Ptr_T out_cloud ( new pcl::PointCloud<PointType> );
    
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(in_cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev_factor);
    sor.filter( *out_cloud );

    return out_cloud;
}

/**
 * Must use a point type that supports normal vector.
 */
template < typename PCL_Ptr_T >
PCL_Ptr_T moving_least_squares(
    const PCL_Ptr_T& in_cloud,
    double radius,
    int poly_order=2) {
    
    typedef typename PCL_Ptr_T::element_type::PointType PointType;

    PCL_Ptr_T out_cloud ( new pcl::PointCloud<PointType> );

    // Create a KD-Tree.
    typename pcl::search::KdTree<PointType>::Ptr tree ( new pcl::search::KdTree<PointType> );

    pcl::MovingLeastSquares<PointType, PointType> mls;
    mls.setComputeNormals(true);
    mls.setPolynomialOrder(poly_order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);
    mls.setInputCloud(in_cloud);
    mls.process(*out_cloud);

    return out_cloud;
}

} // namespace shimizu_pcl_common


#endif // __SHIMIZU_PCL_COMMON_FILTER_HPP__