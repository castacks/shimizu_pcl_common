//
// Created by yaoyu on 3/25/20.
//

#ifndef __SHIMIZU_PCL_COMMON_EXTRACTION_HPP__
#define __SHIMIZU_PCL_COMMON_EXTRACTION_HPP__

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "shimizu_pcl_common/common.hpp"

namespace shimizu_pcl_common
{

template < typename PCL_Ptr_T >
void extract_points( 
    const PCL_Ptr_T pInput,
    PCL_Ptr_T pOutput,
    const pcl::PointIndices::Ptr indices,
    bool negative=false ) {

    typedef typename PCL_Ptr_T::element_type::PointType PointType;
    
    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(pInput);
    extract.setIndices(indices);
    extract.setNegative(negative);

    if ( pInput.get() == pOutput.get() ) {
        PCL_Ptr_T pTemp ( new pcl::PointCloud<PointType> );
        extract.filter( *pTemp );
        pOutput = pTemp; // Should work.
    } else {
        extract.filter( *pOutput );
    }
}

template < typename PCL_Ptr_T >
PCL_Ptr_T extract_points( 
    const PCL_Ptr_T pInput,
    const pcl::PointIndices::Ptr indices,
    bool negative=false ) {

    typedef typename PCL_Ptr_T::element_type::PointType PointType;

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(pInput);
    extract.setIndices(indices);
    extract.setNegative(negative);

    PCL_Ptr_T pOutput( new pcl::PointCloud<PointType> );
    extract.filter( *pOutput );
    return pOutput;
}

template < typename PCL_Ptr_T, typename Int_T >
PCL_Ptr_T extract_points( 
    const PCL_Ptr_T pInput,
    const std::vector<Int_T>& indices,
    bool negative=false ) {
    typedef typename PCL_Ptr_T::element_type::PointType PointType;
    
    PCL_Ptr_T pOutput ( new pcl::PointCloud<PointType> );

    pcl::PointIndices::Ptr pclIndices = convert_vector_2_pcl_indices( indices );

    extract_points( pInput, pOutput, pclIndices, negative );

    return pOutput;
}

template < typename PCL_Ptr_T, typename Int_T >
void extract_points( 
    const PCL_Ptr_T pInput,
    PCL_Ptr_T pOutput,
    const std::vector<Int_T>& indices,
    bool negative=false ) {
    
    typedef typename PCL_Ptr_T::element_type::PointType PointType;
    
    pcl::PointIndices::Ptr pclIndices = convert_vector_2_pcl_indices( indices );

    extract_points( pInput, pOutput, pclIndices, negative );
}

template <typename PCL_Ptr_T>
PCL_Ptr_T crop_by_CropBox(
        const PCL_Ptr_T inCloud,
        const pcl::PointXYZ &minPoint,
        const pcl::PointXYZ &maxPoint ) {
    
    typedef typename PCL_Ptr_T::element_type::PointType PointType;

    Eigen::Vector4f p0 = shimizu_pcl_common::create_eigen_vector4_from_xyz<PointType, float>(minPoint);
    Eigen::Vector4f p1 = shimizu_pcl_common::create_eigen_vector4_from_xyz<PointType, float>(maxPoint);

    PCL_Ptr_T pOutCloud ( new pcl::PointCloud<PointType> );

    pcl::CropBox<PointType> pass;
    pass.setMin( p0 );
    pass.setMax( p1 );
    pass.setInputCloud(inCloud);
    pass.filter(*pOutCloud);

    // std::cout << "Cropped " << pOutCloud->size() << " points. " << std::endl;

    return pOutCloud;
}

} // namespace pcu.

#endif //__SHIMIZU_PCL_COMMON_EXTRACTION_HPP__
