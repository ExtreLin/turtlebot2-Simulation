#ifndef NEXTBESTVIEW_H
#define NEXTBESTVIEW_H

#include "data_types.h"

namespace nextbestview {

    /**
     * 获得路径图，标记出当前地图中哪些是空点，可以到达
     * @param host_uncertainty_map  空间体素，保存每个格子的扫描状态
     * @param volume_size 体块的size
     */
    cv::Mat get_validness_map(const cv::Mat& host_uncertainty_map, const int3& volume_size);

     /**
     * 获得收益值队列，收益值高到低
     * @param host_uncertainty_map  空间体素，保存每个格子的扫描状态
     * @param volume_size 体块的size
     * @param values             收益值队列
     */
    void  get_uncertainty_priority_queue(const cv::Mat& host_uncertainty_map, 
                                                                                   const cv::Mat& host_tsdf_volume,
                                                                                   const int3& volume_size,
                                                                                   std::vector<std::pair<Eigen::Vector3i, float>>& values);


    Eigen::Vector3f  find_next_best_view(const cv::Mat& validness_map,
                                                           const cv::Mat& host_uncertainty_map, 
                                                           const std::vector<std::pair<Eigen::Vector3i, float>>& values,
                                                           const kinectfusion::SurfaceMesh surface_mesh,
                                                           const Eigen::Matrix4f& primary_model_view,
                                                           const int3& volume_size,
                                                           const float voxel_scale);
    
    void threeD_DDA(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                    std::function<bool(const Eigen::Matrix<int,6,1>& currBox)> func
    );

    float get_dis_value(float dis);
}

#endif