#ifndef NEXTBESTVIEW_H
#define NEXTBESTVIEW_H

#include "data_types.h"

namespace nextbestview {

    struct NextBestViewTools
    {
        cv::Mat  scanedVoxel;
        Eigen::Vector3f  oriCarPt;
    };
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


    Eigen::Matrix<float,6,1>  find_next_best_view(const cv::Mat& validness_map,
                                                           const cv::Mat& host_uncertainty_map, 
                                                           const std::vector<std::pair<Eigen::Vector3i, float>>& values,
                                                           const kinectfusion::SurfaceMesh surface_mesh,
                                                           const Eigen::Matrix4f& primary_model_view,
                                                           const int3& volume_size,
                                                           const float voxel_scale);
    
    void threeD_DDA(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                    std::function<bool(const Eigen::Matrix<int,6,1>& currBox)> func
    );

    void twoD_DDA(const Eigen::Vector2f& start_pt, const Eigen::Vector2f& end_pt,
                    std::function<bool(const Eigen::Vector4i& currBox)> func);

    float get_dis_value(float dis);

    uchar3  value_to_color(float  value);

    void  get_2d_target_map(const cv::Mat& host_uncertainty_map, 
                                                                    const std::vector<OpenMesh::Vec3f> bdNoraml,
                                                                    const  cv::Mat& nlMap,
                                                                    const int3& volume_size, 
                                                                    std::vector<Eigen::Vector2i>& target_coord,
                                                                    cv::Mat& u2dMap,
                                                                    cv::Mat& u2dMapNormal);

    void  get_2d_candidate_map(const cv::Mat& host_uncertainty_map, 
                                                                  const  std::vector<Eigen::Vector2i>& target_coord,
                                                                  const cv::Mat& u2dMap,
                                                                  const cv::Mat& u2dMapNormal, 
                                                                  const int3& volume_size, 
                                                                  const float& voxel_scale,
                                                                  cv::Mat& u2dCandidateMap,
                                                                  std::vector<std::vector<Eigen::Matrix<float,7,1>>>& cluster_candidate_coords);

    void get_pose_by_map(const std::vector<Eigen::Vector2i>& input_coords, 
                                                      const Eigen::Vector2i& center, 
                                                      const float& voxel_scale,
                                                      std::vector<Eigen::Matrix<float,7,1>>& poses);

}

#endif