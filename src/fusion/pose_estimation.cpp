// This is the CPU part of the ICP implementation
// Author: Christian Diller, git@christian-diller.de

#include <kinectfusion.h>

using Matf31da = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
using Matrix3frm = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;

#include<robust_pcl_registration/point_cloud_registration.h>

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

typedef   pcl::PointCloud<pcl::PointXYZ> TCloud;

namespace kinectfusion {
    namespace internal {

        namespace cuda { // Forward declare CUDA functions
            void estimate_step(const Eigen::Matrix3f& rotation_current, const Matf31da& translation_current,
                               const cv::cuda::GpuMat& vertex_map_current, const cv::cuda::GpuMat& normal_map_current,
                               const Eigen::Matrix3f& rotation_previous_inv, const Matf31da& translation_previous,
                               const CameraParameters& cam_params,
                               const cv::cuda::GpuMat& vertex_map_previous, const cv::cuda::GpuMat& normal_map_previous,
                               float distance_threshold, float angle_threshold,
                               Eigen::Matrix<double, 6, 6, Eigen::RowMajor>& A, Eigen::Matrix<double, 6, 1>& b);
        }

        bool pose_estimation(Eigen::Matrix4f& pose,
                             const FrameData& frame_data,
                             const ModelData& model_data,
                             const CameraParameters& cam_params,
                             const int pyramid_height,
                             const float distance_threshold, const float angle_threshold,
                             const std::vector<int>& iterations)
        {
            // Get initial rotation and translation
            Eigen::Matrix3f current_global_rotation = pose.block(0, 0, 3, 3);
            Eigen::Vector3f current_global_translation = pose.block(0, 3, 3, 1);

            Eigen::Matrix3f previous_global_rotation_inverse(current_global_rotation.inverse());
            Eigen::Vector3f previous_global_translation = pose.block(0, 3, 3, 1);

            // ICP loop, from coarse to sparse
            for (int level = pyramid_height - 1; level >= 0; --level) {
                for (int iteration = 0; iteration < iterations[level]; ++iteration) {
                    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> A {};
                    Eigen::Matrix<double, 6, 1> b {};
                        
                    // Estimate one step on the CPU
                    cuda::estimate_step(current_global_rotation, current_global_translation,
                                        frame_data.vertex_pyramid[level], frame_data.normal_pyramid[level],
                                        previous_global_rotation_inverse, previous_global_translation,
                                        cam_params.level(level),
                                        model_data.vertex_pyramid[level], model_data.normal_pyramid[level],
                                        distance_threshold, sinf(angle_threshold * 3.14159254f / 180.f),
                                        A, b);

                    // Solve equation to get alpha, beta and gamma
                    double det = A.determinant();
                    if (fabs(det) < 10000 || std::isnan(det))
                        return false;
                                            
                    Eigen::Matrix<float, 6, 1> result { A.fullPivLu().solve(b).cast<float>() };
                    float alpha = result(0);
                    float beta = result(1);
                    float gamma = result(2);

                    // Update rotation
                    auto camera_rotation_incremental(
                            Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ()) *
                            Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY()) *
                            Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
                    auto camera_translation_incremental = result.tail<3>();

                    // Update translation
                    current_global_translation =
                            camera_rotation_incremental * current_global_translation + camera_translation_incremental;
                    current_global_rotation = camera_rotation_incremental * current_global_rotation;
                }
            }

            // Return the new pose
            pose.block(0, 0, 3, 3) = current_global_rotation;
            pose.block(0, 3, 3, 1) = current_global_translation;

            return true;
        }


        void pose_estimation_robust_icp(Eigen::Matrix4f& pose,
                             const FrameData& frame_data,
                             const ModelData& model_data)
        {
              cv::Mat  frameV ,modelV;
            frame_data.vertex_pyramid[0].download(frameV);
            model_data.vertex_pyramid[0].download(modelV);

            //构造pcl ·cloud
            TCloud   sourceCloud, targetCloud;
            //
            Eigen::Matrix3f rrr = pose.block(0, 0, 3, 3);
            Eigen::Vector3f ttt = pose.block(0, 3, 3, 1);
            //std::ofstream   fo("/tmp/fo.asc");
            //std::ofstream   mo("/tmp/mo.asc");
            for(int i=0;i<frameV.cols;++i)
            {
                for(int j =0;j<frameV.rows;++j)
                {
                    float3 fv =  frameV.at<float3>(j,i);
                    Eigen::Vector3f fvv (fv.x, fv.y, fv.z);
                    float3 mv = modelV.at<float3>(j,i);
                    pcl::PointXYZ pt;
                    if(fv.x !=0 || fv.y !=0 ||fv.z != 0)
                    {
                        fvv = rrr *  fvv  + ttt;
                        pt.x = fvv.x(); pt.y = fvv.y(); pt.z = fvv.z();
                        sourceCloud.push_back(pt);
                        //fo<< fvv.x()<<" "<< fvv.y()<<" "<<fvv.z()<<std::endl;
                    }
                    
                    if(mv.x !=0 || mv.y !=0 ||mv.z != 0)
                    {
                        pt.x = mv.x;  pt. y = mv.y; pt.z = mv.z;
                        targetCloud.push_back(pt);
                        //mo<<mv.x<<" "<<mv.y<<" "<<mv.z<<std::endl;
                    }       
                }
            }
            //构造稀疏矩阵
            Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(sourceCloud.size(), targetCloud.size());
            std::vector<Eigen::Triplet<int>> tripletList;
            size_t minSize =  sourceCloud.size() < targetCloud.size()?  sourceCloud.size(): targetCloud.size();
            for (std::size_t i = 0; i < minSize; ++i)
            {
                tripletList.push_back(Eigen::Triplet<int>(i, i, 1));
            }
            data_association.setFromTriplets(tripletList.begin(), tripletList.end());
            data_association.makeCompressed();
            //设置icp参数
            PointCloudRegistrationParams params;
            params.dof = std::numeric_limits<double>::infinity();
            params.max_neighbours = 3;
            params.dimension = 3;
            PointCloudRegistration registration(sourceCloud, targetCloud, data_association, params);
            //设置ceres 非线性优化器的参数
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.use_nonmonotonic_steps = true;
            options.minimizer_progress_to_stdout = false;
            options.max_num_iterations = std::numeric_limits<int>::max();
            options.function_tolerance = 1;
            options.num_threads = 8;
            ceres::Solver::Summary summary;
            //调用icp算法
            registration.solve(options, &summary);
            //得到拼接后的rt
            auto estimated_transform = registration.transformation();
            //将rt和原始rt相乘得到新的rt
            Eigen::MatrixX4f estimated = estimated_transform.matrix().cast<float>();
            pose= estimated * pose;
            //fo.close();
            //mo.close();
        }     
    }
}