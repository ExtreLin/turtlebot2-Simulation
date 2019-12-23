// This is the CPU part of the ICP implementation
// Author: Christian Diller, git@christian-diller.de

#include <kinectfusion.h>
#include<robust_pcl_registration/point_cloud_registration.h>
#include<fstream>
#include<flannRadiusSearch.h>
#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/point_cloud.h>
#include<pcl-1.8/pcl/registration/icp.h>

using Matf31da = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;
using Matrix3frm = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>;

using point_cloud_registration::PointCloudRegistration;
using point_cloud_registration::PointCloudRegistrationParams;

typedef   pcl::PointCloud<pcl::PointXYZ> TCloud;

#define max_icp_size  2000

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

        void get_source_target_map(const TCloud&   sourceCloud, const TCloud& targetCloud, std::vector<int>& pair_map)
        {
            std::vector<Eigen::Vector3f> source_pts(sourceCloud.size());
            for(int i =0;i<source_pts.size();++i)
            {
                source_pts[i] = Eigen::Vector3f(
                    sourceCloud.points[i].x, sourceCloud.points[i].y,sourceCloud.points[i].z
                );
            }

            size_t minSize =std::min(std::min(sourceCloud.size(),targetCloud.size()),size_t(max_icp_size));
            CommonTools::FlannRadiusSearch<float> fnn(source_pts);
            std::vector<int> idxs; 
            std::vector<float>  dists;
            pair_map.resize(minSize,-1);
            for(int i=0;i<minSize;++i)
            {
                Eigen::Vector3f search_pt =   Eigen::Vector3f(
                    targetCloud.points[i].x, targetCloud.points[i].y,targetCloud.points[i].z);
                fnn.radius_search(search_pt,idxs,dists,50);
                if(idxs.empty())
                    continue;
                pair_map[i] = idxs[0];
            }   
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
            //std::ofstream   fo("tmp/fo.asc");
            //std::ofstream   mo("tmp/mo.asc");
            //在构造的时候先缩小100倍
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
                        pt.x = fvv.x()/100.0f; pt.y = fvv.y()/100.0f; pt.z = fvv.z()/100.0f;
                        sourceCloud.push_back(pt);
                        //fo<< fvv.x()<<" "<< fvv.y()<<" "<<fvv.z()<<std::endl;
                    }
                    
                    if(mv.x !=0 || mv.y !=0 ||mv.z != 0)
                    {
                        pt.x = mv.x/100.0f;  pt. y = mv.y/100.0f; pt.z = mv.z/100.0f;
                        targetCloud.push_back(pt);
                        //mo<<mv.x<<" "<<mv.y<<" "<<mv.z<<std::endl;
                    }       
                }
            }
            //fo.close();
            //mo.close();
           Eigen::Matrix4d rt; 
           rt.setIdentity();
           for(int i=0;i<5;++i)
           {
                //打乱顺序 target 往往数据量比 source 小，所以随机 target
                std::random_shuffle(targetCloud.points.begin(), targetCloud.points.end());
                //构造稀疏矩阵
                Eigen::SparseMatrix<int, Eigen::RowMajor> data_association(sourceCloud.size(), targetCloud.size());
                std::vector<Eigen::Triplet<int>> tripletList;
                size_t minSize =std::min(std::min(sourceCloud.size(),targetCloud.size()),size_t(max_icp_size));
                std::vector<int> pair_map;
                //查找每个顶点的最近点作为优化的最近点
                get_source_target_map(sourceCloud,targetCloud,pair_map);
                for (std::size_t i = 0; i < minSize; ++i)
                {
                    if(pair_map[i]<0)
                        continue;
                    tripletList.push_back(Eigen::Triplet<int>(pair_map[i],i, 1));
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
                options.max_num_iterations = 50;
                options.function_tolerance = 1e-6;
                options.num_threads = 8;
                ceres::Solver::Summary summary;
                //调用icp算法
                registration.solve(options, &summary);
                //得到拼接后的rt
                Eigen::Affine3d estimated_transform = registration.transformation();
                rt = estimated_transform.matrix()*rt;
                pcl::transformPointCloud (sourceCloud, sourceCloud, estimated_transform);
           }
           
            //将rt和原始rt相乘得到新的rt
            Eigen::MatrixX4f estimated = rt.cast<float>();
            pose= estimated * pose;
            rrr = pose.block(0, 0, 3, 3);
            //最后构造rt的时候放大100倍 
            ttt = (pose.block(0, 3, 3, 1)) * 100.0f;

            // std::ofstream   oo("tmp/out.asc");
            // for(int i=0;i<frameV.cols;++i)
            // {
            //     for(int j =0;j<frameV.rows;++j)
            //     {
            //         float3 fv =  frameV.at<float3>(j,i);
            //         Eigen::Vector3f fvv (fv.x, fv.y, fv.z);
            //         pcl::PointXYZ pt;
            //         if(fv.x !=0 || fv.y !=0 ||fv.z != 0)
            //         {
            //             fvv = rrr *  fvv  + ttt;
            //             pt.x = fvv.x(); pt.y = fvv.y(); pt.z = fvv.z();
            //             oo<< fvv.x()<<" "<< fvv.y()<<" "<<fvv.z()<<std::endl;
            //         }
            //     }
            // }
            // oo.close();
            //int a = 0;
        }     
    }
}