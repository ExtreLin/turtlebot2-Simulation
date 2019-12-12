// This is the KinectFusion Pipeline Implementation
// Author: Christian Diller, git@christian-diller.de

#include <kinectfusion.h>
#include <fstream>
#include<flannRadiusSearch.h>
#include<cuda/include/common.h>
#include<nextBestView.h>
using cv::cuda::GpuMat;
using Vec3fda = Eigen::Matrix<float, 3, 1, Eigen::DontAlign>;

namespace kinectfusion {

    Pipeline::Pipeline(const CameraParameters _camera_parameters,
                       const GlobalConfiguration _configuration) :
            camera_parameters(_camera_parameters), configuration(_configuration),
            volume(_configuration.volume_size, _configuration.voxel_scale),
            model_data(_configuration.num_levels, _camera_parameters),
            current_pose{}, poses{}, frame_id{0}, last_model_frame{}
    {
        // The pose starts in the middle of the cube, offset along z by the initial depth
        current_pose.setIdentity();
        current_pose(0, 3) = _configuration.volume_size.x / 2 * _configuration.voxel_scale;
        current_pose(1, 3) = _configuration.volume_size.y / 2 * _configuration.voxel_scale;
        current_pose(2, 3) = _configuration.volume_size.z / 2 * _configuration.voxel_scale - _configuration.init_depth;
        nbvTools.scanedVoxel = cv::Mat(_configuration.volume_size.y * _configuration.volume_size.z,_configuration.volume_size.x, CV_16S);
        Eigen::Vector4f   homogeneousCorr =   current_pose.inverse() * Eigen::Vector4f(0,0,0,1);
        nbvTools.oriCarPt =  Eigen::Vector3f(homogeneousCorr.x(),homogeneousCorr.y(),homogeneousCorr.z());
    } 
 
    bool Pipeline::process_frame(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map)
    {     
        // STEP 1: Surface measurement
        internal::FrameData frame_data = internal::surface_measurement(depth_map, camera_parameters,
                                                                       configuration.num_levels,
                                                                       configuration.depth_cutoff_distance,
                                                                       configuration.bfilter_kernel_size,
                                                                       configuration.bfilter_color_sigma,
                                                                       configuration.bfilter_spatial_sigma,
                                                                       configuration.clip_dis,
                                                                       configuration.depth_min_distance);
        frame_data.color_pyramid[0].upload(color_map);
        
        // STEP 2: Pose estimation
        bool icp_success { true };
        if (frame_id >0) { // Do not perform ICP for the very first frame      
            icp_success = internal::pose_estimation(current_pose, frame_data, model_data, camera_parameters,
                                                    configuration.num_levels,
                                                    configuration.distance_threshold, configuration.angle_threshold,
                                                    configuration.icp_iterations);
        }

        if (!icp_success)
            return false;
        poses.push_back(current_pose);

        // STEP 3: Surface reconstruction
        internal::cuda::surface_reconstruction(frame_data.depth_pyramid[0], 
                                               frame_data.color_pyramid[0],
                                               volume, camera_parameters, configuration.truncation_distance,
                                               configuration.depth_cutoff_distance,
                                               configuration.depth_min_distance,
                                               configuration.clip_dis,
                                               current_pose.inverse());
        //

        // Step 4: Surface prediction
        for (int level = 0; level < configuration.num_levels; ++level)
            internal::cuda::surface_prediction(volume, model_data.vertex_pyramid[level],
                                               model_data.normal_pyramid[level],
                                               model_data.color_pyramid[level],
                                               camera_parameters.level(level), configuration.truncation_distance,
                                               current_pose);

        
        if (configuration.use_output_frame) // Not using the output will speed up the processing
            model_data.color_pyramid[0].download(last_model_frame);

        ++frame_id;
  
        internal::cuda::uncertainty_map(volume, current_pose);
        internal::cuda::uncertainty_map_empty(frame_data.depth_pyramid[0], 
          configuration.depth_cutoff_distance,camera_parameters,
          volume,  current_pose);
        return true;
    }

    

    std::vector<Eigen::Matrix<float,7,1> > Pipeline::compute_paths()
    {
           SurfaceMesh surface_mesh =  extract_mesh();
            export_ply("/tmp/meshout.ply",surface_mesh);
            TriMesh tmesh;
            surfacemesh_to_TriMesh(surface_mesh,tmesh);
            //get_uncertainty_points();
            cv::Mat host_uncertainty_map; 
            cv::Mat host_tsdf_volume;
            std::vector<OpenMesh::Vec3f>  bdNormal;
            
            volume.uncertainty_volume.download(host_uncertainty_map);
            cv::Mat nlMap(host_uncertainty_map.rows,host_uncertainty_map.cols,CV_32S);
            nlMap.setTo(0);

            for(auto vh = tmesh.vertices_begin();  vh!= tmesh.vertices_end(); vh++)
            {
                if(!tmesh.is_boundary(*vh))
                    continue;

                OpenMesh::Vec3f& pt = tmesh.point((*vh));
                Eigen::Vector3i voxelPt (pt[0]/volume.voxel_scale,
                                                                  pt[1]/volume.voxel_scale,
                                                                  pt[2]/volume.voxel_scale );  
                 
                      
                OpenMesh::Vec3f   nl;
                tmesh.calc_vertex_normal_correct(vh,nl);
                if(nl[0]== 0 && nl[1] ==0&&nl[2] == 0  )
                    nl = OpenMesh::Vec3f (0,1,0);
                nl.normalize();
                nlMap.at<int>(voxelPt.z()*volume.volume_size.y + voxelPt.y(),voxelPt.x()) = bdNormal.size();
                bdNormal.push_back(nl);
                 //if(mapValue !=2)
                short& mapValue =  host_uncertainty_map.at<short>(voxelPt.z()*volume.volume_size.y + voxelPt.y(),voxelPt.x());
                mapValue = 3;            
            }


            // std::ofstream oput("map.asc");
            // for (size_t i = 0; i < volume.volume_size.x; i++)
            // {
            //     for (size_t j = 0; j < volume.volume_size.y; j++)
            //     {
            //         for (size_t k = 0; k < volume.volume_size.z; k++)
            //         {
            //             short  outValue =  host_uncertainty_map.at<short>(k*volume.volume_size.y + j,i);
            //             if(outValue !=3)
            //                 continue;
            //             oput<< i * volume.voxel_scale<<" "<<j *  volume.voxel_scale<<" "<<k *  volume.voxel_scale<<std::endl;
            //         }              
            //     }         
            // }
            // oput.close();
            
            cv::Mat u2dCandidateMap,u2dMapNormal,u2dMap;
            std::vector<Eigen::Vector2i> candidate_coord;
            std::vector<std::vector<Eigen::Matrix<float,7,1>>> cluster_candidate_coords;
            nextbestview:: get_2d_target_map(host_uncertainty_map,bdNormal, nlMap, volume.volume_size,candidate_coord,u2dMap ,u2dMapNormal);
            nextbestview::get_2d_candidate_map(
                host_uncertainty_map, candidate_coord,u2dMap,u2dMapNormal,
                volume.volume_size,volume.voxel_scale,u2dCandidateMap,cluster_candidate_coords);

            //volume.tsdf_volume.download(host_tsdf_volume);

            // cv::Mat validness_map = nextbestview::get_validness_map(host_uncertainty_map, volume.volume_size);
            // //std::ofstream oput("map.asc");
            // std::vector<std::pair<Eigen::Vector3i, float>> values;
            // nextbestview::get_uncertainty_priority_queue(host_uncertainty_map, host_tsdf_volume, volume.volume_size,values);
            // Eigen::Matrix<float,6,1> nbv =  nextbestview::find_next_best_view(validness_map, host_uncertainty_map,
              //  values, surface_mesh, poses.front().inverse(), volume.volume_size, volume.voxel_scale);
           // oput.close();
            internal::cuda::clear_candidate_points(volume);
            std::vector<Eigen::Matrix<float,7,1>> coords;
            for(int i=0;i<cluster_candidate_coords.size();++i)
            {
                coords.insert(coords.end(),cluster_candidate_coords[i].begin(),cluster_candidate_coords[i].end());
            }
            return  coords;
    }

    bool Pipeline::process_frame_by_rt(const cv::Mat_<float>& depth_map, const cv::Mat_<cv::Vec3b>& color_map, const Eigen::Matrix4f& rt )
    {
        Eigen::Vector3f t  = rt.block(0, 3, 3, 1);
        t = t + Eigen ::Vector3f(configuration.volume_size.x / 2 * configuration.voxel_scale,
                                                      configuration.volume_size.y / 2 * configuration.voxel_scale,
                                                      configuration.volume_size.z / 2 * configuration.voxel_scale) ;

        Eigen::Matrix4f   global_rt = rt;
        global_rt.block(0, 3, 3, 1) = t;

        // STEP 1: Surface measurement  获得扫描数据
        internal::FrameData frame_data = internal::surface_measurement(depth_map, camera_parameters,
                                                                       configuration.num_levels,
                                                                       configuration.depth_cutoff_distance,
                                                                       configuration.bfilter_kernel_size,
                                                                       configuration.bfilter_color_sigma,
                                                                       configuration.bfilter_spatial_sigma,
                                                                       configuration.clip_dis,
                                                                       configuration.depth_min_distance);
        frame_data.color_pyramid[0].upload(color_map);

        // Step 2: Surface prediction  通过射线从tsdf场中获得数据
        for (int level = 0; level < configuration.num_levels; ++level)
            internal::cuda::surface_prediction(volume, model_data.vertex_pyramid[level],
                                               model_data.normal_pyramid[level],
                                               model_data.color_pyramid[level],
                                               camera_parameters.level(level), configuration.truncation_distance,
                                               global_rt);
        
        // STEP 3: Pose estimation  两份数据拼接获得 rt
         current_pose= global_rt;
        internal::pose_estimation_robust_icp(current_pose, frame_data, model_data);

        poses.push_back(current_pose);
        // STEP 4: Surface reconstruction
        internal::cuda::surface_reconstruction(frame_data.depth_pyramid[0], 
                                               frame_data.color_pyramid[0],
                                               volume, camera_parameters, configuration.truncation_distance,
                                               configuration.depth_cutoff_distance,
                                               configuration.depth_min_distance,
                                               configuration.clip_dis,
                                               current_pose.inverse());

    }

    cv::Mat Pipeline::get_last_model_frame() const
    {
        if (configuration.use_output_frame)
            return last_model_frame;

        return cv::Mat(1, 1, CV_8UC1);
    }

    std::vector<Eigen::Matrix4f> Pipeline::get_poses() const
    {
        for (auto pose : poses)
            pose.block(0, 0, 3, 3) = pose.block(0, 0, 3, 3).inverse();
        return poses;
    }

    PointCloud Pipeline::extract_pointcloud() const
    {
        PointCloud cloud_data = internal::cuda::extract_points(volume, configuration.pointcloud_buffer_size);
        return cloud_data;
    }

    SurfaceMesh Pipeline::extract_mesh() const
    {
        SurfaceMesh surface_mesh = internal::cuda::marching_cubes(volume, configuration.triangles_buffer_size);
        return surface_mesh;
    }

    void Pipeline:: get_uncertainty_points() const
     {
       internal:: cuda::get_uncertainty_points_cuda(volume, camera_parameters);
     }

    void export_ply(const std::string& filename, const PointCloud& point_cloud)
    {
        std::ofstream file_out { filename };
        if (!file_out.is_open())
            return;

        file_out << "ply" << std::endl;
        file_out << "format ascii 1.0" << std::endl;
        file_out << "element vertex " << point_cloud.num_points << std::endl;
        file_out << "property float x" << std::endl;
        file_out << "property float y" << std::endl;
        file_out << "property float z" << std::endl;
        file_out << "property float nx" << std::endl;
        file_out << "property float ny" << std::endl;
        file_out << "property float nz" << std::endl;
        file_out << "property uchar red" << std::endl;
        file_out << "property uchar green" << std::endl;
        file_out << "property uchar blue" << std::endl;
        file_out << "end_header" << std::endl;

        for (int i = 0; i < point_cloud.num_points; ++i) {
            float3 vertex = point_cloud.vertices.ptr<float3>(0)[i];
            float3 normal = point_cloud.normals.ptr<float3>(0)[i];
            uchar3 color = point_cloud.color.ptr<uchar3>(0)[i];
            file_out << vertex.x << " " << vertex.y << " " << vertex.z << " " << normal.x << " " << normal.y << " "
                     << normal.z << " ";
            file_out << static_cast<int>(color.z) << " " << static_cast<int>(color.y) << " "
                     << static_cast<int>(color.x) << std::endl;
        }
    }

    void export_ply(const std::string& filename, const SurfaceMesh& surface_mesh)
    {
        std::ofstream file_out { filename };
        if (!file_out.is_open())
            return;

        file_out << "ply" << std::endl;
        file_out << "format ascii 1.0" << std::endl;
        file_out << "element vertex " << surface_mesh.num_vertices << std::endl;
        file_out << "property float x" << std::endl;
        file_out << "property float y" << std::endl;
        file_out << "property float z" << std::endl;
        file_out << "property uchar red" << std::endl;
        file_out << "property uchar green" << std::endl;
        file_out << "property uchar blue" << std::endl;
        file_out << "element face " << surface_mesh.num_triangles << std::endl;
        file_out << "property list uchar int vertex_index" << std::endl;
        file_out << "end_header" << std::endl;

        for (int v_idx = 0; v_idx < surface_mesh.num_vertices; ++v_idx) {
            float3 vertex = surface_mesh.triangles.ptr<float3>(0)[v_idx];
            uchar3 color = surface_mesh.colors.ptr<uchar3>(0)[v_idx];
            file_out << vertex.x << " " << vertex.y << " " << vertex.z << " ";
            file_out << (int) color.x << " " << (int) color.y << " " << (int) color.z << std::endl;
        }

        for (int t_idx = 0; t_idx < surface_mesh.num_vertices; t_idx += 3) {
            file_out << 3 << " " << t_idx + 1 << " " << t_idx << " " << t_idx + 2 << std::endl;
        }
    }
}