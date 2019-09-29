#include "include/common.h"
#include "iostream"

using Vec2ida = Eigen::Matrix<int, 2, 1, Eigen::DontAlign>;
using Vec6ida = Eigen::Matrix<int, 6, 1, Eigen::DontAlign>;
using Vec3ida = Eigen::Matrix<int, 3, 1, Eigen::DontAlign>;

namespace kinectfusion {
    namespace internal {
        namespace cuda {

            __global__
            void update_uncertainty_kernel(
                                    PtrStepSz<short> uncertainty_volume,
                                    int3 volume_size, 
                                    const Vec3fda camera_pos)
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= volume_size.x || y >= volume_size.y)
                    return;

                for (int z = 0; z < volume_size.z; ++z) {
                    if(uncertainty_volume.ptr(z * volume_size.y + y)[x] !=2)
                        continue;

                    //uncertainty_volume.ptr(z * volume_size.y + y)[x]  = -2;
                    //射线求线段穿过的体素块
                   Vec3fda startPt(float(x)+0.5,float(y)+0.5,float(z)+0.5);
                   Vec3fda dir = camera_pos - startPt;

                   //根据方向获得可能相交的三个面
                   int xi, yi ,zi , nx, ny, nz;
                   xi = 0; yi=1; zi=2; nx = ny = nz = -1;

                   if(dir.x()>0)
                   {
                        xi = 3;
                        nx = 1;
                   }

                   if(dir.y()>0)
                   {
                        yi = 4;
                        ny = 1;
                   }

                    if(dir.z()>0)
                   {
                        zi = 5;
                        nz = 1;
                   }
             
                   //三维DDA
                   Vec6ida  currBox;
                   currBox<<x, y, z, x+1, y+1, z+1;
                   Vec3fda  currPt = startPt;

                   float tsum = 0;//用于终止
                   int  boxCount = 0;
                   do{

                        if (boxCount>4)
                            uncertainty_volume.ptr(currBox[2]* volume_size.y  +currBox[1] )[currBox[0]] = 0;
                        boxCount++;

                        float tx = abs((currBox[xi] - currPt.x())/dir.x());
                        float ty = abs((currBox[yi] - currPt.y())/dir.y());
                        float tz = abs((currBox[zi] - currPt.z())/dir.z());

                        int tnx = nx;
                        int tny = ny;
                        int tnz = nz;
                    
                        float t = tx ;
                        if(ty < tx )
                        {
                            tnx = 0;
                            t = ty;
                        }
                        else
                            tny = 0;

                        if(tz < t)
                        {
                            tnx = tny =0;
                            t = tz;
                        }
                        else
                            tnz = 0;
                         

                        currPt = currPt + dir*(t);

                        currBox[0] = currBox[0] + tnx;
                        currBox[1] = currBox[1] + tny;
                        currBox[2] =  currBox[2] + tnz;
                        currBox[3] =  currBox[3] + tnx;
                        currBox[4] =  currBox[4] + tny;
                        currBox[5] =  currBox[5] + tnz;
                        
                        tsum += t;

                        if(tsum>1.0)
                            break;
                        
                   }while(1);
                }
            }

              void uncertainty_map(
                                        VolumeData& volume,
                                        const Eigen::Matrix4f& model_view
            )
            {
                Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation =   model_view.block(0, 0, 3, 3);
                Vec3fda translation = model_view.block(0, 3, 3, 1);
                const Vec3fda camera_pos = (rotation * Vec3fda(0,0,0)  + translation)/ volume.voxel_scale;
                const Vec3ida camera_pos_int(int(camera_pos.x()), int(camera_pos.y()), int(camera_pos.z()));

                const dim3 threads(32, 32);
                const dim3 blocks((volume.volume_size.x + threads.x - 1) / threads.x,
                                  (volume.volume_size.y + threads.y - 1) / threads.y);
                                  
                update_uncertainty_kernel<<<blocks, threads>>>(
                    volume.uncertainty_volume,
                    volume.volume_size, 
                    camera_pos
                );
                
                cudaThreadSynchronize();
            }    
            
            __global__
            void update_uncertainty_empty_kernel(
                                    const PtrStepSz<float> depth_image,
                                    const float depth_cutoff_distance,
                                    CameraParameters cam_params,
                                    PtrStepSz<short> uncertainty_volume,
                                    int3 volume_size, 
                                    const Vec3fda camera_pos,
                                    Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation, 
                                    Vec3fda translation, const float voxel_scale)
            {
                const int u = blockIdx.x * blockDim.x + threadIdx.x;
                const int v = blockIdx.y * blockDim.y + threadIdx.y;

                if (u >= depth_image.cols || v >= depth_image.rows)
                    return;

                if(depth_image.ptr(v)[u]!=0)
                    return;
                
               Vec3fda xylambda(
                            (u- cam_params.principal_x) / cam_params.focal_x,
                            (v- cam_params.principal_y) / cam_params.focal_y,
                            1.f);
                
                xylambda.normalize();

                xylambda = xylambda * depth_cutoff_distance;
                Vec3fda startPt = (rotation*(xylambda)+translation)/voxel_scale;

                int x = int(startPt.x());
                int y = int(startPt.y());
                int z = int(startPt.z());


                //射线求线段穿过的体素块
               
                Vec3fda dir = camera_pos - startPt;

                //根据方向获得可能相交的三个面
                int xi, yi ,zi , nx, ny, nz;
                xi = 0; yi=1; zi=2; nx = ny = nz = -1;

                if(dir.x()>0)
                {
                        xi = 3;
                        nx = 1;
                 }

                if(dir.y()>0)
                {
                    yi = 4;
                    ny = 1;
                 }

                if(dir.z()>0)
                {
                    zi = 5;
                    nz = 1;
                }
             
                //三维DDA 
                Vec6ida  currBox;
                currBox<<x, y, z, x+1, y+1, z+1;
                Vec3fda  currPt = startPt;

                float tsum = 0;//用于终止
                do{
                    
                    if(currBox[0]>=0&&currBox[1]>=0&&currBox[2]>=0&&
                        currBox[0]<volume_size.x&&currBox[1]<volume_size.y&&currBox[2]<volume_size.z)
                   {
                        if(uncertainty_volume.ptr(currBox[2]* volume_size.y  +currBox[1] )[currBox[0]] == -1 )
                            uncertainty_volume.ptr(currBox[2]* volume_size.y  +currBox[1] )[currBox[0]] = 0;
                    }

                    float tx = abs((currBox[xi] - currPt.x())/dir.x());
                    float ty = abs((currBox[yi] - currPt.y())/dir.y());
                    float tz = abs((currBox[zi] - currPt.z())/dir.z());

                    int tnx = nx;
                    int tny = ny;
                    int tnz = nz;
                    
                    float t = tx ;
                    if(ty < tx )
                    {
                        tnx = 0;
                        t = ty;
                    }
                    else
                        tny = 0;

                    if(tz < t)
                    {
                        tnx = tny =0;
                        t = tz;
                    }
                    else
                        tnz = 0;
                         
                    currPt = currPt + dir*(t);

                    currBox[0] = currBox[0] + tnx;
                    currBox[1] = currBox[1] + tny;
                    currBox[2] =  currBox[2] + tnz;
                    currBox[3] =  currBox[3] + tnx;
                    currBox[4] =  currBox[4] + tny;
                    currBox[5] =  currBox[5] + tnz;
                        
                    tsum += t;

                    if(tsum>1.0)  
                        break;
                        
                }while(1);
            }

            void uncertainty_map_empty (const cv::cuda::GpuMat& depth_image, 
                                                                           const float depth_cutoff_distance,
                                                                           const CameraParameters& camera_params,
                                                                           VolumeData& volume,
                                                                           const Eigen::Matrix4f& model_view)
            {
                Eigen::Matrix<float, 3, 3, Eigen::DontAlign> rotation =   model_view.block(0, 0, 3, 3);
                Vec3fda translation = model_view.block(0, 3, 3, 1);
                const Vec3fda camera_pos = (rotation * Vec3fda(0,0,0)  + translation)/ volume.voxel_scale;

                const dim3 threads(32, 32);
                const dim3 blocks((camera_params.image_width + threads.x - 1) / threads.x,
                                  (camera_params.image_height + threads.y - 1) / threads.y);
                
                update_uncertainty_empty_kernel<<<blocks, threads>>>(
                    depth_image, 
                    depth_cutoff_distance,
                    camera_params,
                    volume.uncertainty_volume,
                    volume.volume_size, 
                    camera_pos,
                    model_view.block(0, 0, 3, 3),
                    model_view.block(0, 3, 3, 1),
                    volume.voxel_scale
                );         
                cudaThreadSynchronize();     
            }  

              __global__
            void  extend_mesh_voxel(
                PtrStepSz<short> uncertainty_volume,
                PtrStepSz<short3> offsetMat,
                int offset_size,
                int3 volume_size
            )
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= volume_size.x || y >= volume_size.y)
                    return;

                 for(int z =0; z < volume_size.z; ++z )
                {
                      if(uncertainty_volume.ptr(z * volume_size.y + y)[x]!=1)
                        continue;

                    for(int i=0; i < offset_size; ++i)
                    {
                        int tx = x + offsetMat.ptr(0)[i].x;
                        int ty = y + offsetMat.ptr(0)[i].y;
                        int tz = z + offsetMat.ptr(0)[i].z;

                        if(tx >= volume_size.x || ty >= volume_size.y|| tz >= volume_size.z||
                            tx<0 || ty<0 || tz<0 )
                            continue;

                        if(uncertainty_volume.ptr(tz * volume_size.y + ty)[tx]==0||
                            uncertainty_volume.ptr(tz * volume_size.y + ty)[tx]==-1)
                            uncertainty_volume.ptr(tz * volume_size.y + ty)[tx]=2;
                    }
                }
            }

            __global__
            void  mark_uncertainty_voxels(
                PtrStepSz<short> uncertainty_volume,
                PtrStepSz<short3> offsetMat,
                int offset_size,
                int3 volume_size
            )
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= volume_size.x || y >= volume_size.y)
                    return;

                for(int z =0; z < volume_size.z; ++z )
                {
                    if(uncertainty_volume.ptr(z * volume_size.y + y)[x]!=2)
                        continue;
                        
                    //扩散查找
                    int value = 0;
                    bool hasUnKown = false;
                    for(int i=0; i < offset_size; ++i)
                    {
                        int tx = x + offsetMat.ptr(0)[i].x;
                        int ty = y + offsetMat.ptr(0)[i].y;
                        int tz = z + offsetMat.ptr(0)[i].z;

                        if(tx >= volume_size.x || ty >= volume_size.y || tz >= volume_size.z||
                            tx<0 || ty<0 || tz<0 )
                            continue;

                        if(uncertainty_volume.ptr(tz * volume_size.y + ty)[tx] >=0)
                            value |=  (1<<uncertainty_volume.ptr(tz * volume_size.y + ty)[tx]);
                        else
                            hasUnKown = true;
                    }

                    if((value != 5)||(!hasUnKown))
                        continue;
                    uncertainty_volume.ptr(z * volume_size.y + y)[x] = 3;
                }
            }

            std::vector<Eigen::Vector3f>  get_uncertainty_points_cuda( const VolumeData& volume, 
                                                                                                                                        const CameraParameters& camera_params)
            {
                  GpuMat  offsetMat; 
                std::vector<short3> offsetMat_host;

                for(short i= -1; i<=1; ++i)
                {
                       for(short j= -1; j<=1; ++j)
                       {
                            for(short k= -1;  k<=1; ++k)
                            {
                                offsetMat_host.push_back(make_short3(i,j,k));
                            }
                       } 
                }

                offsetMat = cv::cuda::createContinuous(1, offsetMat_host.size(), CV_16SC3);
                offsetMat.upload(cv::Mat(1, offsetMat_host.size(), CV_16SC3, offsetMat_host.data(), cv::Mat::AUTO_STEP));

                GpuMat  offsetMatMore; 
                std::vector<short3> offsetMatMore_host;

                for(short i= -2; i<=2; ++i)
                {
                       for(short j= -2; j<=2; ++j)
                       {
                            for(short k= -2;  k<=2; ++k)
                            {
                                offsetMatMore_host.push_back(make_short3(i,j,k));
                            }
                       } 
                }

                offsetMatMore = cv::cuda::createContinuous(1, offsetMatMore_host.size(), CV_16SC3);
                offsetMatMore.upload(cv::Mat(1, offsetMatMore_host.size(), CV_16SC3, offsetMatMore_host.data(), cv::Mat::AUTO_STEP));


                const dim3 threads(32, 32);
                const dim3 blocks((camera_params.image_width + threads.x - 1) / threads.x,
                                  (camera_params.image_height + threads.y - 1) / threads.y);

                extend_mesh_voxel<<<blocks, threads>>>(volume.uncertainty_volume, offsetMatMore, offsetMatMore_host.size() , volume.volume_size);

                mark_uncertainty_voxels<<<blocks, threads>>>(volume.uncertainty_volume, offsetMat, offsetMat_host.size() , volume.volume_size);
                return std::vector<Eigen::Vector3f>();
            }

            __global__
            void clear_candidate_points_cuda(
                PtrStepSz<short> uncertainty_volume,
                int3 volume_size
            )
            {
                const int x = blockIdx.x * blockDim.x + threadIdx.x;
                const int y = blockIdx.y * blockDim.y + threadIdx.y;

                if (x >= volume_size.x || y >= volume_size.y)
                    return;

                for(int z =0; z < volume_size.z; ++z )
                {
                    if(uncertainty_volume.ptr(z * volume_size.y + y)[x]==3)
                        uncertainty_volume.ptr(z * volume_size.y + y)[x] = 2;
                }
            }


            void clear_candidate_points(VolumeData& volume)
            {
                const dim3 threads(32, 32);
                const dim3 blocks((volume.volume_size.x + threads.x - 1) / threads.x,
                                  (volume.volume_size.y + threads.y - 1) / threads.y);
                
                clear_candidate_points_cuda<<<blocks, threads>>>(
                    volume.uncertainty_volume,
                    volume.volume_size
                );       
            }

            
        } 
    }
}
