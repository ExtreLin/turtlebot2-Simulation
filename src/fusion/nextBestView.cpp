#include <nextBestView.h>
#include <fstream>
#include<flannRadiusSearch.h>
#include<cuda/include/common.h>

namespace nextbestview {

     cv::Mat get_validness_map(const cv::Mat& host_uncertainty_map, const int3& volume_size)
     {
        cv::Mat validness_map(volume_size.x,volume_size.z, CV_8UC1);
         //std::ofstream oput("map1.asc");
        int halfY= volume_size.y/2;
        for (size_t i = 0; i < volume_size.x; i++)
        {
            for (size_t k = 0; k < volume_size.z; k++)
            {
                bool needInsert = false;
                for (size_t j = 0; j < volume_size.y; j++)
                {
                    short  outValue =  host_uncertainty_map.at<short>(k*volume_size.y + j,i);
                    if(outValue==0&& abs(int(j)-halfY)  < 3)
                        needInsert = true;
                    else if(outValue>0)
                    {
                        needInsert = false;
                        break;
                    }
                }    
                validness_map.at<char>(i,k) =  needInsert? 1:0;
            }           
        }
        return validness_map;
        //oput.close();
     }

    void  get_uncertainty_priority_queue(const cv::Mat& host_uncertainty_map, 
                                                                                    const cv::Mat& host_tsdf_volume,
                                                                                    const int3& volume_size,
                                                                                    std::vector<std::pair<Eigen::Vector3i, float>>& values)
    {
        values.clear();
        for (size_t i = 0; i < volume_size.x; i++)
        {
           for (size_t j = 0; j < volume_size.y; j++)
           {
               for (size_t k = 0; k < volume_size.z; k++)
               {
                   short  outValue =  host_uncertainty_map.at<short>(k*volume_size.y + j,i);
                    if(outValue!=3)
                        continue;

                    short2 tsdf = host_tsdf_volume.at<short2>(k*volume_size.y + j,i);
                    if(tsdf.x == 0)
                        values.push_back(std::make_pair(Eigen::Vector3i(i,j,k),1));
                    else
                        values.push_back(std::make_pair(Eigen::Vector3i(i,j,k),float(tsdf.y) /MAX_WEIGHT));
                }        
             }           
         }
        std::sort(values.begin(),values.end(),[](
            const std::pair<Eigen::Vector3i,float>& lhs, const std::pair<Eigen::Vector3i, float>& rhs){
            return lhs.second > rhs.second;
        });
    }


    Eigen::Vector3f  find_next_best_view(const cv::Mat& validness_map,
                                                        const cv::Mat& host_uncertainty_map, 
                                                        const std::vector<std::pair<Eigen::Vector3i, float>>& values,
                                                        const kinectfusion::SurfaceMesh surface_mesh,
                                                        const Eigen::Matrix4f& primary_model_view,
                                                        const int3& volume_size,
                                                        const float voxel_scale)
    {
        //获得收益值最高的体块的体素位置
        Eigen::Vector2i target_pos =Eigen::Vector2i( values.front().first.x(),values.front().first.z());
        Eigen::Vector3f target_pos_3d =(values.front().first).cast<float>();
                
        //重建出模型中面片的中心点和法向
        std::vector<Eigen::Vector3f> pts;
        std::vector<Eigen::Vector3f> nls;

        //获得目标最近的面片的法向
        for (int v_idx = 0; v_idx < surface_mesh.num_vertices; v_idx+=3) {
            const float3& v0 = surface_mesh.triangles.ptr<float3>(0)[v_idx];
            const float3& v1 = surface_mesh.triangles.ptr<float3>(0)[v_idx + 1];
            const float3& v2 = surface_mesh.triangles.ptr<float3>(0)[v_idx + 2];
            //中心点
            pts.push_back(Eigen::Vector3f(
                v0.x + v1.x + v2.x,
                v0.y + v1.y + v2.y,
                v0.z + v1.z + v2.z)/3);
                    
            Eigen::Vector3f fnl = Eigen::Vector3f (
                v1.x - v0.x, v1.y-v0.y, v1.z - v0.z).cross(Eigen::Vector3f(
                v2.x - v0.x, v2.y-v0.y, v2.z - v0.z ) );

            fnl.normalize();
            //面片的法向
            nls.push_back(std::move(fnl));
        }
        //建立树
        CommonTools::FlannRadiusSearch<float>  frs (pts);
        //用目标位置查找最近面片的法向
        Eigen::Matrix<float, 3, 3> rotation = primary_model_view.block(0, 0, 3, 3);
        Eigen::Vector3f translation = primary_model_view.block(0, 3, 3, 1);
        Eigen::Vector3f camera_target_pos = rotation * target_pos_3d + translation;
        std::vector<int>  idxs;
        std::vector<float>  dists;
        frs.radius_search(camera_target_pos,idxs,dists,4*voxel_scale);
        Eigen::Vector3f face_normal (0,0,-1);
        if(!idxs.empty())
                face_normal = nls[idxs[0]];

        //开始求解每个位置的收益值
        std::vector<std::vector<std::vector<float>>>  view_map(
        validness_map.cols, std::vector<std::vector<float>>(
        validness_map.rows,std::vector<float>(
        72,0)));

#pragma omp parallel for schedule(dynamic)
        for (size_t m = 0; m < validness_map.cols; m++)
        {
            for (size_t n = 0; n < validness_map.rows; n++)
            {
                //判断当前点在validness_map中是否可达
                if(!validness_map.at<char>(m,n))
                    continue;

                //在当前点上每5度历遍方向
                Eigen::Vector2f dir2d = (target_pos - Eigen::Vector2i(m,n)).cast<float>();
                Eigen::Vector3f dir3d(dir2d.x(), dir2d.y(), 0);
                dir3d.normalize();
                Eigen::Vector3f ori (0, 1, 0);
                Eigen::Vector3f crossd = ori.cross(dir3d);
                int t = 1;
                if(crossd.z() < 0)
                    t=-1;
                float dotValue = ori.dot(dir3d);
                float angle = acos(dotValue)/M_PI *180;

                for (size_t i = 0; i < 72; i++)
                {
                    float currAngle =  abs(angle*t-i*5);
                    while (currAngle > 180)
                    {
                        currAngle = currAngle - 180;
                    }
                    //如果方向和目标方向角度大于5度则不考虑
                    if(currAngle>5)
                        continue;
                            
                        //三维DDA算法，判断是否遮挡
                    Eigen::Vector3f start_pt (m, volume_size.y/2, n);
                    bool  need_compute = true;
                    threeD_DDA(start_pt, target_pos_3d,
                    [&](const Eigen::Matrix<int,6,1>& currBox)->bool{
                        short  outValue =  host_uncertainty_map.at<short>(currBox[2]*volume_size.y + currBox[1], currBox[0]);
                        //判断是否是障碍点,如果是的话，表示提前碰到障碍点，目标点被遮挡
                        if(outValue == 1)
                        {
                            need_compute = false;
                            return true;
                        }  
                        return false;
                    });
                    //目标点没有遮挡
                    if(!need_compute)
                        continue;
                            
                    Eigen::Vector3f view_dir =  start_pt - target_pos_3d;
                    float dis =  view_dir.norm() *voxel_scale ;
                    view_dir.normalize();
                    //计算当前位置的收益值
                    float av = acos(face_normal.dot(view_dir));
                    view_map[m][n][i] = get_dis_value(dis) *pow(M_E,-(av*av/0.36)); //0.36 means 0.6^2
                }          
            }
        }               
        float maxValue = 0;
        Eigen::Vector3f nbv(0,0,0);
            //取map中最大值作为nbv
        for (size_t m = 0; m < validness_map.cols; m++)
        {
            for (size_t n = 0; n < validness_map.rows; n++)
            {
                for (size_t i = 0; i < 72; i++)
                {
                    if(view_map[m][n][i] <= maxValue)
                        continue;
                    maxValue = view_map[m][n][i];
                    nbv = Eigen::Vector3f(m, volume_size.y/2, n);
                }
            }
        }     
        //转化为世界坐标系中的点 
         return rotation * nbv + translation;
    }

    float get_dis_value(float dis)
    {
        if(dis>6000)
            return 0;
        if(dis>=1000&&dis<=3000)
            return 1;
        if(dis< 1000)
            return dis/1000;
        else
            return(6000 - dis )/3000;         
    }

    void threeD_DDA(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                std::function<bool(const Eigen::Matrix<int,6,1>& currBox)> func
    )
    {
        Eigen::Vector3f dir = end_pt - start_pt;
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
        Eigen::Matrix<int,6,1 >  currBox;
        currBox<<start_pt.x(), start_pt.y(), start_pt.z(), start_pt.x()+1, start_pt.y()+1, start_pt.z()+1;
        Vec3fda  currPt = start_pt;

        float tsum = 0;//用于终止
        do
        {
            if(func(currBox))
                break;
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