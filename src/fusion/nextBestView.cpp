#include <nextBestView.h>
#include <fstream>
#include<flannRadiusSearch.h>
#include<cuda/include/common.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>

namespace nextbestview {

     cv::Mat get_validness_map(const cv::Mat& host_uncertainty_map, const int3& volume_size)
     {
        cv::Mat validness_map(volume_size.x,volume_size.z, CV_8UC1);
        cv::Mat obstacle_map(volume_size.x,volume_size.z, CV_8UC1);
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

                bool hasObstacle = false;
                for (size_t j = 0; j < volume_size.y; j++)
                {
                    short  outValue =  host_uncertainty_map.at<short>(k*volume_size.y + j,i);
                    if(outValue==1)
                    {
                        hasObstacle = true;
                        break;
                    }
                }   
                obstacle_map.at<char>(i,k) =  hasObstacle? 1:0;          
            }           
        }
        //扩散范围
        std::vector<Eigen::Vector2i> pts;
        for(int i = -14;i<14;++i)
        {
            for(int j = -14;j<14;++j)
            {
                pts.push_back(Eigen::Vector2i(i,j));
            }
        }
        //扩散
        for (size_t i = 0; i < volume_size.x; i++)
        {
            for (size_t k = 0; k < volume_size.z; k++)
            {
                if(!obstacle_map.at<char>(i,k) )
                    continue;
                
                for(int sz =0;sz < pts.size();++sz)
                {
                    int m = i+pts[sz].x();
                    int n = k+pts[sz].y();
                    if(m<0||m>=volume_size.x||n<0||n>=volume_size.z)
                        continue;
                        
                    validness_map.at<char>(m,n) = 0;
                }
            }
        }

        cv::Mat image(volume_size.x, volume_size.z,CV_8UC3);
        for (size_t i = 0; i < volume_size.x; i++)
        {
            for (size_t k = 0; k < volume_size.z; k++)
            {
                 if(validness_map.at<char>(i,k))
                 {
                    image.at<uchar3>(i,k) = value_to_color(1);
                 }
                 else
                 {
                    image.at<uchar3>(i,k) = value_to_color(0);
                 }     
            }
        }
        cv::imwrite("out3.bmp",image); 
        return validness_map;
     }

    void  get_uncertainty_priority_queue(const cv::Mat& host_uncertainty_map, 
                                                                                   const cv::Mat& host_tsdf_volume,
                                                                                   const int3& volume_size,
                                                                                   std::vector<std::pair<Eigen::Vector3i, float>>& values)
    {
        values.clear();
        for (size_t i = 0; i < volume_size.x; i++)
        {
           for (size_t j = 0; j < volume_size.y/2; j++)
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

        std::vector<short>  tsdfs;
        cv::Mat image(volume_size.x, volume_size.z,CV_8UC3);
        for (size_t i = 0; i < volume_size.x; i++)
        {
            for (size_t k = 0; k < volume_size.z; k++)
            {
                float maxValue =0;
                for (size_t j = 0; j < volume_size.y; j++)
                {    
                   //short  outValue =  host_uncertainty_map.at<short>(k*volume_size.y + j,i);
                    //if(outValue!=3)
                        //continue;

                    short2 tsdf = host_tsdf_volume.at<short2>(k*volume_size.y + j,i);
                    tsdfs.push_back(tsdf.x);
                    float tmpValue = 0;
                    if(tsdf.x > SHORTMAX -10||tsdf.x == 0||tsdf.x < -SHORTMAX-10)
                        tmpValue = 0;
                    else
                        tmpValue  =1 ;

                    if(tmpValue> maxValue)
                        maxValue = tmpValue;
                }    

                image.at<uchar3>(i,k) = value_to_color(maxValue);
            }           
         }
         std::sort(tsdfs.begin(),tsdfs.end());
         cv::imwrite("out2.bmp",image); 
    }


   Eigen::Matrix<float,6,1> find_next_best_view(const cv::Mat& validness_map,
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

        Eigen::Matrix<float, 3, 3> rotation = primary_model_view.block(0, 0, 3, 3);
        Eigen::Vector3f translation = primary_model_view.block(0, 3, 3, 1);

        //重建出模型中面片的中心点和法向
        std::vector<Eigen::Vector3f> pts;
        std::vector<Eigen::Vector3f> nls;
        
        //获得目标最近的面片的法向
        for (int v_idx = 0; v_idx < surface_mesh.num_vertices; v_idx+=3) {
            float3 v1 = surface_mesh.triangles.ptr<float3>(0)[v_idx];
            float3 v0 = surface_mesh.triangles.ptr<float3>(0)[v_idx + 1];
            float3 v2 = surface_mesh.triangles.ptr<float3>(0)[v_idx + 2];

            if(v1.x == v0.x&&v1.y==v0.y&& v1.z==v0.z)
                continue;
            if(v1.x == v2.x&&v1.y==v2.y&& v1.z==v2.z)
                continue;
            if(v2.x == v0.x&&v2.y==v0.y&& v2.z==v0.z)
                continue;
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
        Eigen::Vector3f camera_target_pos =  (target_pos_3d* voxel_scale) ;
        std::vector<int>  idxs;
        std::vector<float>  dists;
        frs.radius_search(camera_target_pos,idxs,dists,40000);
        Eigen::Vector3f face_normal (0,0,-1);
        if(!idxs.empty())
                face_normal = nls[idxs[0]];

        //开始求解每个位置的收益值
        std::vector<std::vector<std::vector<float>>>  view_map(
        validness_map.cols, std::vector<std::vector<float>>(
        validness_map.rows,std::vector<float>(
        72,0)));

//#pragma omp parallel for schedule(dynamic)
        for (size_t m = 0; m < validness_map.cols; m++)
        {
            for (size_t n = 0; n < validness_map.rows; n++)
            {
                //判断当前点在validness_map中是否可达
                if(!validness_map.at<char>(m,n))
                    continue;

                //在当前点上每5度历遍方向
                Eigen::Vector2f dir2d = (target_pos - Eigen::Vector2i(m,n)).cast<float>();
                if(dir2d.norm()*voxel_scale <400)
                    continue;
                
                Eigen::Vector3f dir3d(dir2d.x(), dir2d.y(), 0);
                dir3d.normalize();
                Eigen::Vector3f ori (0, 1, 0);
                Eigen::Vector3f crossd = ori.cross(dir3d);
                int t = 1;
                if(crossd.z() < 0)
                    t= -1;
                float dotValue = ori.dot(dir3d);
                float angle = acos(dotValue)/M_PI *180;

                bool needInsert = false;
                for (size_t i = 0; i < 72; i++)
                {
                    float currAngle =  abs(angle*t-i*5);
                    while (currAngle > 180)
                    {
                        currAngle = currAngle - 180;
                    }
                    //如果方向和目标方向角度大于5度则不考虑
                    if(currAngle > 5)
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

                    needInsert = true;      
                    Eigen::Vector3f view_dir =  start_pt - target_pos_3d;
                    float dis =  view_dir.norm() *voxel_scale ;
                    view_dir.normalize();
                    float bv = M_PI/2.0 - acos(  Eigen::Vector3f(0,1,0).dot(view_dir));
                    if(bv>0.6)
                        continue;

                    //计算当前位置的收益值
                    float av = acos(face_normal.dot(view_dir));
                    float a0 = get_dis_value(dis);
                    float a1 = pow(M_E,-(av*av/0.36));
                    //float a2 = pow(M_E,-(bv*bv/0.36));// 0.36 means 0.6^2
                    float tmp =  a0*a1;//*a2;
                    view_map[m][n][i] =tmp;
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

       cv::Mat image(validness_map.cols, validness_map.rows,CV_8UC3);
       for (size_t m = 0; m < validness_map.cols; m++)
        {
            for (size_t n = 0; n < validness_map.rows; n++)
            {
                float valueTmp = 0;
                for (size_t i = 0; i < 72; i++)
                {
                    if(view_map[m][n][i] <= valueTmp)
                        continue;
                    valueTmp = view_map[m][n][i];
                }
                valueTmp  = valueTmp /maxValue;
                image.at<uchar3>(m,n) =  value_to_color(valueTmp);
            }
        }   
        uchar3 colorout;
        colorout.x = colorout.y = colorout.z = 255;
        image.at<uchar3>(target_pos.x(),target_pos.y()) =  colorout;
        cv::imwrite("out.bmp",image); 

        Eigen::Vector3f next_pose =  rotation * (nbv*voxel_scale) + translation;
        Eigen::Vector3f next_target =  rotation * (Eigen::Vector3f(target_pos_3d.x(),volume_size.y/2,target_pos_3d.z())*voxel_scale) + translation;
        //转化为世界坐标系中的点 
        Eigen::Matrix<float,6,1> out;
        out <<next_pose.x(),next_pose.y(),next_pose.z(),next_target.x(),next_target.y(),next_target.z();
        return out;
    }

    uchar3  value_to_color(float value)
    {
        float valueTmp = value <0? 0:(value > 1? 1:value);
        uchar3 colorout ;
        if(valueTmp < 1./8.)
        {
            colorout.z = 0;
            colorout.y = 0;
            colorout.x= (0.5 + (valueTmp)/(1./8.)*0.5)*255;
        }else if(valueTmp < 3./8.)
        {
            colorout.z = 0;
            colorout.y =  (valueTmp- 1./8.)/(3./8. -1./8.)*255;
            colorout.x = 255;
        }else if(valueTmp < 5./8.)
        {
            colorout.z = 0.8*(valueTmp-3./8.)/(5./8. - 3./8.)*255;
            colorout.y = 255;
            colorout.x = (1.-(valueTmp - 3./8.)/(5./8. - 3./8.))*255;
        }else if(valueTmp < 7./8.)
        {
            colorout.z = 0.8 * 255;
            colorout.y = (1.-(valueTmp - 5./8.) /(7./8. - 5./8.))*255;
            colorout.x = 0;
        }
        else
        {
            colorout.z = (0.8-(valueTmp - 7./8.) /(1. - 7./8.)*0.5)*255;
            colorout.y = 0;
            colorout.x = 0;
        }
        return colorout;
    }

    float get_dis_value(float dis)
    {
        if(dis>3000)
            return 0;
        if(dis>=700&&dis<=1500)
            return 1;
        if(dis< 700)
            return dis/700;
        else
            return(3000 - dis )/1500;         
    }

    void threeD_DDA(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& end_pt,
                std::function<bool(const Eigen::Matrix<int,6,1>& currBox)> func)
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
        Eigen::Vector3f  currPt = start_pt + Eigen::Vector3f(0.5, 0.5, 0.5);

        float tsum = 0;//用于终止
        int count = 0;
        do
        {
            if(count > 1000)
                break;
            count++;

            if((Eigen::Vector3f(currBox[0],currBox[1],currBox[2]) - end_pt).norm()< 10)
                break;

            if(func(currBox))
                break;
            float tx, ty, tz;
            tx = ty = tz = 1;
            if(dir.x() != 0)
                tx = abs((currBox[xi] - currPt.x())/dir.x());
             if(dir.y() != 0)
                ty = abs((currBox[yi] - currPt.y())/dir.y());
             if(dir.z() != 0)
                tz = abs((currBox[zi] - currPt.z())/dir.z());

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
            currBox[2] = currBox[2] + tnz;
            currBox[3] = currBox[3] + tnx;
            currBox[4] = currBox[4] + tny;
            currBox[5] = currBox[5] + tnz;
                                
            tsum += t;

            if(tsum>1.0)
                    break;    

            }while(1);
    }

    void twoD_DDA(const Eigen::Vector2f& start_pt, const Eigen::Vector2f& end_pt,
                    std::function<bool(const Eigen::Vector4i& currBox)> func)
    {
        Eigen::Vector2f dir = end_pt - start_pt;
        //根据方向获得可能相交的三个面
        int xi, yi , nx, ny;
        xi = 0; yi=1; nx = ny -1;
            
        if(dir.x()>0)
        {
            xi = 2;
            nx = 1;
        }

        if(dir.y()>0)
        {
            yi = 3;
            ny = 1;
        }
        Eigen::Vector4i  currBox;
        currBox<<start_pt.x(), start_pt.y(), start_pt.x()+1, start_pt.y()+1;
        Eigen::Vector2f  currPt = start_pt + Eigen::Vector2f(0.5, 0.5);

        float tsum = 0;//用于终止
        int count = 0;
        do
        {
            if(count > 1000)
                break;
            count++;
            
            if(func(currBox))
                break;

            float tx, ty;
            tx = ty = 1;
            if(dir.x() != 0)
                tx = abs((currBox[xi] - currPt.x())/dir.x());
             if(dir.y() != 0)
                ty = abs((currBox[yi] - currPt.y())/dir.y());

            int tnx = nx;
            int tny = ny;
                            
            float t = tx ;
            if(ty < tx )
            {
                tnx = 0;
                t = ty;
            }
            else
                tny = 0;

            currPt = currPt + dir*(t);

            currBox[0] = currBox[0] + tnx;
            currBox[1] = currBox[1] + tny;
            currBox[2] = currBox[2] + tnx;
            currBox[3] = currBox[3] + tny;
                                
            tsum += t;

            if(tsum>1.0)
                    break;    

            }while(1);
    }


    void get_2d_target_map(
                                                                    const cv::Mat& host_uncertainty_map, 
                                                                    const std::vector<OpenMesh::Vec3f> bdNoraml,
                                                                    const  cv::Mat& nlMap,
                                                                    const int3& volume_size, 
                                                                    std::vector<Eigen::Vector2i>& target_coord,
                                                                    cv::Mat&  u2dMap,
                                                                    cv::Mat& u2dMapNormal)
    {
       //初始化可信度地图,
        u2dMap = cv::Mat(volume_size.y, volume_size.x, CV_32S);
        u2dMapNormal = cv::Mat(volume_size.y, volume_size.x, CV_32FC3);
        u2dMap.setTo(0);
        u2dMapNormal.setTo(0);
        int maxInt = 0;    
        //将三维可信度压缩到二维
        for (size_t i = 0; i < volume_size.x; i++)
        {
            for (size_t j = 0; j < volume_size.y; j++)
            {
                for (size_t k = 0; k < volume_size.z; k++)
                {
                    short  outValue =  host_uncertainty_map.at<short>(k*volume_size.y + j,i);
                    if(outValue !=3)
                        continue;

                    u2dMap.at<int>(k,i )++;
                    if(u2dMap.at<int>(k,i)> maxInt)
                        maxInt = u2dMap.at<int>(k,i);

                    int idx = nlMap.at<int>(k*volume_size.y + j,i);
                    u2dMapNormal.at<OpenMesh::Vec3f>(k,i) +=  bdNoraml[idx]; 
                }              
            }         
        } 
        //将可信度地图中所有有值的坐标放入可信度队列
        std::vector<Eigen::Vector3i> uncertainty_queue;
        for (size_t m = 0; m < u2dMap.cols; m++)
        {
            for (size_t n = 0; n < u2dMap.rows; n++)
            {
                if(u2dMap.at<int>(m,n ) == 0)
                    continue;
                uncertainty_queue.push_back(Eigen::Vector3i(u2dMap.at<int>(m,n ),m,n));
            }
        }
        //排序可信度队列，按高到低进行排序
        std::sort(uncertainty_queue.begin(),uncertainty_queue.end(),
        [](const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs)->bool{
                return lhs.x() > rhs.x();
        } );

        //取可信度最高的坐标作为聚类中心，屏蔽聚类中心一定半径内的所有位置
         cv::Mat  islive = cv::Mat( u2dMap.cols, u2dMap.rows,CV_8U);
         islive.setTo(0);
        std::vector<Eigen::Vector2i>  corrOffset;
        corrOffset.reserve(4000);
        for(int i=-50; i<=50; ++i)
        {
            for(int j=-50;j<=50;++j)
            {
                corrOffset.push_back(Eigen::Vector2i(i,j));
            }
        }  
        //每次只取10个区域进行扫描
        for(int i=0;i<10;++i)
        {
            int m =0 ,n = 0;
            for(int j=0; j<uncertainty_queue.size(); ++j)
            {
                if(islive.at<uchar>(uncertainty_queue[j].y(),uncertainty_queue[j].z()))
                    continue;
                m = uncertainty_queue[j].y();
                n = uncertainty_queue[j].z();
                break;
            }

            if(m ==0 && n ==0)
                break;

            target_coord.push_back(Eigen::Vector2i(m,n));
            for(int j=0; j< corrOffset.size(); ++j)
            {
                int new_m = m+ corrOffset[j].x();
                int new_n = n+ corrOffset[j].y();
                if(new_m<0|| new_n <0||new_m>= islive.cols|| new_n >= islive.rows)
                    continue;
                islive.at<uchar>(new_m,new_n) = 1;
            }
        }
    
        cv::Mat image(u2dMap.cols, u2dMap.rows,CV_8UC3);
        for (size_t m = 0; m < u2dMap.cols; m++)
        {
            for (size_t n = 0; n < u2dMap.rows; n++)
            {
                 float  valueTmp = 0;
                if( u2dMap.at<int>(m,n) !=0)
                    valueTmp  = u2dMap.at<int>(m,n) /float(maxInt);
                image.at<uchar3>(n,m) =  value_to_color(valueTmp);
            }
        }   
        cv::imwrite("/tmp/out.bmp",image); 
    }

    void  get_2d_candidate_map(const cv::Mat& host_uncertainty_map, 
                                                                  const std::vector<Eigen::Vector2i>& target_coord,
                                                                  const cv::Mat& u2dMap,
                                                                  const cv::Mat& u2dMapNormal, 
                                                                  const int3& volume_size,
                                                                  const float& voxel_scale,
                                                                  cv::Mat& u2dCandidateMap,
                                                                  std::vector<std::vector<Eigen::Matrix<float,7,1>>>& cluster_candidate_coords)
    {
        //半径函数
        std::vector<Eigen::Vector2i>  corrOffset, emptyOffset;
        corrOffset.reserve(250); emptyOffset.reserve(250);
        for(int i=0; i <= 100; ++i)
        {
            int j =  std::sqrt(10000 - i*i);
            corrOffset.push_back(Eigen::Vector2i(i,j));
            corrOffset.push_back(Eigen::Vector2i(i,-j));
            corrOffset.push_back(Eigen::Vector2i(-i,j));
            corrOffset.push_back(Eigen::Vector2i(-i,-j));
        } 

        for(int i =-20;i<=20;++i)
        {
            
            for(int j =-20; j <= 20; ++ j)
            {
                emptyOffset.push_back(Eigen::Vector2i(i,j));
            }
        }

        //初始化候选视点地图
        u2dCandidateMap = cv::Mat (u2dMapNormal.cols, u2dMapNormal.rows,CV_32F);
        u2dCandidateMap.setTo(0);
        int maxInt = 0;      
        std::vector<Eigen::Vector3i>  candidate_coord;
        //以目标点为中心，一定半径上的点作为该目标点的候选视点
        for (size_t  j= 0; j < target_coord.size(); j++)
        {
                 int m  = target_coord[j].x();
                 int n =  target_coord[j].y();
                
                OpenMesh::Vec3f nl = u2dMapNormal.at<OpenMesh::Vec3f >(m,n);
                nl.normalize();

                for(int i =0 ;i< corrOffset.size();++i)
                {
                    Eigen::Vector2i coor =  Eigen::Vector2i(m,n) + corrOffset[i];
                    if(coor.x()<0||coor.y()<0||coor.x()>= u2dCandidateMap.cols||coor.y()>= u2dCandidateMap.rows)
                        continue;

                    OpenMesh::Vec3f  dir = OpenMesh::Vec3f (corrOffset[i].y(),0,corrOffset[i].x()) ;
                    dir.normalize();
                    dir = -dir;
                    float dotValue = OpenMesh::dot(nl,dir);
                    if(dotValue < 0.4)//除去视点方向和边界法向方向不一致的点
                        continue;

                    //视点只会出现在扫描过程中被判断为空的地方
                    if(host_uncertainty_map.at<short>(coor.x()*volume_size.y + 256, coor.y())!=1)
                       continue; 
                    
                    //判断候选视点是否在实体附近，如果在附近的话，就跳过
                    bool isNearObstacle = false;
                    for(int k =0; k< emptyOffset.size();++k)
                    {
                        Eigen ::Vector2i new_coor =  coor + emptyOffset[k];
                        if(u2dMap.at<int>(new_coor.x() , new_coor.y())==0)
                            continue; 
                        isNearObstacle = true;
                        break;
                    }

                    if(isNearObstacle)
                        continue;

                    candidate_coord.push_back(Eigen::Vector3i(coor.x(),coor.y(),j));

                    u2dCandidateMap.at<float>(coor.x(),coor.y()) = 1* dotValue ;
                    if(u2dCandidateMap.at<float>(coor.x(),coor.y())> maxInt)
                        maxInt = u2dCandidateMap.at<float>(coor.x(),coor.y());
                }
        }
        if(candidate_coord.empty())
            return;

        //排序候选视点，然后进行聚类
        std::sort(candidate_coord.begin(),candidate_coord.end(),
        [](const Eigen::Vector3i& lhs,const Eigen::Vector3i& rhs)->bool{
            return lhs.z() < rhs.z();
        });

        int targetNum  = candidate_coord[0].z();
        std::vector<Eigen::Vector2i>  tmp_corrds;
        for(int i=0;i<candidate_coord.size();++i)
        {
            if(targetNum != candidate_coord[i].z())
            {
                //simple candidate_coords
                std::vector<Eigen::Matrix<float,7,1>>  candidate_pose;
                get_pose_by_map(tmp_corrds,
                Eigen::Vector2i(target_coord[targetNum]),
                voxel_scale, candidate_pose);
                cluster_candidate_coords.push_back(candidate_pose);
                targetNum = candidate_coord[i].z();
                tmp_corrds.clear();
            }
            tmp_corrds.push_back(Eigen::Vector2i(candidate_coord[i].x(),candidate_coord[i].y()));        
        }

        //

        cv::Mat image(u2dCandidateMap.cols, u2dCandidateMap.rows,CV_8UC3);
        for (size_t m = 0; m < u2dCandidateMap.cols; m++)
        {
            for (size_t n = 0; n < u2dCandidateMap.rows; n++)
            {
               float  valueTmp  = u2dCandidateMap.at<float>(m,n) /maxInt;
                image.at<uchar3>(n,m) =  value_to_color(valueTmp);
            }
        }   
        cv::imwrite("/tmp/out1.bmp",image); 
    }


    void get_pose_by_map(const std::vector<Eigen::Vector2i>& input_coords, 
                                                      const Eigen::Vector2i& center, 
                                                      const float& voxel_scale,
                                                      std::vector<Eigen::Matrix<float,7,1>>& poses)
    {
        //首先对候选的坐标点进行采样
        std::vector<Eigen::Vector2i> tmp_coords = input_coords;
        std::sort(tmp_coords.begin(),tmp_coords.end(),
        [](const Eigen::Vector2i& lhs, const Eigen::Vector2i& rhs){
            return lhs.x()< rhs.x();
        });
        
        //弧度制
        auto  comupte_matrix =  [](const float angle,
            const Eigen::Vector3f& axisNorm,
            Eigen::Matrix3f& rigid)
        {
            Eigen::AngleAxisf  angleAxis(angle, axisNorm);
            rigid = angleAxis.matrix();
        };

        for(int i =0; i< tmp_coords.size(); i+=10)
        {
            Eigen::Vector2i dir =  center - tmp_coords[i];
            float angle = atan2(-float(dir.y()), float(dir.x()));
            Eigen::Matrix3f rigid;
            comupte_matrix(angle,Eigen::Vector3f(0,0,1),rigid);
            Eigen::Quaternionf quaternion(rigid);
            Eigen::Matrix<float,7,1> pose;
            pose[0] = (tmp_coords[i].x() - 256)*voxel_scale/1000.0f;
            pose[1] = (256 - tmp_coords[i].y())*voxel_scale/1000.0f;
            pose[2] = 0;
            pose[3] = quaternion.x();
            pose[4] = quaternion.y();
            pose[5] = quaternion.z();
            pose[6] = quaternion.w();
            poses.push_back(pose);
        }
    }
}