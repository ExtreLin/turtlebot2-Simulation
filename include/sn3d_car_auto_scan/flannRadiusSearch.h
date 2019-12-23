#include"flann/flann.hpp"
#include"flann/algorithms/dist.h"
#include"flann/algorithms/all_indices.h"
#include<eigen3/Eigen/Eigen>
#ifdef _OPENMP
#include<omp.h>
#endif

namespace CommonTools
{
    template<typename T>
    class FlannRadiusSearch
    {
    public:
        FlannRadiusSearch():_index(nullptr){};
        FlannRadiusSearch(const  std::vector<Eigen::Matrix<T,3,1>>& );
        ~FlannRadiusSearch(){
            if(_index!=nullptr )
                delete _index;
        };

    void set_points(const std::vector<Eigen::Matrix<T,3,1>>&);
    void set_params(const  flann::SearchParams&  params);
    void radius_search(const Eigen::Matrix<T,3,1>& pt, std::vector<int>& indices, std::vector<T>& dists, float radius);

    private:
        flann::Index<flann::L2_Simple<T>>   *_index;
        flann::Matrix<T>                                          _data;
        flann::SearchParams                                _searchParams;
    };

    template<typename T>
    FlannRadiusSearch<T>::FlannRadiusSearch(const std::vector<Eigen::Matrix<T,3,1>>&  pts)
    {
         set_points(pts);
        _searchParams.sorted = true;
        _searchParams.cores  = omp_get_max_threads();
        _searchParams.max_neighbors = 3;
    }

    template<typename T>
    void FlannRadiusSearch<T>::set_points(const std::vector<Eigen::Matrix<T,3,1>>& pts)
    {
         _data = flann::Matrix<T>(const_cast<T*>(&(pts[0][0])), pts.size(), 3); 
        _index = new flann::Index<flann::L2_Simple<T>> (_data, flann::KDTreeSingleIndexParams(15));
        _index->buildIndex();
    }

    template<typename T>
     void FlannRadiusSearch<T>::set_params(const  flann::SearchParams&  params)
     {
         _searchParams = params;
     }

     template<typename T>
     void FlannRadiusSearch<T>::radius_search(
         const Eigen::Matrix<T,3,1>& pt, std::vector<int>& indices, std::vector<T>& dists, float radius
     )
     {
        flann::Matrix<T> data = flann::Matrix<T>(const_cast<T*>(&(pt[0])), 1, 3); 
        std::vector<std::vector<int>> vec_indices;
        std::vector<std::vector<T>>  vec_dists;
        _index->radiusSearch(data, vec_indices, vec_dists, radius, _searchParams);
        indices = vec_indices[0];
        dists  = vec_dists[0];
     }
}