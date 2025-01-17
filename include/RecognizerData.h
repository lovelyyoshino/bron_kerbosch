# ifndef RecognizerTestData_h
# define RecognizerTestData_h

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace bron_kerbosch {
typedef int64_t Id;
typedef pcl::PointXYZ PclPoint;
typedef std::pair<Id, Id> IdPair;
typedef std::pair<PclPoint, PclPoint> PointPair;

class PairwiseMatch {
 public:
 /**
 // @brief 表示一对点云之间的匹配，包括它们的ID、质心、置信度等信息
 // @param  id1              第一个点云的标识符
 // @param  id2              第二个点云的标识符
 // @param  centroid1        第一个点云的质心坐标
 // @param  centroid2        第二个点云的质心坐标
 // @param  confidence_in    匹配的置信度值
  */
  PairwiseMatch(Id id1, Id id2, const PclPoint& centroid1, const PclPoint& centroid2,
                float confidence_in) :
                  ids_(id1, id2),
                  confidence_(confidence_in),
                  centroids_(PointPair(centroid1, centroid2)) {}

  // @brief 返回当前 PairwiseMatch 对象中的质心信
  PointPair getCentroids() const { return centroids_; }
  IdPair ids_;
  float confidence_;
  Eigen::MatrixXd features1_;
  Eigen::MatrixXd features2_;
  PointPair centroids_;
};

// 定义一个新的类型 PairwiseMatches，它是 PairwiseMatch 对象的动态数组（向量），并且使用 Eigen::aligned_allocator 来确保内存对齐
typedef std::vector<PairwiseMatch,
    Eigen::aligned_allocator<PairwiseMatch> > PairwiseMatches;
    
struct Translation {
  Translation(double x_in, double y_in, double z_in) :
    x(x_in), y(y_in), z(z_in) {}
  double x;
  double y;
  double z;
};

/// \brief Struct providing an hashing function for pairs of segment IDs.
struct IdPairHash {
  /// \brief Hashing function for pairs of segment IDs.
  /// \param pair Pair of IDs to be hashed.
  /// \returns The hash of the ID pair.
  // 分割块ID对的哈希函数
  // 参数：ID对
  // 返回：ID对的散列
  size_t operator() (const IdPair& pair) const {
    static_assert(std::is_same<IdPair, std::pair<int64_t, int64_t>>::value,
                  "The hashing function is valid only if IdPair is defined as "
                  "std::pair<int64_t, int64_t>");
    // We expect IDs to be always positive, which enables this trick for combining the two IDs. If
    // that would not be the case the hashing function could be less efficient, but still
    // functional.
	// ID应该是正的，才可以这一操作。如果不是正的，功能性可以满足，但效率会降低
    return std::hash<uint64_t>{}(static_cast<uint64_t>(pair.first) << 1 +
                                 static_cast<uint64_t>(pair.second));
  }
};

}

#endif // RecognizerTestData_h
