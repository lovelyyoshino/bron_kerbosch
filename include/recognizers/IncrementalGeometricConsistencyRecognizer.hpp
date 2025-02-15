#ifndef INCREMENTAL_GEOMETRIC_CONSISTENCY_RECOGNIZER_HPP_
#define INCREMENTAL_GEOMETRIC_CONSISTENCY_RECOGNIZER_HPP_

#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>

#include "parameter.h"
#include "recognizers/GraphBasedGeometricConsistencyRecognizer.hpp"
#include "RecognizerData.h"

namespace bron_kerbosch {

/// \brief Recognizes a model in a scene using a graph-based approach. First a consistency graph
/// is constructed, where the nodes represent the matches and edges connect matches that are
/// pairwise consistent. Recognition finds a maximum clique matches that are pairwise consistent.
/// In this incremental approach, information about candidate consistent match pairs are cached and
/// reused in successive recognition steps.
// 在这个增量式的方法中，缓存候选一致性匹配对的信息，在连续的识别步骤中复用
class IncrementalGeometricConsistencyRecognizer : public GraphBasedGeometricConsistencyRecognizer {
 public:
  /// \brief Initializes a new instance of the IncrementalGeometricConsistencyRecognizer class.
  /// \param params The parameters of the geometry consistency grouping.
  /// \param max_model_radius Radius of the bounding cylinder of the model.
  IncrementalGeometricConsistencyRecognizer(const GeometricConsistencyParams& params,
                                            float max_model_radius) noexcept;

 protected:
  /// \brief Builds a consistency graph of the provided matches.
  /// \param predicted_matches Vector of possible correspondences between model and scene.
  /// \returns Graph encoding pairwise consistencies. Match \c predicted_matches[i] is represented
  /// by node \c i .
  
  // 根据提供的匹配，构建一致性图
  // 参数：模型与场景间的可能一致性
  // 返回：图编码的成对一致性
  ConsistencyGraph buildConsistencyGraph(const PairwiseMatches& predicted_matches) override;

 private:
  // Per-partition data.
  struct PartitionData { };

  // Structure containing cached information for a match.
  // 一个匹配的缓存数据
  struct MatchCacheSlot {
    std::vector<size_t> candidate_consistent_matches;
    PointPair centroids_at_caching;
  };

  // Keeps track of the positions of a match in the vector of predicted matches and in the cache.
  // 跟踪在预测匹配向量和缓存中的一个匹配的位置
  struct MatchLocations {
    MatchLocations(const size_t match_index, const size_t cache_slot_index)
      : match_index(match_index), cache_slot_index(cache_slot_index) { }
    size_t match_index;
    size_t cache_slot_index;
  };

  // Computes the consistency distance between two matches, i.e. the difference between the
  // centroids distances in the scene and in the model.
  // 计算两个匹配间的一致性距离，即在场景中和模型中质心距离之差
  float computeConsistencyDistance(const PairwiseMatch& first_match,
                                   const PairwiseMatch& second_match,
                                   float max_target_distance) const;

  // Processes the predicted matches that are already present in the cache. Cleans up old entries,
  // finds consistencies and adds them to the consistency graph.
  // 处理缓存中已存在的预测匹配，清理旧条目，找到一致性并添加到一致性图
  void processCachedMatches(
      const PairwiseMatches& predicted_matches,
      const std::vector<MatchLocations>& cached_matches_locations,
      const std::vector<size_t>& cache_slot_index_to_match_index,
      std::unordered_map<IdPair, size_t, IdPairHash>& new_cache_slot_indices,
      ConsistencyGraph& consistency_graph);

  // Process the predicted matches that were not present in the cache. Finds consistencies and adds
  // them to the consistency graph.
  // 处理缓存中不存在的预测匹配，找到一致性并添加到一致性图
  void processNewMatches(
      const PairwiseMatches& predicted_matches,
      const std::vector<size_t>& free_cache_slot_indices,
      std::vector<size_t>& match_index_to_cache_slot_index,
      std::unordered_map<IdPair, size_t, IdPairHash>& new_cache_slot_indices,
      ConsistencyGraph& consistency_graph);

  // Decide if the match must be invalidated.
  // 决定匹配是否无效化
  bool mustRemoveFromCache(const PairwiseMatch& match, size_t cache_slot_index);

  // State of the cache.
  // 缓存状态
  std::vector<MatchCacheSlot> matches_cache_;
  // IdPairHash是自行定义的哈希函数
  std::unordered_map<IdPair, size_t, IdPairHash> cache_slot_indices_;

  static constexpr size_t kNoMatchIndex_ = std::numeric_limits<size_t>::max();
  static constexpr size_t kNoCacheSlotIndex_ = std::numeric_limits<size_t>::max();

  float max_consistency_distance_;
  float max_consistency_distance_for_caching_;
  float half_max_consistency_distance_for_caching_;
}; // class IncrementalGeometricConsistencyRecognizer

} // namespace bron_kerbosch

#endif // INCREMENTAL_GEOMETRIC_CONSISTENCY_RECOGNIZER_HPP_
