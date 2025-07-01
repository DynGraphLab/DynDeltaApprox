#pragma once
#include <vector>
namespace dyn_delta_approx::dynamic_edge_orientation::cchhqrs {

template <class DEdge>
struct SingleBucket {
  std::vector<DEdge*> bucket_elements;
  SingleBucket() {}
  ~SingleBucket() {}
};

template <class DEdge>
class Buckets {
 public:
  // contains j, Bj for different j. External list in sorted order, internal
  // list Bj in arbitrary order. Bj contains in neighbours w s.t. j = log(d+(w))
  std::vector<SingleBucket<DEdge>> buckets;

 public:
  int get_bucket_id(int du, int b, double lambda_precomp, int offset) const {
    float duf = du;
    if (duf < 4 * b) {
      return 0;
    }
    return static_cast<int>(log(duf) * lambda_precomp) - offset;
  }
  void add(DEdge* uv, int out_degree, int bucket_v, int b, double lambda_precomp, int offset) {
    int j = get_bucket_id(out_degree, b, lambda_precomp, offset);
    if (j >= buckets.size()) {
      buckets.resize(j + 1);
    }

    buckets[j].bucket_elements.push_back(uv);
    uv->location_in_neighbours = buckets[j].bucket_elements.size() - 1;
    uv->bucket = j;
  }
  void remove(DEdge* uv) {
    assert(uv->location_in_neighbours != -1);
    assert(buckets[uv->bucket].bucket_elements.size() >
           uv->location_in_neighbours);  //"The edge you are trying to delete is not present in the
                                         // bucket at all"

    assert(buckets[uv->bucket].bucket_elements[uv->location_in_neighbours]->target == uv->target &&
           buckets[uv->bucket].bucket_elements[uv->location_in_neighbours]->mirror->target ==
               uv->mirror->target);  // "The edge you are trying to delete is not present in the
                                     // bucket location"
    if (buckets[uv->bucket].bucket_elements.size() > 1) {
      std::swap(
          buckets[uv->bucket].bucket_elements[uv->location_in_neighbours],
          buckets[uv->bucket].bucket_elements[buckets[uv->bucket].bucket_elements.size() - 1]);
      buckets[uv->bucket].bucket_elements[uv->location_in_neighbours]->location_in_neighbours =
          uv->location_in_neighbours;
    }
    buckets[uv->bucket].bucket_elements.resize(buckets[uv->bucket].bucket_elements.size() - 1);
    if (buckets.size() == uv->bucket + 1 && buckets[uv->bucket].bucket_elements.empty()) {
      buckets.resize(buckets.size() - 1);
    }
    uv->location_in_neighbours = -1;  // buckets[uv->bucket].bucket_elements.size();
  }
  void update(DEdge* uv, int outdegree_u, int b, double lambda_precomp, int offset) {
    int j = get_bucket_id(outdegree_u, b, lambda_precomp, offset);
    int j_prev = uv->bucket;
    if (j == j_prev) {
      return;
    }
    if (j >= buckets.size()) {
      buckets.resize(j + 1);
    }
    remove(uv);
    buckets[j].bucket_elements.push_back(uv);
    uv->location_in_neighbours = (buckets[j].bucket_elements.size() - 1);
    uv->bucket = j;
  }
  DEdge* get_max() {
    while (buckets.back().bucket_elements.empty()) {
      buckets.pop_back();
    }
    return buckets.back().bucket_elements.front();
  }
};
}  // namespace dyn_delta_approx::dynamic_edge_orientation::cchhqrs