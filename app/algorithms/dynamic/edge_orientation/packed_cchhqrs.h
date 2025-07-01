#pragma once

#include <sys/types.h>

#include <map>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "app/algorithms/dyn_algorithm_impl.h"
#include "dynamic_edge_orientation/cchhqrs/bucket.h"

namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation {
template <class InEdge, class Vertex>
struct InBucketsVector {
 public:
  // contains j, Bj for different j. External list in sorted order, internal
  // list Bj in arbitrary order. Bj contains in neighbors w s.t. j = log(d+(w))
  std::vector<std::vector<InEdge>> buckets;

 public:
  int get_bucket_id(int du, int b, double lambda_precomp, int offset) const {
    float duf = du;
    if (duf < 4 * b) {
      return 0;
    }
    return static_cast<int>(log(duf) * lambda_precomp) - offset;
  }
  std::pair<int, int> add(typename Vertex::NT source, int positionOutNeighbor, int out_degree,
                          int b, double lambda_precomp, int offset) {
    int j = get_bucket_id(out_degree, b, lambda_precomp, offset);
    if (j >= buckets.size()) {
      buckets.resize(j + 1);
    }

    buckets[j].push_back(InEdge(source, positionOutNeighbor));
    return std::make_pair(buckets[j].size() - 1, j);
  }
  void remove(typename Vertex::NT u, int offSet, std::vector<Vertex>& vertices) {
    int bck = vertices[u].out_edges[offSet].bucket;
    assert(buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source == u);
    assert(vertices[u].out_edges[offSet].location_in_neighbors != -1);
    assert(buckets[bck].size() > vertices[u].out_edges[offSet].location_in_neighbors);
    //"The edge you are trying to delete is not present in
    //        the
    // bucket at all"

    // assert(buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->target ==
    //            uv->target &&
    //        buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->mirror->target ==
    //            uv->mirror->target);  // "The edge you are trying to delete is not present in
    //            the
    //                                  // bucket location"
    if (buckets[bck].size() > 1) {
      if (vertices[u].out_edges[offSet].location_in_neighbors != buckets[bck].size() - 1) {
        assert(vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        assert(vertices[buckets[bck][buckets[bck].size() - 1].source].out_edges.size() >
               buckets[bck][buckets[bck].size() - 1].location_out_neighbors);
        std::swap(buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors],
                  buckets[bck][buckets[bck].size() - 1]);
        assert(vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
            .out_edges[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                           .location_out_neighbors]
            .location_in_neighbors = vertices[u].out_edges[offSet].location_in_neighbors;
      }
    }
    buckets[bck].resize(buckets[bck].size() - 1);
    if (buckets.size() == bck + 1 && buckets[bck].empty()) {
      buckets.resize(buckets.size() - 1);
    }
    // uv.location_in_neighbors = -1;  // buckets[uv->bucket].bucket_elements.size();*/
  }
  void update(typename Vertex::NT u, int offsetU, int outdegree_u, int b, double lambda_precomp,
              int offset, std::vector<Vertex>& vertices) {
    int j = get_bucket_id(outdegree_u, b, lambda_precomp, offset);
    int j_prev = vertices[u].out_edges[offsetU].bucket;
    if (j == j_prev) {
      return;
    }
    //   std::cout << "UZP" << std::endl;

    remove(u, offsetU, vertices);
    if (j >= buckets.size()) {
      buckets.resize(j + 1);
    }
    buckets[j].push_back(InEdge(u, offsetU));

    vertices[u].out_edges[offsetU].location_in_neighbors = (buckets[j].size() - 1);
    vertices[u].out_edges[offsetU].bucket = j;
  }
  std::pair<bool, InEdge> get_max() {
    while (!buckets.empty() && buckets.back().empty()) {
      buckets.pop_back();
    }
    if (buckets.empty() || buckets.back().empty()) {
      return {false, InEdge{}};
    }
    return std::make_pair(true, buckets.back().front());
  }
  void update_in_edge(int bucket, int location_in_neighbors, int val) {
    buckets[bucket][location_in_neighbors].location_out_neighbors = val;
  }
};
template <class InEdge, class Vertex>
struct InBucketsMap {
 public:
  // contains j, Bj for different j. External list in sorted order, internal
  // list Bj in arbitrary order. Bj contains in neighbors w s.t. j = log(d+(w))
  std::map<int, std::vector<InEdge>, std::greater<int>> buckets;

 public:
  int get_bucket_id(int du, int b, double lambda_precomp, int offset) const {
    float duf = du;
    if (duf < 4 * b) {
      return 0;
    }
    return static_cast<int>(log(duf) * lambda_precomp) - offset;
  }
  std::pair<int, int> add(typename Vertex::NT source, int positionOutNeighbor, int out_degree,
                          int b, double lambda_precomp, int offset) {
    int j = get_bucket_id(out_degree, b, lambda_precomp, offset);
    buckets[j].push_back(InEdge(source, positionOutNeighbor));
    return std::make_pair(buckets[j].size() - 1, j);
  }
  void remove(typename Vertex::NT u, int offSet, std::vector<Vertex>& vertices) {
    int bck = vertices[u].out_edges[offSet].bucket;
    assert(buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source == u);
    assert(vertices[u].out_edges[offSet].location_in_neighbors != -1);
    assert(buckets[bck].size() > vertices[u].out_edges[offSet].location_in_neighbors);
    //"The edge you are trying to delete is not present in
    //        the
    // bucket at all"

    // assert(buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->target ==
    //            uv->target &&
    //        buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->mirror->target ==
    //            uv->mirror->target);  // "The edge you are trying to delete is not present in
    //            the
    //                                  // bucket location"
    if (buckets[bck].size() > 1) {
      if (vertices[u].out_edges[offSet].location_in_neighbors != buckets[bck].size() - 1) {
        assert(vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        assert(vertices[buckets[bck][buckets[bck].size() - 1].source].out_edges.size() >
               buckets[bck][buckets[bck].size() - 1].location_out_neighbors);
        std::swap(buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors],
                  buckets[bck][buckets[bck].size() - 1]);
        assert(vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        vertices[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors].source]
            .out_edges[buckets[bck][vertices[u].out_edges[offSet].location_in_neighbors]
                           .location_out_neighbors]
            .location_in_neighbors = vertices[u].out_edges[offSet].location_in_neighbors;
      }
    }
    buckets[bck].resize(buckets[bck].size() - 1);
    if (buckets[bck].empty()) {
      buckets.erase(bck);
    }
    // uv.location_in_neighbors = -1;  // buckets[uv->bucket].bucket_elements.size();*/
  }
  void update(typename Vertex::NT u, int offsetU, int outdegree_u, int b, double lambda_precomp,
              int offset, std::vector<Vertex>& vertices) {
    int j = get_bucket_id(outdegree_u, b, lambda_precomp, offset);
    int j_prev = vertices[u].out_edges[offsetU].bucket;
    if (j == j_prev) {
      return;
    }
    //   std::cout << "UZP" << std::endl;

    remove(u, offsetU, vertices);
    buckets[j].push_back(InEdge(u, offsetU));

    vertices[u].out_edges[offsetU].location_in_neighbors = (buckets[j].size() - 1);
    vertices[u].out_edges[offsetU].bucket = j;
  }
  std::pair<bool, InEdge> get_max() {
    while (!buckets.empty() && buckets.begin()->second.empty()) {
      buckets.erase(buckets.begin());
    }
    if (buckets.empty() || buckets.begin()->second.empty()) {
      return {false, InEdge{}};
    }
    return std::make_pair(true, buckets.begin()->second.front());
  }
  void update_in_edge(int bucket, int location_in_neighbors, int val) {
    buckets[bucket][location_in_neighbors].location_out_neighbors = val;
  }
};
template <class InEdge, class Vertex>
struct InBucketsList {
 public:
  // contains j, Bj for different j. External list in sorted order, internal
  // list Bj in arbitrary order. Bj contains in neighbors w s.t. j = log(d+(w))
  struct SingleBucket {
    int value = -1;
    std::vector<InEdge> edges;
    SingleBucket* next = nullptr;
    SingleBucket(int val) : value(val) {}
    ~SingleBucket() {
      if (next != nullptr) {
        delete next;
      }
    }
  };
  SingleBucket* highest = nullptr;

 public:
  int get_bucket_id(int du, int b, double lambda_precomp, int offset) const {
    float duf = du;
    if (duf < 4 * b) {
      return 0;
    }
    return static_cast<int>(log(duf) * lambda_precomp) - offset;
  }
  std::pair<int, int> add(typename Vertex::NT source, int positionOutNeighbor, int out_degree,
                          int b, double lambda_precomp, int offset) {
    int j = get_bucket_id(out_degree, b, lambda_precomp, offset);
    if (highest == nullptr || j > highest->value) {
      SingleBucket* new_h = new SingleBucket(j);
      new_h->next = highest;
      highest = new_h;
      new_h->edges.push_back(InEdge(source, positionOutNeighbor));
      return std::make_pair(0, j);
    }
    SingleBucket* prevP = highest;
    SingleBucket* currP = highest->next;
    while (currP != nullptr && currP->value > j) {
      prevP = currP;
      currP = currP->next;
    }
    if (currP == nullptr && j == prevP->value) {
      currP = prevP;
    }
    if (j == prevP->value) {
      prevP->edges.push_back(InEdge(source, positionOutNeighbor));
      return std::make_pair(prevP->edges.size() - 1, j);
    }
    if (currP == nullptr || j > currP->value) {
      SingleBucket* new_h = new SingleBucket(j);
      new_h->next = currP;
      prevP->next = new_h;
      new_h->edges.push_back(InEdge(source, positionOutNeighbor));
      return std::make_pair(0, j);
    }

    assert(currP->value == j);
    currP->edges.push_back(InEdge(source, positionOutNeighbor));
    return std::make_pair(currP->edges.size() - 1, j);
  }
  void remove(typename Vertex::NT u, int offSet, std::vector<Vertex>& vertices) {
    int bck = vertices[u].out_edges[offSet].bucket;
    SingleBucket* bucket = highest;
    SingleBucket* prev = nullptr;

    while (bucket != nullptr && bucket->value > bck) {
      prev = bucket;
      bucket = bucket->next;
    }
    assert(bucket->value == bck);

    assert(bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors].source == u);
    assert(vertices[u].out_edges[offSet].location_in_neighbors != -1);
    assert(bucket->edges.size() > vertices[u].out_edges[offSet].location_in_neighbors);
    //"The edge you are trying to delete is not present in
    //        the
    // bucket at all"

    // assert(buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->target ==
    //            uv->target &&
    //        buckets[uv->bucket].bucket_elements[uv->location_in_neighbors]->mirror->target ==
    //            uv->mirror->target);  // "The edge you are trying to delete is not present in
    //            the
    //                                  // bucket location"
    if (bucket->edges.size() > 1) {
      if (vertices[u].out_edges[offSet].location_in_neighbors != bucket->edges.size() - 1) {
        assert(vertices[bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        assert(vertices[bucket->edges[bucket->edges.size() - 1].source].out_edges.size() >
               bucket->edges[bucket->edges.size() - 1].location_out_neighbors);
        std::swap(bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors],
                  bucket->edges[bucket->edges.size() - 1]);
        assert(vertices[bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors].source]
                   .out_edges.size() >
               bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors]
                   .location_out_neighbors);
        vertices[bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors].source]
            .out_edges[bucket->edges[vertices[u].out_edges[offSet].location_in_neighbors]
                           .location_out_neighbors]
            .location_in_neighbors = vertices[u].out_edges[offSet].location_in_neighbors;
      }
    }
    bucket->edges.resize(bucket->edges.size() - 1);
    if (bucket->edges.empty()) {
      if (prev != nullptr) {
        prev->next = bucket->next;
      } else {
        highest = bucket->next;
      }
      bucket->next = nullptr;
      delete bucket;
    }
    // uv.location_in_neighbors = -1;  // buckets[uv->bucket].bucket_elements.size();*/
  }
  void update(typename Vertex::NT u, int offsetU, int outdegree_u, int b, double lambda_precomp,
              int offset, std::vector<Vertex>& vertices) {
    int j = get_bucket_id(outdegree_u, b, lambda_precomp, offset);
    int j_prev = vertices[u].out_edges[offsetU].bucket;
    if (j == j_prev) {
      return;
    }
    //   std::cout << "UZP" << std::endl;

    remove(u, offsetU, vertices);
    auto result = add(u, offsetU, outdegree_u, b, lambda_precomp, offset);

    vertices[u].out_edges[offsetU].location_in_neighbors = result.first;
    vertices[u].out_edges[offsetU].bucket = j;
  }
  std::pair<bool, InEdge> get_max() {
    if (highest == nullptr || highest->edges.empty()) {
      return {false, InEdge{}};
    }
    return std::make_pair(true, highest->edges.front());
  }
  void update_in_edge(int bucket, int location_in_neighbors, int val) {
    SingleBucket* ptr = highest;
    while (ptr->value > bucket) {
      ptr = ptr->next;
    }
    ptr->edges[location_in_neighbors].location_out_neighbors = val;
  }
};
template <class DynGraph, template <class, class> class InBuckets, int TargetPrecision = 32,
          int LocationInNeighborPrecision = 22, int LocationOutPrecision = 22,
          int CountPrecision = 10, int BucketPrecision = 10>
class PackedCCHHQRSEdgeOrientation {
  using NT = typename DynGraph::NodeType;
  struct OutEdge {
    // Vertex *source;
    // TODO resort
    int target : TargetPrecision;
    int count : CountPrecision = 0;
    int location_in_neighbors : LocationInNeighborPrecision = -1;
    unsigned int bucket : BucketPrecision = -1;  // location of the edge
    int mirror : LocationInNeighborPrecision = -1;
    // int location_out_neighbors : LocationOutPrecision = -1;  // location of edge in the out
    // neighbor list of
    //  source
    OutEdge(NT v) : target(v){};
    OutEdge() {}
  };
  struct InEdge {
    int source : TargetPrecision;
    int location_out_neighbors : LocationOutPrecision;
    InEdge(int source) : source(source) {}
    InEdge(int source, int loc_u) : source(source), location_out_neighbors(loc_u) {}

    InEdge() {}
  };
  struct Vertex;

  struct Vertex {
    using NT = typename DynGraph::NodeType;

    std::vector<OutEdge> out_edges;
    //  std::set<DEdge*> deleted_out_edges;
    InBuckets<InEdge, Vertex> in_edges;
    // DEdge<NT>* self_loop; //self loop at 0?
    int out_degree = 0;  // out degree, //TODO remove, multiplicity of all
    int active_edges = 0;
    int robin = 0;
  };
  std::vector<Vertex> vertices;
#ifdef VALIDATE_EDGE_ORIENTATION
  std::map<std::pair<int, int>, bool> seen_edges;
#endif
  int64_t maxOutDegree() const {
    int64_t outd = 0;
    int64_t sum = 0;
    int64_t i = 0;
    int64_t saa = 0;
    int min_b = ((double)b) / 2.0;
    for (const auto& vertex : vertices) {
      int64_t v_out = 0;
      for (auto e = 0UL; e < vertex.active_edges; e++) {
        auto uv = vertex.out_edges[e];
        saa += uv.count;
        // v_out += std::floor(((double)uv.count) / (double)b);
        //  if (uv.count > b) std::cout << uv.count << std::endl;
        //  for (int j = 0; j < uv.count / b; j++) {
        //    v_out++;
        //  }
        //  "partial" edges
        //  if there is a half edge, we break ties arbitrarily
        v_out += ((2 * (uv.count) == b) & (i > uv.target)) | (2 * (uv.count) > b);
        // if (2 * (uv.count) == b) {
        //   if (i > uv.target) {
        //     // std::cout << "INC1" << std::endl;
        //     v_out++;
        //   }
        // } else if (2 * (uv.count) > b) {
        //   // std::cout << "INC2" << std::endl;
        //   v_out++;
        // }
      }
      outd = std::max(outd, v_out);
      sum += v_out;
      i++;
    }
    // std::cout << sum << std::endl;
    // assert(sum == insertions);
    return outd;
  }
  double squaredSumDegree() const {
    double outd = 0;
    int64_t i = 0;
    int min_b = ((double)b) / 2.0;
    for (const auto& vertex : vertices) {
      int64_t v_out = 0;
      for (auto e = 0UL; e < vertex.active_edges; e++) {
        auto uv = vertex.out_edges[e];
        // v_out += std::floor(((double)uv.count) / (double)b);
        // if (uv.count > b) std::cout << uv.count << std::endl;
        // for (int j = 0; j < uv.count / b; j++) {
        //   v_out++;
        // }
        // "partial" edges
        // if there is a half edge, we break ties arbitrarily
        v_out += ((2 * (uv.count) == b) & (i > uv.target)) | (2 * (uv.count) > b);

        // if (2 * (uv.count) == b) {
        //   if (i > uv.target) {
        //     // std::cout << "INC1" << std::endl;
        //     v_out++;
        //   }
        // } else if (2 * uv.count > b) {
        //   // std::cout << "INC2" << std::endl;
        //   v_out++;
        // }
      }
      outd += std::pow((double)v_out, 2);
      i++;
    }
    return outd;
  }
  int robin_size;
  int offset, theta;
  int b;
  double lambda_precomp, lambda;
  void checkAll() {
    return;
    for (NT u = 0; u < vertices.size(); u++) {
      for (int j = 0; j < vertices[u].active_edges; j++) {
        // assert(vertices[u])
        OutEdge e = vertices[u].out_edges[j];
        assert(vertices[e.target].in_edges.buckets[e.bucket][e.location_in_neighbors].source == u);
        assert(vertices[e.target].out_edges[e.mirror].target == u);
      }
      // for (int j = 0; j < vertices[u].in_edges.buckets.size(); j++) {
      //   // assert(vertices[u])
      //   for (InEdge e : vertices[u].in_edges.buckets[j]) {
      //     //   OutEdge e = vertices[u].out_edges[j];
      //     //  std::cout << e.source << " " << e.location_out_neighbors << std::endl;
      //     assert(vertices[e.source].out_edges.size() > e.location_out_neighbors);
      //     assert(vertices[e.source].out_edges[e.location_out_neighbors].target == u);
      //   }
      // }
    }
  }

 public:
  struct DynGraphInfo {
    size_t num_changes = 0;
    size_t num_nodes = 0;
    DynGraphInfo(size_t nc, size_t nn) : num_changes(nc), num_nodes(nn) {}
    size_t currentNumEdges() const { return num_changes; }
    size_t currentNumNodes() const { return num_nodes; }
  };
  DynGraphInfo graph;
  template <template <typename, typename> class MapType, typename StringType>
  PackedCCHHQRSEdgeOrientation(const DynGraph& g, MapType<StringType, double> double_params,
                               MapType<StringType, int64_t> int64_params)
      : graph(g.currentNumEdges(), g.currentNumNodes()),
        lambda(double_params["lambda"]),
        theta(int64_params["theta"]),
        b(int64_params["b"]) {
    robin_size = std::max(10.0, 2.0 / lambda) + 1;
    lambda_precomp = 1.0 / (log(1.0 + lambda));
    offset = static_cast<int>(log(std::max(1.0f, static_cast<float>(4 * b))) * lambda_precomp);
    vertices.resize(g.currentNumNodes());
    std::cout << sizeof(OutEdge) << std::endl;
    std::cout << sizeof(InEdge) << std::endl;

    // no setup, self loops needed
  }
  int64_t weight() const { return size(); }
  int64_t size() const { return maxOutDegree(); }
  double quality() { return squaredSumDegree(); }
  int free_edges_size() { return 0; };
  const DynGraphInfo& instance() const { return graph; }
  bool valid() {
#ifdef VALIDATE_EDGE_ORIENTATION
    int i = 0;
    int min_b = ((double)b) / 2.0;
    for (const auto& vertex : vertices) {
      for (auto e = 0UL; e < vertex.active_edges; e++) {
        auto uv = vertex.out_edges[e];
        // std::cout << i << " " << uv.target << " " << uv.count << std::endl;
        int64_t v_out = 0;
        bool vout = std::floor(((double)uv.count) / (double)b);
        // if (uv.count > b) std::cout << uv.count << std::endl;
        // for (int j = 0; j < uv.count / b; j++) {
        //   v_out++;
        // }
        // "partial" edges
        // if there is a half edge, we break ties arbitrarily
        if (2 * (uv.count) == b) {
          if (i > uv.target) {
            // std::cout << "INC1" << std::endl;
            v_out = true;
          }
        } else if (2 * (uv.count) > b) {
          // std::cout << "INC2" << std::endl;
          v_out = true;
        }
        if (v_out) {
          auto source = i;
          auto target = uv.target;
          // std::cout << source << " " << target << " seen" << std::endl;

          if (!seen_edges[std::make_pair(std::max(source, target), std::min(source, target))]) {
            return false;
          }
          seen_edges[std::make_pair(std::max(source, target), std::min(source, target))] = false;
        }
      }
      i++;
    }
    bool no_error = true;
    for (auto [k, v] : seen_edges) {
      if (v) {
        std::cout << k.first << " " << k.second << std::endl;
        no_error = false;
      }
    }
    return no_error;
#else
    return true;
#endif
  }
  int addEdge(NT u, NT v, int reverse = -1) {
    vertices[u].out_edges.emplace_back(v);
    if (reverse != -1) {
      vertices[v].out_edges[reverse].mirror = vertices[u].out_edges.size() - 1;
      vertices[u].out_edges.back().mirror = reverse;
    }
    return vertices[u].out_edges.size() - 1;
  }
  int insertions = 0;
  NT g_source, g_target;
  int g_offset_uv = -1, g_offset_vu = -1;
  void handle_insertion(const typename DynGraph::StreamMember& edge) {
    insertions++;
    g_source = edge.first;
    g_target = edge.second;
    // std::cout << "Insert " << g_source << " " << g_target << std::endl;

#ifdef VALIDATE_EDGE_ORIENTATION
    seen_edges[std::make_pair(std::max(g_target, g_source), std::min(g_source, g_target))] = true;
#endif
    if (g_source == g_target) {
      return;
    }
    g_offset_uv = addEdge(g_source, g_target);
    g_offset_vu = addEdge(g_target, g_source, g_offset_uv);

    for (unsigned i = 0; i < b; i++) {
      if (vertices[g_source].out_degree <= vertices[g_target].out_degree) {
        insert_directed_worst_case(g_source, g_offset_uv);
      } else {
        insert_directed_worst_case(g_target, g_offset_vu);
      }
      // Insert Directed is currently not adjusted for this new structure
      // b
      assert(vertices[g_source].out_edges[g_offset_uv].target == g_target);
      assert(vertices[g_target].out_edges[g_offset_vu].target == g_source);
      // checkAll();
    }
#ifdef VALIDATE_EDGE_ORIENTATION
    int uv_cap = 0;
    for (int i = 0; i < vertices[g_source].active_edges; i++) {
      if (vertices[g_source].out_edges[i].target == g_target) {
        uv_cap += vertices[g_source].out_edges[i].count;
        // break;
      }
    }
    for (int i = 0; i < vertices[g_target].active_edges; i++) {
      if (vertices[g_target].out_edges[i].target == g_source) {
        uv_cap += vertices[g_target].out_edges[i].count;
        // break;
      }
    }
    assert(uv_cap == b);
    checkAll();
#endif
  }
  void handle_deletion(const typename DynGraph::StreamMember& edge) {
    g_source = edge.first;
    g_target = edge.second;
    // std::cout << "Remove " << g_source << " " << g_target << std::endl;
#ifdef VALIDATE_EDGE_ORIENTATION
    seen_edges[std::make_pair(std::max(g_source, g_target), std::min(g_source, g_target))] = false;
#endif
    if (g_source == g_target) {
      return;
    }
    g_offset_uv = -1;
    g_offset_vu = -1;
    for (int i = 0; i < vertices[g_source].active_edges; i++) {  // TODO optimize.
      if (vertices[g_source].out_edges[i].target == g_target) {
        g_offset_uv = i;
        g_offset_vu = vertices[g_source].out_edges[i].mirror;
      }
    }
    if (g_offset_uv == -1) {
      for (int i = 0; i < vertices[g_target].active_edges; i++) {
        if (vertices[g_target].out_edges[i].target == g_source) {
          g_offset_vu = i;
          g_offset_uv = vertices[g_target].out_edges[i].mirror;
        }
      }
    }
    while (vertices[g_source].out_edges[g_offset_uv].count > 0 ||
           vertices[g_target].out_edges[g_offset_vu].count > 0) {
      if (vertices[g_source].out_edges[g_offset_uv].count >
          vertices[g_target].out_edges[g_offset_vu].count) {
        delete_directed_worst_case(g_source, g_offset_uv);
      } else {
        delete_directed_worst_case(g_target, g_offset_vu);
      }
    }
    // remove out of arrays
    auto& v = vertices[g_target];
    auto& u = vertices[g_source];
    assert(u.out_edges[g_offset_uv].target == g_target);
    assert(v.out_edges[g_offset_vu].target == g_source);

    if (g_offset_uv != -1 && g_offset_uv != u.out_edges.size() - 1) {
      std::swap(u.out_edges[u.out_edges.size() - 1], u.out_edges[g_offset_uv]);
      if (u.out_edges[g_offset_uv].location_in_neighbors != -1) {
        vertices[u.out_edges[g_offset_uv].target].in_edges.update_in_edge(
            u.out_edges[g_offset_uv].bucket, u.out_edges[g_offset_uv].location_in_neighbors,
            g_offset_uv);
      }
      if (u.out_edges[g_offset_uv].mirror != -1) {
        vertices[u.out_edges[g_offset_uv].target]
            .out_edges[u.out_edges[g_offset_uv].mirror]
            .mirror = g_offset_uv;
      }
    }
    u.out_edges.pop_back();
    if (g_offset_vu != -1 && g_offset_vu != v.out_edges.size() - 1) {
      std::swap(v.out_edges[v.out_edges.size() - 1], v.out_edges[g_offset_vu]);
      if (v.out_edges[g_offset_vu].location_in_neighbors != -1) {
        vertices[v.out_edges[g_offset_vu].target].in_edges.update_in_edge(
            v.out_edges[g_offset_vu].bucket, v.out_edges[g_offset_vu].location_in_neighbors,
            g_offset_vu);
      }
      if (v.out_edges[g_offset_vu].mirror != -1) {
        vertices[v.out_edges[g_offset_vu].target]
            .out_edges[v.out_edges[g_offset_vu].mirror]
            .mirror = g_offset_vu;
      }
    }
    v.out_edges.pop_back();
#ifdef VALIDATE_EDGE_ORIENTATION
    checkAll();
#endif
  }
  [[nodiscard]] int add(NT u, int offsetOutArray) {
    vertices[u].out_degree++;
    vertices[u].out_edges[offsetOutArray].count++;
    assert(vertices[u].out_edges[offsetOutArray].count <= b);
    {
      auto uw = vertices[u].out_edges[offsetOutArray];
      assert(vertices[uw.target].out_edges[uw.mirror].target == u);
    }
    if (vertices[u].out_edges[offsetOutArray].count == 1) {  // new edge
      if (vertices[u].active_edges < vertices[u].out_edges.size() - 1) {
        // std::cout << vertices[u].active_edges << " swapping " << offsetOutArray << " "
        //           << vertices[u].out_edges.size() << std::endl;
        // // assert(vertices[u].active_edges <= uv->location_out_neighbors);
        // assert(vertices[u].out_edges[uv->location_out_neighbors] == uv);
        swap_in_place(u, vertices[u].active_edges, offsetOutArray);
        offsetOutArray = vertices[u].active_edges;
      }

      vertices[u].active_edges++;
      // assert(vertices[u].out_edges[vertices[u].active_edges - 1] == uv);
      //  insert u in the in neighbors of v
      auto target = vertices[u].out_edges[offsetOutArray].target;
      auto [locInNeighbor, bucket] = vertices[target].in_edges.add(
          u, offsetOutArray, vertices[u].out_degree, b, lambda_precomp, offset);
      vertices[u].out_edges[offsetOutArray].location_in_neighbors = locInNeighbor;
      vertices[u].out_edges[offsetOutArray].bucket = bucket;
    }
    return offsetOutArray;
  }
  [[nodiscard]] int insert_directed_worst_case(NT u, int offsetOutArray) {
    // NOLINT(*-no-recursion)
    //   assert_edges(u);
    offsetOutArray = add(u, offsetOutArray);
    {
      auto uw = vertices[u].out_edges[offsetOutArray];
      assert(vertices[uw.target].out_edges[uw.mirror].target == u);
    }
    //  assert_edges(u);
    int t_max = 0;
    int old_robin = vertices[u].robin;
    // Round robin
    // unsigned int out_edges_size = vertices[u].out_edges.size();
    auto max_val_loop = std::min(robin_size, vertices[u].active_edges);
    const auto out_deg = vertices[u].out_degree;
    auto& u_ = vertices[u];
    for (int t = 0; t < max_val_loop; t++) {
      if (u_.robin >= vertices[u].active_edges) {
        u_.robin = 0;
      }
      assert(u_.robin < u_.active_edges);
      OutEdge uw = u_.out_edges[u_.robin];
      // Find
      if (u_.out_degree >
          std::max((double)b, vertices[uw.target].out_degree * (1.0 + lambda) + theta)) {
        assert(vertices[uw.target].out_edges[uw.mirror].target == u);
        int elem = remove(u, u_.robin);
        // if (vertices[uw.target].out_edges[uw.mirror].target != u) {
        //   uw.mirror = vertices[uw.target].active_edges;
        // }
        assert(vertices[uw.target].out_edges[uw.mirror].target == u);
        int mirror = insert_directed_worst_case(uw.target, uw.mirror);
        assert(u_.out_edges[elem].target == uw.target);
        // u_.out_edges[u_.robin].mirror = mirror;

        u_.robin++;
        if (u_.robin >= vertices[u].active_edges) {
          u_.robin = 0;
        }
        t_max = t;
        break;
      }
      u_.robin++;  //= ( u_robin+1) % u_.out_edges.size();
    }
    if (u_.robin >= vertices[u].active_edges) {
      u_.robin = 0;
    }

    for (int t = 0; t <= t_max; t++) {
      if (old_robin >= vertices[u].active_edges) {
        old_robin = 0;
      }

      vertices[vertices[u].out_edges[old_robin].target].in_edges.update(
          u, old_robin, u_.out_degree, b, lambda_precomp, offset, vertices);

      // checkAll();
      old_robin++;
    }
    // checkAll();
    return offsetOutArray;
  }
  void delete_directed_worst_case(NT u, int curr_location) {
    remove(u, curr_location);

    auto [non_empty, xu] = vertices[u].in_edges.get_max();
    if (non_empty &&
        vertices[xu.source].out_degree >
            std::max(static_cast<double>(b), (1.0 + lambda) * vertices[u].out_degree + theta)) {
      auto real_xu = vertices[xu.source].out_edges[xu.location_out_neighbors];
      add(u, real_xu.mirror);
      delete_directed_worst_case(xu.source, xu.location_out_neighbors);
    } else {
      for (int t = 0; t < robin_size; t++) {
        if (vertices[u].robin >= vertices[u].active_edges) {
          vertices[u].robin = 0;
        }
        OutEdge uw = vertices[u].out_edges[vertices[u].robin];
        vertices[uw.target].in_edges.update(u, vertices[u].robin, vertices[u].out_degree, b,
                                            lambda_precomp, offset, vertices);
        vertices[u].robin++;
      }
    }
  }
  // add(uv) =1
  // remove(ux)
  // --_> u [x,v];
  //   u [v,x];
  // u=0
  void swap_in_place(NT u, int current_location, int b) {
    if (current_location == b) {
      // std::cout << "WARN" << std::endl;
      return;
    }
    // std::cout << "SWAP " << u << " " << current_location << " " << b << std::endl;
    std::swap(vertices[u].out_edges[current_location], vertices[u].out_edges[b]);
    if (u == g_source && vertices[u].out_edges[current_location].target == g_target) {
      g_offset_uv = current_location;
    }
    if (u == g_target && vertices[u].out_edges[current_location].target == g_source) {
      g_offset_vu = current_location;
    }
    if (u == g_source && vertices[u].out_edges[b].target == g_target) {
      g_offset_uv = b;
    }
    if (u == g_target && vertices[u].out_edges[b].target == g_source) {
      g_offset_vu = b;
    }
    assert(vertices[g_source].out_edges[g_offset_uv].target == g_target);
    assert(vertices[g_target].out_edges[g_offset_vu].target == g_source);

    if (vertices[u].out_edges[current_location].location_in_neighbors != -1) {
      vertices[vertices[u].out_edges[current_location].target].in_edges.update_in_edge(
          vertices[u].out_edges[current_location].bucket,
          vertices[u].out_edges[current_location].location_in_neighbors, current_location);
    }
    assert(vertices[u].out_edges[current_location].mirror != -1);
    {
      auto uw = vertices[u].out_edges[current_location];
      vertices[uw.target].out_edges[uw.mirror].mirror = current_location;
    }
    if (vertices[u].out_edges[b].location_in_neighbors != -1) {
      vertices[vertices[u].out_edges[b].target].in_edges.update_in_edge(
          vertices[u].out_edges[b].bucket, vertices[u].out_edges[b].location_in_neighbors, b);
    }
    assert(vertices[u].out_edges[b].mirror != -1);
    {
      vertices[vertices[u].out_edges[b].target].out_edges[vertices[u].out_edges[b].mirror].mirror =
          b;
    }
  }
  [[nodiscard]] int remove(NT u, int current_location) {
    // decrement the multiplicity of the edge
    assert(vertices[u].out_edges[current_location].count > 0);
    // checkAll();

    vertices[u].out_edges[current_location].count--;

    if (vertices[u].robin >= vertices[u].active_edges) {
      vertices[u].robin = 0;
    }
    int robin = vertices[u].robin;

    assert(current_location < vertices[u].active_edges);
    // if the new multiplicity is 0, remove the element.
    if (vertices[u].out_edges[current_location].count == 0) {
      // std::cout << u << " " << vertices[u].out_edges[current_location].target << "delete"
      //           << std::endl;
      //  del_events++;
      if (vertices[u].out_edges.size() > 1) {
        if (current_location >= robin - 1) {
          swap_in_place(u, current_location, vertices[u].active_edges - 1);
          if (current_location == robin - 1) {
            robin--;
          }
          current_location = vertices[u].active_edges - 1;
          //     std::cout << "V" << std::endl;
        } else {
          //   std::cout << "F" << std::endl;
          // swap robin element with the element to delete
          swap_in_place(u, current_location,
                        robin);  // swap the element to delete (at robin location) with the end
          swap_in_place(u, robin, vertices[u].active_edges - 1);
          current_location = vertices[u].active_edges - 1;
          robin--;
        }
      }
      // vertices[u].out_edges.resize(vertices[u].out_edges.size() - 1);
      vertices[u].active_edges--;
      vertices[vertices[u].out_edges[current_location].target].in_edges.remove(u, current_location,
                                                                               vertices);
      vertices[u].out_edges[current_location].location_in_neighbors = -1;
      // checkAll();
    }
    if (vertices[u].out_edges[current_location].count < 0) {
      std::cerr << "Error: edge count below 0" << std::endl;
      throw;
    }
    vertices[u].out_degree--;
    vertices[u].robin = robin;
    return current_location;
    // checkAll();
  }
};
}  // namespace dyn_delta_approx::app::algorithms::dynamic::edge_orientation