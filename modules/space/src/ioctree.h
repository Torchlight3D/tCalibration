#pragma once

#include <stdint.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>
#include <chrono>
#include <fstream>

#include <Eigen/Geometry>

namespace tl {

template <typename T>
struct PointType_CMP
{
    T point;
    float dist = INFINITY;

    PointType_CMP() {};

    PointType_CMP(T p, float d)
    {
        this->point = p;
        this->dist = d;
    };

    bool operator<(const PointType_CMP &a) const { return dist < a.dist; }
};

// 自动排序堆栈！！首个元素永远是值最大的一个。对于kd树搜索，不需要完全排序，只要首个元素最大即可
template <typename PointType>
class MANUAL_HEAP
{
private:
    PointType_CMP<PointType> *heap;
    int heap_size = 0;
    int cap = 0;

public:
    MANUAL_HEAP(int max_capacity = 100)
    {
        cap = max_capacity;
        heap = new PointType_CMP<PointType>[max_capacity];
        heap_size = 0;
    }

    ~MANUAL_HEAP() { delete[] heap; }

    void pop()
    {
        if (heap_size == 0)
            return;
        heap[0] = heap[heap_size - 1];
        heap_size--;
        MoveDown(0);
        return;
    }

    PointType_CMP<PointType> top() { return heap[0]; }

    float top_v() { return heap[0].dist; }

    void push(PointType_CMP<PointType> point)
    {
        if (heap_size >= cap) {
            if (point < heap[0]) {
                // std::cout<<"point.dist: "<<point.dist<<", heap[0].dist:
                // "<<heap[0].dist<<std::endl;
                pop();
            }
            else {
                // std::cout<<"point.dist: "<<point.dist<<", heap[0].dist:
                // "<<heap[0].dist<<std::endl;
                return;
            }
        }
        heap[heap_size] = point;
        FloatUp(heap_size);
        heap_size++;
        return;
    }

    bool full() { return heap_size >= cap; }

    int size() { return heap_size; }

    void clear()
    {
        heap_size = 0;
        return;
    }

    std::vector<PointType_CMP<PointType>> get_data()
    {
        std::vector<PointType_CMP<PointType>> datas;
        for (int i = 0; i < heap_size; i++) {
            datas.push_back(heap[i]);
        }
        return datas;
    }

private:
    void MoveDown(int heap_index)
    {
        int l = heap_index * 2 + 1;
        PointType_CMP<PointType> tmp = heap[heap_index];
        while (l < heap_size) {
            if (l + 1 < heap_size && heap[l] < heap[l + 1]) {
                l++;
            }
            if (tmp < heap[l]) {
                heap[heap_index] = heap[l];
                heap_index = l;
                l = heap_index * 2 + 1;
            }
            else {
                break;
            }
        }
        heap[heap_index] = tmp;
    }

    void FloatUp(int heap_index)
    {
        int ancestor = (heap_index - 1) / 2;
        PointType_CMP<PointType> tmp = heap[heap_index]; // 新加入的数据
        while (heap_index > 0) {
            if (heap[ancestor] < tmp) {
                heap[heap_index] = heap[ancestor];
                heap_index = ancestor;
                ancestor = (heap_index - 1) / 2;
            }
            else {
                break;
            }
        }
        heap[heap_index] = tmp;
    }
};

struct DistanceIndex
{
    float dist_;
    float *index_;

    DistanceIndex()
    {
        dist_ = std::numeric_limits<float>::max();
        // index_ = std::numeric_limits<size_t>::max();
    }

    DistanceIndex(float dist, float *index) : dist_(dist), index_(index) {}

    bool operator<(const DistanceIndex &dist_index) const
    {
        return dist_ < dist_index.dist_;
    }
};

class KNNSimpleResultSet
{
public:
    KNNSimpleResultSet(size_t capacity_) : capacity_(capacity_)
    {
        // reserving capacity to prevent memory re-allocations
        dist_index_.resize(capacity_, DistanceIndex());
        clear();
    }
    const std::vector<DistanceIndex> &get_data() { return dist_index_; }

    void clear()
    {
        worst_distance_ = std::numeric_limits<float>::max();
        dist_index_[capacity_ - 1].dist_ = worst_distance_;
        count_ = 0;
    }

    size_t size() const { return count_; }

    bool full() const { return count_ == capacity_; }

    void addPoint(float dist, float *index)
    {
        if (dist >= worst_distance_) {
            return;
        }

        if (count_ < capacity_) {
            ++count_;
        }

        size_t i;
        for (i = count_ - 1; i > 0; --i) {
            if (dist_index_[i - 1].dist_ > dist)
                dist_index_[i] = dist_index_[i - 1];
            else {
                break;
            }
        }
        dist_index_[i].dist_ = dist;
        dist_index_[i].index_ = index;
        worst_distance_ = dist_index_[capacity_ - 1].dist_;
    }

    float worstDist() const { return worst_distance_; }

private:
    size_t capacity_;
    size_t count_;
    float worst_distance_;
    std::vector<DistanceIndex> dist_index_;
};

struct RunDetails
{
    int depth;     // 搜索深度
    int pts_n;     // 搜索的点数
    int node_n;    // 搜索的叶子节点数
    bool one_path; // 首次搜索的单条路径
    std::chrono::high_resolution_clock::time_point begin_;
    std::chrono::high_resolution_clock::time_point end_;
    float total_time;

    RunDetails() { clear(); }

    void start() { begin_ = std::chrono::high_resolution_clock::now(); }

    float end()
    {
        end_ = std::chrono::high_resolution_clock::now();
        float time =
            float(std::chrono::duration_cast<std::chrono::microseconds>(end_ -
                                                                        begin_)
                      .count()) /
            1e3;
        begin_ = end_;
        total_time += time;
        return time;
    }

    void clear()
    {
        depth = pts_n = node_n = 0;
        one_path = true;
        total_time = 0.0f;
    }

    void show()
    {
        printf("octree\t depth: %d, node_n: %d, pts_n: %d\n\n", depth, node_n,
               pts_n);
    }

    friend std::ostream &operator<<(std::ostream &os, const RunDetails &c)
    {
        os << "octree depth: " << c.depth << ", node_n: " << c.node_n
           << ", pts_n: " << c.pts_n << std::endl;
        ; // 以"a+bi"的形式输出
        return os;
    }
};

using _Point = Eigen::Vector3f;
using _PointCloud = std::vector<_Point>;

class Octant
{
public:
    bool isActive;
    float x, y, z; // center
    float extent;  // half of side-length
    std::vector<float *> points;
    // Matrix<float> ordered_points;
    std::vector<Octant *> child; // 对于叶子节点可减少 56字节的内存需求

    Octant() : x(0.0f), y(0.0f), z(0.0f), extent(0.0f) { isActive = true; }

    ~Octant()
    {
        if (!child.empty()) {
            for (size_t i = 0; i < 8; ++i) {
                if (child[i]) {
                    delete child[i];
                }
            }
            child.clear();
        }
    }

    size_t size() const
    {
        size_t pts_num = 0;
        get_octant_size(this, pts_num);
        return pts_num;
    }

    void get_octant_size(const Octant *octant, size_t &size_) const
    {
        if (octant->child.empty()) {
            size_ += octant->points.size();
            return;
        }

        for (size_t c = 0; c < 8; ++c) {
            if (octant->child[c]) {
                get_octant_size(octant->child[c], size_);
            }
        }
    }

    void init_child()
    {
        // FIXME: Very bad taste of code
        child.resize(8, new Octant);
    }

    bool inside(const _Point &center, float sqaured_radius) const
    {
        /** \brief test if search ball S(q,r) contains octant
         * @param query    query point
         * @param sqRadius "squared" radius
         * @param octant   pointer to octant
         * @return true, if search ball overlaps with octant, false otherwise.
         */
        // we exploit the symmetry to reduce the test to test
        // whether the farthest corner is inside the search ball.

        // TODO: Use broadcast
        return ((center - Eigen::Vector3f{x, y, z}).cwiseAbs() +
                Eigen::Vector3f::Constant(extent))
                   .squaredNorm() < sqaured_radius;
    }

    bool contains(const _Point &query, float sqaured_radius) const
    {
        /** \brief test if search ball S(q,r) is completely inside octant.
         * @param query   query point
         * @param radius2  radius r*r
         * @param octant  point to octant.
         * @return true, if search ball is completely inside the octant, false
         * otherwise.
         */
        // we exploit the symmetry to reduce the test to test
        // whether the farthest corner is inside the search ball.
        // TODO: Use Eigen
        float x = extent - std::abs(query[0] - this->x);
        float y = extent - std::abs(query[1] - this->y);
        float z = extent - std::abs(query[2] - this->z);
        // octant->extent < radius + std::abs(query[0] - octant->x)
        if (x < 0 || x * x < sqaured_radius)
            return false;
        if (y < 0 || y * y < sqaured_radius)
            return false;
        if (z < 0 || z * z < sqaured_radius)
            return false;
        return true;
    }

    float sqrDist_point2octant(const _Point &query) const
    {
        // 点到box的距离，在边界上或者内部距离为0
        float x = std::max(std::abs(query[0] - this->x) - extent, 0.0f);
        float y = std::max(std::abs(query[1] - this->y) - extent, 0.0f);
        float z = std::max(std::abs(query[2] - this->z) - extent, 0.0f);
        return x * x + y * y + z * z;
    }

    bool overlaps(const _Point &query, float sqRadius) const
    {
        /** \brief test if search ball S(q,r) overlaps with octant
         * @param query   query point
         * @param radius  "squared" radius
         * @param o       pointer to octant
         * @return true, if search ball overlaps with octant, false otherwise.
         */
        // we exploit the symmetry to reduce the test to testing if its inside
        // the Minkowski sum around the positive quadrant.
        float x = std::abs(query[0] - this->x) - extent;
        float y = std::abs(query[1] - this->y) - extent;
        float z = std::abs(query[2] - this->z) - extent;
        // float maxdist = radius + o->extent;
        // Completely outside, since q' is outside the relevant area.
        // std::abs(query[0] - o->x) - o->extent > radius
        if ((x > 0 && x * x > sqRadius) || (y > 0 && y * y > sqRadius) ||
            (z > 0 && z * z > sqRadius)) {
            return false;
        }

        int32_t num_less_extent = (x < 0) + (y < 0) + (z < 0);
        // Checking different cases:
        // a. inside the surface region of the octant.
        if (num_less_extent > 1) {
            return true;
        }

        // b. checking the corner region && edge region.
        x = std::max(x, 0.0f);
        y = std::max(y, 0.0f);
        z = std::max(z, 0.0f);

        return (x * x + y * y + z * z < sqRadius);
    }
};

class InfoRecord
{
public:
    std::vector<Octant *> octant_vec;

    InfoRecord() {}

    void add_octant(Octant *octant)
    {
        if (octant == 0)
            return;
        octant_vec.push_back(octant);
    }

    void write_to_txt(std::string name = "otree_info")
    {
        {
            std::string filename = "/media/zhujun/0DFD06D20DFD06D2/SLAM/"
                                   "octree_test/ikd-Tree-main/data/";
            filename += name + ".txt";
            std::ofstream log_file(filename, std::ios::out);
            log_file << "# x y z extent" << std::endl;
            for (int i = 0; i < octant_vec.size(); i++) {
                log_file << octant_vec[i]->x << " " << octant_vec[i]->y << " "
                         << octant_vec[i]->z << " " << octant_vec[i]->extent
                         << "\n";
            }
            log_file.close();
        }
    }

    void clear() { octant_vec.clear(); }
};

class iOctree
{
public:
    size_t m_bucketSize;
    float m_minExtent;
    bool m_downSize;
    int dim = 4;
    // clang-format off
    inline static constexpr size_t ordered_indies[8][7]{
        {1, 2, 4, 3, 5, 6, 7},
        {0, 3, 5, 2, 4, 7, 6},
        {0, 3, 6, 1, 4, 7, 5},
        {1, 2, 7, 0, 5, 6, 4},
        {0, 5, 6, 1, 2, 7, 3},
        {1, 4, 7, 0, 3, 6, 2},
        {2, 4, 7, 0, 3, 5, 1},
        {3, 5, 6, 1, 2, 4, 0}};
    // clang-format on
    bool ordered;
    RunDetails run_details;
    InfoRecord info_record;

    iOctree();
    iOctree(size_t bucketSize_, bool copyPoints_, float minExtent_);
    iOctree(size_t bucketSize_, bool copyPoints_, float minExtent_, int dim_);
    ~iOctree();

    void set_order(bool ordered_ = false);

    void set_min_extent(float extent); // 网格最小内接园半径

    void set_bucket_size(size_t bucket_size);

    void set_down_size(bool down_size);

    void initialize(_PointCloud &pts_);

    void update(_PointCloud &pts_, bool down_size = false);

    void clear();

    void radiusNeighbors(const _Point &query, float radius,
                         std::vector<size_t> &resultIndices);

    void radiusNeighbors(const _Point &query, float radius,
                         std::vector<size_t> &resultIndices,
                         std::vector<float> &distances);

    void radiusNeighbors(const _Point &query, float radius,
                         std::vector<_Point> &resultIndices,
                         std::vector<float> &distances);

    int32_t knnNeighbors(const _Point &query, int k,
                         std::vector<_Point> &resultIndices,
                         std::vector<float> &distances);

    int32_t knnNeighbors(const _Point &query, int k,
                         std::vector<size_t> &resultIndices,
                         std::vector<float> &distances);

    int32_t knnNeighbors(const _Point &query, int k,
                         std::vector<int> &resultIndices,
                         std::vector<float> &distances);

    void boxWiseDelete(const Eigen::AlignedBox3f &box_range, bool clear_data);

    size_t size() const;

    void get_nodes(Octant *octant, std::vector<Octant *> &nodes,
                   float min_extent = 0);

    _PointCloud get_data();

    void write_node_info_2_txt(float min_extent = 0);

    void get_leaf_nodes(const Octant *octant,
                        std::vector<const Octant *> &nodes);

    void initialize_record(const _PointCloud &pts_);

    Octant *createOctant_record(float x, float y, float z, float extent,
                                std::vector<float *> &points);

    void update_record(const _PointCloud &pts_, bool down_size = false);

    void updateOctant_record(Octant *octant,
                             const std::vector<float *> &points);

    void write_info_record(std::string name = "otree_info", bool clear = false);

    void write_remain_node_record(std::string name = "otree_info");

protected:
    Octant *m_root_;
    size_t last_pts_num, pts_num_deleted; // 主要为了确定每个点的索引

    iOctree(iOctree &);
    iOctree &operator=(const iOctree &oct);

    Octant *createOctant(float x, float y, float z, float extent,
                         std::vector<float *> &points);

    void updateOctant(Octant *octant, const std::vector<float *> &points);

    void radiusNeighbors(const Octant *octant, const _Point &query,
                         float radius, float sqrRadius,
                         std::vector<float *> &resultIndices);

    void radiusNeighbors(const Octant *octant, const _Point &query,
                         float radius, float sqrRadius,
                         std::vector<float *> &resultIndices,
                         std::vector<float> &distances);

    bool radiusNeighbors2(const Octant *octant, const _Point &query,
                          float sqrRadius, std::vector<size_t> &resultIndices,
                          std::vector<float> &distances);

    bool knnNeighbors(const Octant *octant, const _Point &query,
                      KNNSimpleResultSet &heap);

    // TODO: Not implemented
    bool knnNeighbors(const Octant *octant, const _Point &query,
                      MANUAL_HEAP<size_t> &heap);

    void boxWiseDelete(Octant *octant, const Eigen::AlignedBox3f &box_range,
                       bool &deleted, bool clear_data);
};

} // namespace tl
