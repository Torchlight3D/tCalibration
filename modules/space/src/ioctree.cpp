#include "ioctree.h"

namespace tl {

iOctree::iOctree()
    : m_bucketSize(32), m_minExtent(0.01f), m_root_(0), m_downSize(false)
{
    ordered = true;
    pts_num_deleted = last_pts_num = 0;
    dim = 4;
}

iOctree::iOctree(size_t bucketSize_, bool copyPoints_, float minExtent_)
    : m_bucketSize(bucketSize_),
      m_minExtent(minExtent_),
      m_root_(0),
      m_downSize(false)
{
    ordered = true;
    pts_num_deleted = last_pts_num = 0;
    dim = 4;
}

iOctree::iOctree(size_t bucketSize_, bool copyPoints_, float minExtent_,
                 int dim_)
    : m_bucketSize(bucketSize_),
      m_minExtent(minExtent_),
      m_root_(0),
      m_downSize(false)
{
    ordered = true;
    pts_num_deleted = last_pts_num = 0;
    dim = 4;
    if (dim_ > 4) {
        dim = dim_;
    }
}

iOctree::~iOctree() { clear(); }

void iOctree::set_order(bool ordered_) { ordered = ordered_; }

void iOctree::set_min_extent(float extent) // 网格最小内接园半径
{
    m_minExtent = extent;
}

void iOctree::set_bucket_size(size_t bucket_size)
{
    m_bucketSize = bucket_size;
}

void iOctree::set_down_size(bool down_size) { m_downSize = down_size; }

void iOctree::initialize(_PointCloud &pts_)
{
    clear();
    const size_t pts_num = pts_.size();
    std::vector<float *> points;
    int dim_ = 3;
    points.resize(pts_num, 0);
    size_t cloud_index = 0;
    float min[3], max[3];
    for (const auto &point : pts_) {
        const float &x = point.x();
        const float &y = point.y();
        const float &z = point.z();
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            continue;
        }

        float *cloud_ptr = new float[dim];
        cloud_ptr[0] = x;
        cloud_ptr[1] = y;
        cloud_ptr[2] = z;
        cloud_ptr[3] = float(cloud_index); // 保存在**原始**数据中的索引
        points[cloud_index] = cloud_ptr;
        if (cloud_index == 0) {
            min[0] = max[0] = x;
            min[1] = max[1] = y;
            min[2] = max[2] = z;
        }
        else {
            min[0] = x < min[0] ? x : min[0];
            min[1] = y < min[1] ? y : min[1];
            min[2] = z < min[2] ? z : min[2];
            max[0] = x > max[0] ? x : max[0];
            max[1] = y > max[1] ? y : max[1];
            max[2] = z > max[2] ? z : max[2];
        }
        cloud_index++;
    }
    last_pts_num = cloud_index;
    points.resize(cloud_index); // 删除多余元素
    float ctr[3] = {min[0], min[1], min[2]};
    float maxextent = 0.5f * (max[0] - min[0]);
    maxextent = std::max(maxextent, 0.01f);
    ctr[0] += maxextent;
    for (size_t i = 1; i < 3; ++i) {
        float extent = 0.5f * (max[i] - min[i]);
        ctr[i] += extent;
        if (extent > maxextent) {
            maxextent = extent;
        }
    }
    // std::cout<<"maxextent: "<<maxextent<<", "
    // 		<<"min: "<<min[0]<<", "<<min[1]<<", "<<min[2]<<", "
    // 		<<"max: "<<max[0]<<", "<<max[1]<<", "<<max[2]<<", "
    // 		<<std::endl;
    // m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1,
    // N);
    m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, points);
    // std::cout<<"createOctant success!"<<std::endl;
    for (auto &point : points) {
        delete[] point;
    }
}

void iOctree::initialize_record(const _PointCloud &pts_)
{
    clear();
    const size_t pts_num = pts_.size();
    std::vector<float *> points;
    points.resize(pts_num, 0);
    size_t cloud_index = 0;
    float min[3], max[3];
    for (const auto &point : pts_) {
        const float &x = point.x();
        const float &y = point.y();
        const float &z = point.z();
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            continue;
        }

        float *cloud_ptr = new float[dim];
        cloud_ptr[0] = x;
        cloud_ptr[1] = y;
        cloud_ptr[2] = z;
        cloud_ptr[3] = float(cloud_index); // 保存在**原始**数据中的索引
        points[cloud_index] = cloud_ptr;
        if (cloud_index == 0) {
            min[0] = max[0] = x;
            min[1] = max[1] = y;
            min[2] = max[2] = z;
        }
        else {
            min[0] = x < min[0] ? x : min[0];
            min[1] = y < min[1] ? y : min[1];
            min[2] = z < min[2] ? z : min[2];
            max[0] = x > max[0] ? x : max[0];
            max[1] = y > max[1] ? y : max[1];
            max[2] = z > max[2] ? z : max[2];
        }
        cloud_index++;
    }
    last_pts_num = cloud_index;
    points.resize(cloud_index); // 删除多余元素
    float ctr[3] = {min[0], min[1], min[2]};
    float maxextent = 0.5f * (max[0] - min[0]);
    maxextent = std::max(maxextent, 0.01f);
    ctr[0] += maxextent;
    for (size_t i = 1; i < 3; ++i) {
        float extent = 0.5f * (max[i] - min[i]);
        ctr[i] += extent;
        if (extent > maxextent) {
            maxextent = extent;
        }
    }
    // std::cout<<"maxextent: "<<maxextent<<", "
    // 		<<"min: "<<min[0]<<", "<<min[1]<<", "<<min[2]<<", "
    // 		<<"max: "<<max[0]<<", "<<max[1]<<", "<<max[2]<<", "
    // 		<<std::endl;
    // m_root_ = createOctant(ctr[0], ctr[1], ctr[2], maxextent, 0, N - 1,
    // N);
    m_root_ = createOctant_record(ctr[0], ctr[1], ctr[2], maxextent, points);
    // std::cout<<"createOctant success!"<<std::endl;
    for (auto &point : points) {
        delete[] point;
    }
}

void iOctree::update_record(const _PointCloud &pts_, bool down_size)
{
    if (!m_root_) {
        initialize_record(pts_);
        return;
    }
    // std::cout<<"update start\n";
    m_downSize = down_size;
    size_t pts_num = pts_.size();
    // std::cout<<"updateOctant init: "<<pts_num<<std::endl;
    std::vector<float *> points_tmp;
    int dim_ = 3;
    points_tmp.resize(pts_num, 0);
    size_t cloud_index = 0;
    float min[3], max[3];
    const size_t N_old = last_pts_num;
    for (const auto &point : pts_) {
        const float &x = point.x();
        const float &y = point.y();
        const float &z = point.z();
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            continue;
        }

        float *cloud_ptr = new float[dim];
        cloud_ptr[0] = x;
        cloud_ptr[1] = y;
        cloud_ptr[2] = z;
        cloud_ptr[3] = N_old + cloud_index;
        points_tmp[cloud_index] = cloud_ptr;
        if (cloud_index == 0) {
            min[0] = max[0] = x;
            min[1] = max[1] = y;
            min[2] = max[2] = z;
        }
        else {
            min[0] = x < min[0] ? x : min[0];
            min[1] = y < min[1] ? y : min[1];
            min[2] = z < min[2] ? z : min[2];
            max[0] = x > max[0] ? x : max[0];
            max[1] = y > max[1] ? y : max[1];
            max[2] = z > max[2] ? z : max[2];
        }
        cloud_index++;
    }

    if (cloud_index == 0) {
        return;
    }

    points_tmp.resize(cloud_index);
    // std::cout<<"updateOctant filter: "<<cloud_index<<std::endl;
    // 先创建一个对当前节点全包围的父节点，首先确定父节点中心所在的方向
    constexpr float factor[] = {-0.5f, 0.5f};
    // 判断是否存在越界
    while (std::abs(max[0] - m_root_->x) > m_root_->extent ||
           std::abs(max[1] - m_root_->y) > m_root_->extent ||
           std::abs(max[2] - m_root_->z) > m_root_->extent) {
        // 父节点中心坐标
        float parentExtent = 2 * m_root_->extent;
        float parentX = m_root_->x + factor[max[0] > m_root_->x] * parentExtent;
        float parentY = m_root_->y + factor[max[1] > m_root_->y] * parentExtent;
        float parentZ = m_root_->z + factor[max[2] > m_root_->z] * parentExtent;
        // 构造父节点
        Octant *octant = new Octant;
        octant->x = parentX;
        octant->y = parentY;
        octant->z = parentZ;
        octant->extent = parentExtent;
        octant->init_child();
        size_t mortonCode = 0;
        if (m_root_->x > parentX)
            mortonCode |= 1;
        if (m_root_->y > parentY)
            mortonCode |= 2;
        if (m_root_->z > parentZ)
            mortonCode |= 4;
        octant->child[mortonCode] = m_root_;
        m_root_ = octant;
        info_record.add_octant(octant);
    }

    while (std::abs(min[0] - m_root_->x) > m_root_->extent ||
           std::abs(min[1] - m_root_->y) > m_root_->extent ||
           std::abs(min[2] - m_root_->z) > m_root_->extent) {
        // 父节点中心坐标
        float parentExtent = 2 * m_root_->extent;
        float parentX = m_root_->x + factor[min[0] > m_root_->x] * parentExtent;
        float parentY = m_root_->y + factor[min[1] > m_root_->y] * parentExtent;
        float parentZ = m_root_->z + factor[min[2] > m_root_->z] * parentExtent;
        // 构造父节点
        Octant *octant = new Octant;
        // octant->isLeaf = false;
        octant->x = parentX;
        octant->y = parentY;
        octant->z = parentZ;
        octant->extent = parentExtent;
        octant->init_child();
        size_t mortonCode = 0;
        if (m_root_->x > parentX)
            mortonCode |= 1;
        if (m_root_->y > parentY)
            mortonCode |= 2;
        if (m_root_->z > parentZ)
            mortonCode |= 4;
        octant->child[mortonCode] = m_root_;
        m_root_ = octant;
        info_record.add_octant(octant);
    }

    if (points_tmp.empty()) {
        return;
    }
    // std::cout<<"updateOctant start: "<<points_tmp.size()<<std::endl;;
    last_pts_num += points_tmp.size();
    updateOctant_record(m_root_, points_tmp);
    // std::cout<<"updateOctant end\n";
    for (auto &point : points_tmp) {
        delete[] point;
    }
}

void iOctree::update(_PointCloud &pts_, bool down_size)
{
    if (!m_root_) {
        initialize(pts_);
        return;
    }

    // std::cout<<"update start\n";
    m_downSize = down_size;
    size_t pts_num = pts_.size();
    // std::cout<<"updateOctant init: "<<pts_num<<std::endl;
    std::vector<float *> points_tmp;
    int dim_ = 3;
    points_tmp.resize(pts_num, 0);
    size_t cloud_index = 0;
    float min[3], max[3];
    const size_t N_old = last_pts_num;
    for (const auto &point : pts_) {
        const float &x = point.x();
        const float &y = point.y();
        const float &z = point.z();
        if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
            continue;
        }
        float *cloud_ptr = new float[dim];
        cloud_ptr[0] = x;
        cloud_ptr[1] = y;
        cloud_ptr[2] = z;
        cloud_ptr[3] = N_old + cloud_index;
        points_tmp[cloud_index] = cloud_ptr;
        if (cloud_index == 0) {
            min[0] = max[0] = x;
            min[1] = max[1] = y;
            min[2] = max[2] = z;
        }
        else {
            min[0] = x < min[0] ? x : min[0];
            min[1] = y < min[1] ? y : min[1];
            min[2] = z < min[2] ? z : min[2];
            max[0] = x > max[0] ? x : max[0];
            max[1] = y > max[1] ? y : max[1];
            max[2] = z > max[2] ? z : max[2];
        }
        cloud_index++;
    }

    if (cloud_index == 0) {
        return;
    }

    points_tmp.resize(cloud_index);
    // std::cout<<"updateOctant filter: "<<cloud_index<<std::endl;
    // 先创建一个对当前节点全包围的父节点，首先确定父节点中心所在的方向
    constexpr float factor[] = {-0.5f, 0.5f};
    // 判断是否存在越界
    while (std::abs(max[0] - m_root_->x) > m_root_->extent ||
           std::abs(max[1] - m_root_->y) > m_root_->extent ||
           std::abs(max[2] - m_root_->z) > m_root_->extent) {
        // 父节点中心坐标
        float parentExtent = 2 * m_root_->extent;
        float parentX = m_root_->x + factor[max[0] > m_root_->x] * parentExtent;
        float parentY = m_root_->y + factor[max[1] > m_root_->y] * parentExtent;
        float parentZ = m_root_->z + factor[max[2] > m_root_->z] * parentExtent;
        // 构造父节点
        Octant *octant = new Octant;
        octant->x = parentX;
        octant->y = parentY;
        octant->z = parentZ;
        octant->extent = parentExtent;
        octant->init_child();
        size_t mortonCode = 0;
        if (m_root_->x > parentX)
            mortonCode |= 1;
        if (m_root_->y > parentY)
            mortonCode |= 2;
        if (m_root_->z > parentZ)
            mortonCode |= 4;
        octant->child[mortonCode] = m_root_;
        m_root_ = octant;
    }
    while (std::abs(min[0] - m_root_->x) > m_root_->extent ||
           std::abs(min[1] - m_root_->y) > m_root_->extent ||
           std::abs(min[2] - m_root_->z) > m_root_->extent) {
        // 父节点中心坐标
        float parentExtent = 2 * m_root_->extent;
        float parentX = m_root_->x + factor[min[0] > m_root_->x] * parentExtent;
        float parentY = m_root_->y + factor[min[1] > m_root_->y] * parentExtent;
        float parentZ = m_root_->z + factor[min[2] > m_root_->z] * parentExtent;
        // 构造父节点
        Octant *octant = new Octant;
        // octant->isLeaf = false;
        octant->x = parentX;
        octant->y = parentY;
        octant->z = parentZ;
        octant->extent = parentExtent;
        octant->init_child();
        size_t mortonCode = 0;
        if (m_root_->x > parentX)
            mortonCode |= 1;
        if (m_root_->y > parentY)
            mortonCode |= 2;
        if (m_root_->z > parentZ)
            mortonCode |= 4;
        octant->child[mortonCode] = m_root_;
        m_root_ = octant;
    }

    if (points_tmp.empty()) {
        return;
    }

    // std::cout<<"updateOctant start: "<<points_tmp.size()<<std::endl;;
    last_pts_num += points_tmp.size();
    updateOctant(m_root_, points_tmp);
    // std::cout<<"updateOctant end\n";
    for (auto &point : points_tmp) {
        delete[] point;
    }
}

void iOctree::boxWiseDelete(const Eigen::AlignedBox3f &box_range,
                            bool clear_data)
{
    if (!m_root_) {
        return;
    }

    bool deleted = false;
    boxWiseDelete(m_root_, box_range, deleted, clear_data);

    if (deleted) {
        m_root_ = nullptr;
    }
}

size_t iOctree::size() const { return last_pts_num - pts_num_deleted; }

void iOctree::get_nodes(Octant *octant, std::vector<Octant *> &nodes,
                        float min_extent)
{
    if (!octant) {
        return;
    }

    if (min_extent > 0) {
        if (octant->extent <= min_extent) {
            nodes.push_back(octant);
            return;
        }
    }
    nodes.push_back(octant);

    if (octant->child.empty()) {
        // nodes.push_back(octant);
        return;
    }

    for (const auto &child : octant->child) {
        get_nodes(child, nodes, min_extent);
    }
}

_PointCloud iOctree::get_data()
{
    std::vector<Octant *> nodes;
    _PointCloud pts;
    get_nodes(m_root_, nodes);
    for (auto octant : nodes) {
        for (auto p : octant->points) {
            _Point pt;
            pt.x() = p[0];
            pt.y() = p[1];
            pt.z() = p[2];
            pts.push_back(pt);
        }
    }
    return pts;
}

void iOctree::clear()
{
    delete m_root_;
    m_root_ = 0;
}

void iOctree::radiusNeighbors(const _Point &query, float radius,
                              std::vector<size_t> &resultIndices)
{
    resultIndices.clear();
    if (!m_root_) {
        return;
    }

    float sqrRadius = radius * radius; // "squared" radius
    std::vector<float *> points_ptr;
    radiusNeighbors(m_root_, query, radius, sqrRadius, points_ptr);
    resultIndices.resize(points_ptr.size());
    for (size_t i = 0; i < points_ptr.size(); i++) {
        resultIndices[i] = size_t(points_ptr[i][3]);
    }
}

void iOctree::radiusNeighbors(const _Point &query, float radius,
                              std::vector<size_t> &resultIndices,
                              std::vector<float> &distances)
{
    resultIndices.clear();
    distances.clear();
    if (!m_root_) {
        return;
    }

    float sqrRadius = radius * radius; // "squared" radius
    std::vector<float *> points_ptr;
    radiusNeighbors(m_root_, query, radius, sqrRadius, points_ptr, distances);
    // radiusNeighbors2(m_root_, query_, sqrRadius, resultIndices,
    // distances);
    resultIndices.resize(points_ptr.size());
    for (size_t i = 0; i < points_ptr.size(); i++) {
        resultIndices[i] = size_t(points_ptr[i][3]);
    }
}

void iOctree::radiusNeighbors(const _Point &query, float radius,
                              std::vector<_Point> &resultIndices,
                              std::vector<float> &distances)
{
    resultIndices.clear();
    distances.clear();
    if (!m_root_) {
        return;
    }
    float sqrRadius = radius * radius; // "squared" radius
    std::vector<float *> points_ptr;
    radiusNeighbors(m_root_, query, radius, sqrRadius, points_ptr, distances);
    // radiusNeighbors2(m_root_, query_, sqrRadius, resultIndices,
    // distances);
    resultIndices.resize(points_ptr.size());
    for (size_t i = 0; i < resultIndices.size(); i++) {
        _Point pt;
        pt.x() = points_ptr[i][0];
        pt.y() = points_ptr[i][1];
        pt.z() = points_ptr[i][2];
        resultIndices[i] = pt;
    }
}

int32_t iOctree::knnNeighbors(const _Point &query, int k,
                              std::vector<_Point> &resultIndices,
                              std::vector<float> &distances)
{
    if (!m_root_) {
        return 0;
    }

    // MANUAL_HEAP<size_t> heap(k);
    // knnNeighbors(m_root_, query, heap);
    // std::cout<<"knnNeighbors start"<<std::endl;

    // run_details.clear();
    // run_details.start();
    KNNSimpleResultSet heap(k);
    knnNeighbors(m_root_, query, heap);
    // run_details.end();
    // run_details.show();
    std::vector<DistanceIndex> data = heap.get_data();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());
    for (size_t i = 0; i < heap.size(); i++) {
        _Point pt;
        pt.x() = data[i].index_[0];
        pt.y() = data[i].index_[1];
        pt.z() = data[i].index_[2];
        resultIndices[i] = pt;
        distances[i] = data[i].dist_;
    }
    // run_details.end();
    // run_details.show();
    return data.size();
}

int32_t iOctree::knnNeighbors(const _Point &query, int k,
                              std::vector<size_t> &resultIndices,
                              std::vector<float> &distances)
{
    if (!m_root_) {
        return 0;
    }

    // MANUAL_HEAP<size_t> heap(k);
    // knnNeighbors(m_root_, query, heap);
    // std::cout<<"knnNeighbors start"<<std::endl;

    // run_details.clear();
    // run_details.start();
    KNNSimpleResultSet heap(k);
    knnNeighbors(m_root_, query, heap);
    // run_details.end();
    // run_details.show();
    std::vector<DistanceIndex> data = heap.get_data();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());
    for (int i = 0; i < heap.size(); i++) {
        resultIndices[i] = size_t(data[i].index_[3]);
        distances[i] = data[i].dist_;
    }
    // run_details.end();
    // run_details.show();
    return data.size();
}

int32_t iOctree::knnNeighbors(const _Point &query, int k,
                              std::vector<int> &resultIndices,
                              std::vector<float> &distances)
{
    if (!m_root_) {
        return 0;
    }

    // MANUAL_HEAP<size_t> heap(k);
    // knnNeighbors(m_root_, query, heap);
    // std::cout<<"knnNeighbors start"<<std::endl;

    // run_details.clear();
    // run_details.start();
    KNNSimpleResultSet heap(k);
    knnNeighbors(m_root_, query, heap);
    // run_details.end();
    // run_details.show();
    std::vector<DistanceIndex> data = heap.get_data();
    resultIndices.resize(heap.size());
    distances.resize(heap.size());
    for (int i = 0; i < heap.size(); i++) {
        resultIndices[i] = int(data[i].index_[3]);
        distances[i] = data[i].dist_;
    }
    // run_details.end();
    // run_details.show();
    return data.size();
}

void iOctree::write_node_info_2_txt(float min_extent)
{
    if (!m_root_) {
        return;
    }

    std::vector<Octant *> nodes;
    get_nodes(m_root_, nodes);

    {
        std::ofstream fout(
            "/media/zhujun/0DFD06D20DFD06D2/SLAM/octree_test/ikd-Tree-main/"
            "data/node_info.txt");
        fout << "# x y z extent" << std::endl;
        for (const auto &node : nodes) {
            fout << node->x << " " << node->y << " " << node->z << " "
                 << node->extent << "\n";
        }
    }
}

void iOctree::get_leaf_nodes(const Octant *octant,
                             std::vector<const Octant *> &nodes)
{
    if (!octant) {
        return;
    }

    if (octant->child.empty()) {
        nodes.push_back(octant);
        return;
    }

    nodes.reserve(nodes.size() + 8);
    for (const auto &child : octant->child) {
        get_leaf_nodes(child, nodes);
    }
}

Octant *iOctree::createOctant_record(float x, float y, float z, float extent,
                                     std::vector<float *> &points)
{
    // For a leaf we don't have to change anything; points are already
    // correctly linked or correctly reordered.
    auto *octant = new Octant;
    const size_t size = points.size();
    octant->x = x;
    octant->y = y;
    octant->z = z;
    octant->extent = extent;
    info_record.add_octant(octant);
    constexpr float factor[] = {-0.5f, 0.5f};
    if (size > m_bucketSize && extent > 2 * m_minExtent) // 32 0
    {
        std::vector<std::vector<float *>> child_points(8,
                                                       std::vector<float *>());
        for (size_t i = 0; i < size; ++i) {
            float *p = points[i];
            size_t mortonCode = 0;
            if (p[0] > x)
                mortonCode |= 1;
            if (p[1] > y)
                mortonCode |= 2;
            if (p[2] > z)
                mortonCode |= 4;
            child_points[mortonCode].push_back(p);
        }
        // now, we can create the child nodes...
        float childExtent = 0.5f * extent;
        octant->init_child();
        for (size_t i = 0; i < 8; ++i) {
            if (child_points[i].empty()) {
                continue;
            }

            float childX = x + factor[(i & 1) > 0] * extent;
            float childY = y + factor[(i & 2) > 0] * extent;
            float childZ = z + factor[(i & 4) > 0] * extent;
            octant->child[i] = createOctant_record(
                childX, childY, childZ, childExtent, child_points[i]);
        }
    }
    else {
        const size_t size = points.size();
        octant->points.resize(size, 0);
        float *continue_points = new float[size * dim];
        for (size_t i = 0; i < size; ++i) {
            std::copy(points[i], points[i] + dim, continue_points + dim * i);
            octant->points[i] = continue_points + dim * i;
        }
    }
    return octant;
}

void iOctree::updateOctant_record(Octant *octant,
                                  const std::vector<float *> &points)
{
    // std::cout<<"updateOctant0 start "<<points.size()<<std::endl;
    constexpr float factor[] = {-0.5f, 0.5f};
    const float x = octant->x, y = octant->y, z = octant->z,
                extent = octant->extent;
    octant->isActive = true; // 更新状态
    if (octant->child.empty()) {
        if (octant->points.size() + points.size() > m_bucketSize &&
            extent > 2 * m_minExtent) // 32 0
        {
            octant->points.insert(octant->points.end(), points.begin(),
                                  points.end());
            const size_t size = octant->points.size();
            std::vector<std::vector<float *>> child_points(
                8, std::vector<float *>());
            for (size_t i = 0; i < size; ++i) {
                size_t mortonCode = 0;
                if (octant->points[i][0] > x)
                    mortonCode |= 1;
                if (octant->points[i][1] > y)
                    mortonCode |= 2;
                if (octant->points[i][2] > z)
                    mortonCode |= 4;
                child_points[mortonCode].push_back(octant->points[i]);
            }
            float childExtent = 0.5f * extent;
            octant->init_child();
            for (size_t i = 0; i < 8; ++i) {
                if (child_points[i].empty()) {
                    continue;
                }
                float childX = x + factor[(i & 1) > 0] * extent;
                float childY = y + factor[(i & 2) > 0] * extent;
                float childZ = z + factor[(i & 4) > 0] * extent;
                octant->child[i] = createOctant_record(
                    childX, childY, childZ, childExtent, child_points[i]);
            }
            delete[] octant->points[0];
            std::vector<float *>().swap(octant->points); // 清空非叶子节点索引
        }
        else {
            //* 如果有下采样且满足条件，直接不添加
            if (m_downSize && extent <= 2 * m_minExtent &&
                octant->points.size() > m_bucketSize / 8) {
                return;
            }
            octant->points.insert(octant->points.end(), points.begin(),
                                  points.end());
            const size_t size = octant->points.size();
            float *continue_points = new float[size * dim];
            float *old_points = octant->points[0];
            for (size_t i = 0; i < size; ++i) {
                std::copy(octant->points[i], octant->points[i] + dim,
                          continue_points + dim * i);
                octant->points[i] = continue_points + dim * i;
            }
            delete[] old_points;
        }
    }
    else {
        const size_t size = points.size();
        std::vector<std::vector<float *>> child_points(8,
                                                       std::vector<float *>());
        for (size_t i = 0; i < size; ++i) {
            size_t mortonCode = 0;
            if (points[i][0] > x)
                mortonCode |= 1;
            if (points[i][1] > y)
                mortonCode |= 2;
            if (points[i][2] > z)
                mortonCode |= 4;
            child_points[mortonCode].push_back(points[i]);
        }
        float childExtent = 0.5f * extent;
        for (size_t i = 0; i < 8; ++i) {
            // 可能存在某些节点没有新分配点，但是存在点的情况！！！
            if (!child_points[i].empty()) {
                if (!octant->child[i]) {
                    float childX = x + factor[(i & 1) > 0] * extent;
                    float childY = y + factor[(i & 2) > 0] * extent;
                    float childZ = z + factor[(i & 4) > 0] * extent;
                    octant->child[i] = createOctant_record(
                        childX, childY, childZ, childExtent, child_points[i]);
                }
                else {
                    updateOctant_record(octant->child[i], child_points[i]);
                }
            }
        }
    }
}

void iOctree::write_info_record(std::string name, bool clear)
{
    info_record.write_to_txt(name);
    if (clear) {
        info_record.clear();
    }
}

void iOctree::write_remain_node_record(std::string name)
{
    if (!m_root_) {
        return;
    }

    std::vector<Octant *> nodes;
    get_nodes(m_root_, nodes);

    std::string filename = "/media/zhujun/0DFD06D20DFD06D2/SLAM/"
                           "octree_test/ikd-Tree-main/data/";
    filename += name + ".txt";
    {
        std::ofstream fout(filename);
        fout << "# x y z extent" << std::endl;
        for (int i = 0; i < nodes.size(); i++) {
            fout << nodes[i]->x << " " << nodes[i]->y << " " << nodes[i]->z
                 << " " << nodes[i]->extent << "\n";
        }
    }
}

Octant *iOctree::createOctant(float x, float y, float z, float extent,
                              std::vector<float *> &points)
{
    // For a leaf we don't have to change anything; points are already
    // correctly linked or correctly reordered.
    Octant *octant = new Octant;
    const size_t size = points.size();
    octant->x = x;
    octant->y = y;
    octant->z = z;
    octant->extent = extent;
    constexpr float factor[] = {-0.5f, 0.5f};
    if (size > m_bucketSize && extent > 2 * m_minExtent) // 32 0
    {
        std::vector<std::vector<float *>> child_points(8,
                                                       std::vector<float *>());
        for (size_t i = 0; i < size; ++i) {
            float *p = points[i];
            size_t mortonCode = 0;
            if (p[0] > x)
                mortonCode |= 1;
            if (p[1] > y)
                mortonCode |= 2;
            if (p[2] > z)
                mortonCode |= 4;
            child_points[mortonCode].push_back(p);
        }
        // now, we can create the child nodes...
        float childExtent = 0.5f * extent;
        octant->init_child();
        for (size_t i = 0; i < 8; ++i) {
            if (child_points[i].empty()) {
                continue;
            }

            float childX = x + factor[(i & 1) > 0] * extent;
            float childY = y + factor[(i & 2) > 0] * extent;
            float childZ = z + factor[(i & 4) > 0] * extent;
            octant->child[i] = createOctant(childX, childY, childZ, childExtent,
                                            child_points[i]);
        }
    }
    else {
        const size_t size = points.size();
        octant->points.resize(size, 0);
        float *continue_points = new float[size * dim];
        for (size_t i = 0; i < size; ++i) {
            std::copy(points[i], points[i] + dim, continue_points + dim * i);
            octant->points[i] = continue_points + dim * i;
        }
    }
    return octant;
}

void iOctree::updateOctant(Octant *octant, const std::vector<float *> &points)
{
    // std::cout<<"updateOctant0 start "<<points.size()<<std::endl;
    constexpr float factor[] = {-0.5f, 0.5f};
    const float x = octant->x, y = octant->y, z = octant->z,
                extent = octant->extent;
    octant->isActive = true; // 更新状态
    if (octant->child.empty()) {
        if (octant->points.size() + points.size() > m_bucketSize &&
            extent > 2 * m_minExtent) // 32 0
        {
            octant->points.insert(octant->points.end(), points.begin(),
                                  points.end());
            const size_t size = octant->points.size();
            std::vector<std::vector<float *>> child_points(
                8, std::vector<float *>());
            for (size_t i = 0; i < size; ++i) {
                size_t mortonCode = 0;
                if (octant->points[i][0] > x)
                    mortonCode |= 1;
                if (octant->points[i][1] > y)
                    mortonCode |= 2;
                if (octant->points[i][2] > z)
                    mortonCode |= 4;
                child_points[mortonCode].push_back(octant->points[i]);
            }
            float childExtent = 0.5f * extent;
            octant->init_child();
            for (size_t i = 0; i < 8; ++i) {
                if (child_points[i].empty()) {
                    continue;
                }
                float childX = x + factor[(i & 1) > 0] * extent;
                float childY = y + factor[(i & 2) > 0] * extent;
                float childZ = z + factor[(i & 4) > 0] * extent;
                octant->child[i] = createOctant(childX, childY, childZ,
                                                childExtent, child_points[i]);
            }
            delete[] octant->points[0];
            std::vector<float *>().swap(octant->points); // 清空非叶子节点索引
        }
        else {
            //* 如果有下采样且满足条件，直接不添加
            if (m_downSize && extent <= 2 * m_minExtent &&
                octant->points.size() > m_bucketSize / 8) {
                return;
            }

            octant->points.insert(octant->points.end(), points.begin(),
                                  points.end());
            const size_t size = octant->points.size();
            float *continue_points = new float[size * dim];
            float *old_points = octant->points[0];
            for (size_t i = 0; i < size; ++i) {
                std::copy(octant->points[i], octant->points[i] + dim,
                          continue_points + dim * i);
                octant->points[i] = continue_points + dim * i;
            }
            delete[] old_points;
        }
    }
    else {
        const size_t size = points.size();
        std::vector<std::vector<float *>> child_points(8,
                                                       std::vector<float *>());
        for (size_t i = 0; i < size; ++i) {
            size_t mortonCode = 0;
            if (points[i][0] > x)
                mortonCode |= 1;
            if (points[i][1] > y)
                mortonCode |= 2;
            if (points[i][2] > z)
                mortonCode |= 4;
            child_points[mortonCode].push_back(points[i]);
        }
        float childExtent = 0.5f * extent;
        for (size_t i = 0; i < 8; ++i) {
            // 可能存在某些节点没有新分配点，但是存在点的情况！！！
            if (!child_points[i].empty()) {
                if (!octant->child[i]) {
                    float childX = x + factor[(i & 1) > 0] * extent;
                    float childY = y + factor[(i & 2) > 0] * extent;
                    float childZ = z + factor[(i & 4) > 0] * extent;
                    octant->child[i] = createOctant(
                        childX, childY, childZ, childExtent, child_points[i]);
                }
                else {
                    updateOctant(octant->child[i], child_points[i]);
                }
            }
        }
    }
}

void iOctree::radiusNeighbors(const Octant *octant, const _Point &query,
                              float radius, float sqrRadius,
                              std::vector<float *> &resultIndices)
{
    if (!octant->isActive) {
        return;
    }

    if (3 * octant->extent * octant->extent < sqrRadius &&
        octant->inside(query, sqrRadius)) {
        // printf("contains\n");
        std::vector<const Octant *> candidate_octants;
        candidate_octants.reserve(8);
        get_leaf_nodes(octant, candidate_octants);
        for (size_t k = 0; k < candidate_octants.size(); k++) {
            const size_t size = candidate_octants[k]->points.size();
            const size_t result_size = resultIndices.size();
            resultIndices.resize(result_size + size);
            for (size_t i = 0; i < size; ++i) {
                // const float * p = ordered?
                // candidate_octants[k]->ordered_points[i] :
                // candidate_octants[k]->points[i]; const float * p =
                // candidate_octants[k]->points[i];
                resultIndices[result_size + i] =
                    candidate_octants[k]->points[i];
            }
        }
        return;
    }

    if (octant->child.empty()) {
        const size_t size = octant->points.size();
        for (size_t i = 0; i < size; ++i) {
            // const float * p = ordered? octant->ordered_points[i] :
            // octant->points[i];
            const float *p = octant->points[i];
            float dist = 0, diff = 0;
            for (size_t j = 0; j < 3; ++j) {
                diff = p[j] - query[j];
                dist += diff * diff;
            }
            if (dist < sqrRadius) {
                resultIndices.push_back(octant->points[i]);
            }
        }
        return;
    }
    for (const auto &child : octant->child) {
        if (!child) {
            continue;
        }
        if (!child->overlaps(query, sqrRadius)) {
            continue;
        }
        radiusNeighbors(child, query, radius, sqrRadius, resultIndices);
    }
}

void iOctree::radiusNeighbors(const Octant *octant, const _Point &query,
                              float radius, float sqrRadius,
                              std::vector<float *> &resultIndices,
                              std::vector<float> &distances)
{
    if (!octant->isActive) {
        return;
    }

    if (3 * octant->extent * octant->extent < sqrRadius &&
        octant->inside(query, sqrRadius)) {
        std::vector<const Octant *> candidate_octants;
        get_leaf_nodes(octant, candidate_octants);
        for (size_t k = 0; k < candidate_octants.size(); k++) {
            const size_t size = candidate_octants[k]->points.size();
            const size_t result_size = resultIndices.size();
            resultIndices.resize(result_size + size);
            for (size_t i = 0; i < size; ++i) {
                const float *p = candidate_octants[k]->points[i];
                float dist = 0, diff = 0;
                for (size_t j = 0; j < 3; ++j) {
                    diff = p[j] - query[j];
                    dist += diff * diff;
                }
                distances.push_back(dist);
                resultIndices[result_size + i] =
                    candidate_octants[k]->points[i];
            }
        }
        return;
    }

    if (octant->child.empty()) {
        const size_t size = octant->points.size();
        for (size_t i = 0; i < size; ++i) {
            const float *p = octant->points[i];
            float dist = 0, diff = 0;
            for (size_t j = 0; j < 3; ++j) {
                diff = p[j] - query[j];
                dist += diff * diff;
            }
            if (dist < sqrRadius) {
                resultIndices.push_back(octant->points[i]);
                distances.push_back(dist);
            }
        }
        return;
    }

    for (const auto &child : octant->child) {
        if (!child) {
            continue;
        }
        if (!child->overlaps(query, sqrRadius)) {
            continue;
        }
        radiusNeighbors(child, query, radius, sqrRadius, resultIndices,
                        distances);
    }
}

bool iOctree::radiusNeighbors2(const Octant *octant, const _Point &query,
                               float sqrRadius,
                               std::vector<size_t> &resultIndices,
                               std::vector<float> &distances)
{
    if (!octant->isActive) {
        return false;
    }

    if (octant->child.empty()) {
        const size_t size = octant->points.size();
        for (int i = 0; i < size; ++i) {
            // const float * p = ordered? octant->ordered_points[i] :
            // octant->points[i];
            const float *p = octant->points[i];
            float dist = 0, diff = 0;
            for (int j = 0; j < 3; ++j) {
                diff = p[j] - query[j];
                dist += diff * diff;
            }
            if (dist < sqrRadius) {
                resultIndices.push_back(size_t(p[3]));
                distances.push_back(dist);
            }
        }
        // 如果堆已经满了且最远点在当前网格内，则不必搜索了
        return octant->contains(query, sqrRadius);
    }

    size_t mortonCode = 0;
    if (query[0] > octant->x)
        mortonCode |= 1;
    if (query[1] > octant->y)
        mortonCode |= 2;
    if (query[2] > octant->z)
        mortonCode |= 4;
    if (octant->child[mortonCode] != 0) {
        if (radiusNeighbors2(octant->child[mortonCode], query, sqrRadius,
                             resultIndices, distances)) {
            return true;
        }
    }

    for (int i = 0; i < 7; ++i) {
        int c = ordered_indies[mortonCode][i];
        if (!octant->child[c]) {
            continue;
        }
        if (!octant->child[c]->overlaps(query, sqrRadius)) {
            continue;
        }
        if (radiusNeighbors2(octant->child[c], query, sqrRadius, resultIndices,
                             distances)) {
            return true;
        }
    }
    return octant->contains(query, sqrRadius);
}

bool iOctree::knnNeighbors(const Octant *octant, const _Point &query,
                           KNNSimpleResultSet &heap)
{
    // if (run_details.one_path) run_details.depth++;
    if (!octant->isActive) {
        return false;
    }

    if (octant->child.empty()) {
        const size_t size = octant->points.size();
        for (int i = 0; i < size; ++i) {
            // const float * p = ordered? octant->ordered_points[i] :
            // octant->points[i];
            const float *p = octant->points[i];
            float dist = 0, diff = 0;
            for (int j = 0; j < 3; ++j) {
                diff = p[j] - query[j];
                dist += diff * diff;
            }
            if (dist < heap.worstDist()) {
                heap.addPoint(dist, octant->points[i]);
            }
        }
        // run_details.one_path = false;
        // run_details.pts_n += size;
        // run_details.node_n++;
        // run_details.show();
        // 如果堆已经满了且最远点在当前网格内，则不必搜索了
        return heap.full() && octant->contains(query, heap.worstDist());
    }

    size_t mortonCode = 0;
    if (query[0] > octant->x)
        mortonCode |= 1;
    if (query[1] > octant->y)
        mortonCode |= 2;
    if (query[2] > octant->z)
        mortonCode |= 4;
    if (octant->child[mortonCode] != 0) {
        if (knnNeighbors(octant->child[mortonCode], query, heap)) {
            return true;
        }
    }

    for (int i = 0; i < 7; ++i) {
        int c = ordered_indies[mortonCode][i];
        if (!octant->child[c]) {
            continue;
        }
        if (heap.full() &&
            !octant->child[c]->overlaps(query, heap.worstDist())) {
            continue;
        }
        if (knnNeighbors(octant->child[c], query, heap)) {
            return true;
        }
    }
    return heap.full() && octant->contains(query, heap.worstDist());
}

void iOctree::boxWiseDelete(Octant *octant,
                            const Eigen::AlignedBox3f &box_range, bool &deleted,
                            bool clear_data)
{
    const Eigen::Vector3f center{octant->x, octant->y, octant->z};
    const auto extend = Eigen::Vector3f::Constant(octant->extent);
    const Eigen::AlignedBox3f bbox{center - extend, center + extend};

    if (bbox.intersects(box_range)) {
        return;
    }
    // printf("octant->extent: %f\n", octant->extent);
    // printf("octant->extent: %f, %d \n", octant->extent,clear_data);
    // if(!clear_data) exit(1);
    // 确定有交集
    if (box_range.contains(bbox)) {
        if (!clear_data) {
            octant->isActive = false;
            return;
        }

        pts_num_deleted += octant->size();
        delete octant;
        deleted = true;
        return;
    }

    if (octant->child.empty()) {
        // printf("octant->extent: %f, %d \n", octant->extent,clear_data);
        if (!clear_data) {
            octant->isActive = false;
            return;
        }

        // printf("octant->extent: %f\n", octant->extent);
        const size_t size = octant->points.size();
        std::vector<float *> remainder_points;
        remainder_points.resize(size, 0);
        size_t valid_num = 0;
        for (int i = 0; i < size; ++i) {
            const float *p = octant->points[i];
            if (!box_range.contains(Eigen::Vector3f{p})) {
                remainder_points[valid_num] = octant->points[i];
                valid_num++;
                continue;
            }
        }
        // printf("valid_num: %d\n", valid_num);
        pts_num_deleted += size - valid_num;
        if (valid_num == 0) {
            delete octant;
            deleted = true;
            return;
        }

        float *continue_points = new float[valid_num * dim];
        float *old_points = octant->points[0];
        for (size_t i = 0; i < valid_num; ++i) {
            std::copy(remainder_points[i], remainder_points[i] + dim,
                      continue_points + dim * i);
            octant->points[i] = continue_points + dim * i;
        }
        octant->points.resize(valid_num);
        delete[] old_points;
        // printf("delete: \n");
        return;
    }

    // check whether child nodes are in range.
    for (auto &child : octant->child) {
        if (!child) {
            continue;
        }

        bool deleted1 = false;
        boxWiseDelete(child, box_range, deleted1, clear_data);
        if (deleted1) {
            child = nullptr;
        }
    }

    int valid_child =
        std::ranges::count_if(octant->child, [](auto child) { return child; });

    if (valid_child == 0) {
        delete octant;
        deleted = true;
    }
}

} // namespace tl
