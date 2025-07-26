#pragma once

#include <vector>
#include <mutex>
#include <thread>

#include <Eigen/Geometry>

#define Q_LEN 1000000

enum delete_point_storage_set
{
    NOT_RECORD,
    DELETE_POINTS_REC,
    MULTI_THREAD_REC
};

template <typename T>
class MANUAL_Q
{
public:
    void push(T op);
    void pop();
    void clear();
    T front() const;
    T back() const;
    bool empty() const;
    int size() const;

private:
    int head = 0, tail = 0, counter = 0;
    T q[Q_LEN];
    bool is_empty;
};

using PointType = Eigen::Vector3f;
using _PointCloud = std::vector<PointType>;

class iKdTree
{
public:
    struct Node
    {
        PointType point;
        uint8_t division_axis;
        int tree_size = 1;
        int invalid_point_num = 0;
        int down_del_num = 0;
        bool point_deleted = false;
        bool tree_deleted = false;
        bool point_downsample_deleted = false;
        bool tree_downsample_deleted = false;
        bool need_push_down_to_left = false;
        bool need_push_down_to_right = false;
        bool working_flag = false;
        float radius_sq;
        std::mutex push_down_mutex_lock;
        Eigen::AlignedBox3f range;
        Node *left = nullptr;
        Node *right = nullptr;
        Node *parent = nullptr;

        // For paper data record
        float alpha_del;
        float alpha_bal;

        void Init();

        float CalcBoxDistance(const PointType &point) const;

        void Update();
    };

    struct Operation
    {
        enum Type
        {
            kAddPoint,
            kDeletePoint,
            kAddBox,
            kDeleteBox,
            kDownsampleDelete,
            kPushDown
        };

        PointType point;
        Eigen::AlignedBox3f box;
        Type type;
        bool tree_deleted;
        bool tree_downsample_deleted;
    };

    struct PointType_CMP
    {
        PointType point;
        float dist = 0.f;

        explicit PointType_CMP(PointType p = PointType(), float d = INFINITY)
            : point(p), dist(d)
        {
        }

        bool operator<(const PointType_CMP &a) const
        {
            if (std::abs(dist - a.dist) < 1e-10) {
                return point.x() < a.point.x();
            }

            return dist < a.dist;
        }

        bool operator>=(const PointType_CMP &a) const { return !(*this < a); }
    };

    class MANUAL_HEAP
    {
    public:
        MANUAL_HEAP(int max_capacity = 100)
        {
            cap = max_capacity;
            heap = new PointType_CMP[max_capacity];
            heap_size = 0;
        }

        ~MANUAL_HEAP() { delete[] heap; }

        void push(PointType_CMP point)
        {
            if (heap_size >= cap) {
                return;
            }

            heap[heap_size] = point;
            FloatUp(heap_size);
            heap_size++;
        }

        void pop()
        {
            if (heap_size == 0) {
                return;
            }

            heap[0] = heap[heap_size - 1];
            heap_size--;
            MoveDown(0);
        }

        PointType_CMP top() const { return heap[0]; }

        int size() const { return heap_size; }

        void clear() { heap_size = 0; }

    private:
        void MoveDown(int heap_index)
        {
            int l = heap_index * 2 + 1;
            PointType_CMP tmp = heap[heap_index];
            while (l < heap_size) {
                if (l + 1 < heap_size && heap[l] < heap[l + 1]) {
                    l++;
                }

                if (tmp >= heap[l]) {
                    break;
                }

                heap[heap_index] = heap[l];
                heap_index = l;
                l = heap_index * 2 + 1;
            }
            heap[heap_index] = tmp;
        }

        void FloatUp(int heap_index)
        {
            int ancestor = (heap_index - 1) / 2;
            PointType_CMP tmp = heap[heap_index];
            while (heap_index > 0) {
                if (heap[ancestor] >= tmp) {
                    break;
                }

                heap[heap_index] = heap[ancestor];
                heap_index = ancestor;
                ancestor = (heap_index - 1) / 2;
            }
            heap[heap_index] = tmp;
        }

    private:
        int heap_size = 0;
        int cap = 0;
        PointType_CMP *heap;
    };

public:
    using Ptr = std::shared_ptr<iKdTree>;

    iKdTree(float delete_param = 0.5, float balance_param = 0.6,
            float box_length = 0.2);
    ~iKdTree();

    // Properties
    void SetDeleteCriterion(float delete_param);
    void SetBalanceCriterion(float balance_param);
    void SetDownsampleBoxLength(float box_length);
    void InitializeKDTree(float delete_param = 0.5, float balance_param = 0.7,
                          float box_length = 0.2);

    int size() const;
    int validnum() const;
    void root_alpha(float &alpha_bal, float &alpha_del);
    Eigen::AlignedBox3f tree_range() const;

    void Build(_PointCloud &point_cloud);
    void NearestSearch(const PointType &point, int kth,
                       _PointCloud &nearest_points,
                       std::vector<float> &nearest_distances,
                       double max_dist = INFINITY);
    void BoxSearch(const Eigen::AlignedBox3f &box, _PointCloud &points);
    void RadiusSearch(const PointType &point, float radius,
                      _PointCloud &points);

    int AddPoints(const _PointCloud &points, bool downsample);
    void DeletePoints(const _PointCloud &points);
    void AddBoxes(const std::vector<Eigen::AlignedBox3f> &boxes);
    int DeleteBoxes(const std::vector<Eigen::AlignedBox3f> &boxes);

    void flatten(Node *root, delete_point_storage_set storage_type,
                 _PointCloud &Storage);
    // NOTE: Not const because this function will clear cache points
    _PointCloud acquire_removed_points();

public:
    _PointCloud PCL_Storage;
    Node *root_ = nullptr;
    int max_queue_size = 0;

private:
    static void *multi_thread_ptr(void *arg);
    void multi_thread_rebuild();
    void start_thread();
    void stop_thread();

    void run_operation(Node **root, const Operation &operation);

    void BuildTree(Node **root, int l, int r, _PointCloud &out_points);
    void Rebuild(Node **root);

    int DeleteByRange(Node **root, const Eigen::AlignedBox3f &range,
                      bool rebuild, bool downsample);
    void DeleteByPoint(Node **root, const PointType &point, bool rebuild);
    void AddByPoint(Node **root, const PointType &point, bool rebuild,
                    int father_axis);
    void AddByRange(Node **root, const Eigen::AlignedBox3f &range,
                    bool rebuild);
    void Search(Node *root, int kth, const PointType &point, MANUAL_HEAP &q,
                double max_dist);
    void SearchByRange(Node *root, const Eigen::AlignedBox3f &range,
                       _PointCloud &out_points);
    void SearchByRadius(Node *root, const PointType &point, float radius,
                        _PointCloud &out_points);

    bool Criterion_Check(Node *root) const;
    void Push_Down(Node *root);
    void Update(Node *root);
    void delete_tree_nodes(Node **root);
    void downsample(Node **root);

private:
    // Multi-thread Tree Rebuild
    // TODO: Maybe use atomic
    std::atomic<bool> termination_flag = false;
    bool rebuild_flag = false;
    std::thread rebuild_thread;
    std::mutex rebuild_ptr_mutex_lock, search_flag_mutex;
    mutable std::mutex working_flag_mutex;
    std::mutex rebuild_logger_mutex_lock, points_deleted_rebuild_mutex_lock;
    // std::queue<Operation> Rebuild_Logger;
    MANUAL_Q<Operation> Rebuild_Logger;
    _PointCloud Rebuild_PCL_Storage;
    Node **Rebuild_Ptr = nullptr;
    int search_mutex_counter = 0;

    // KD Tree Functions and augmented variables
    int Treesize_tmp = 0, Validnum_tmp = 0;
    float alpha_bal_tmp = 0.5, alpha_del_tmp = 0.0;
    float delete_criterion_param = 0.5f;
    float balance_criterion_param = 0.7f;
    float downsample_size = 0.2f;
    bool Delete_Storage_Disabled = false;
    Node *STATIC_ROOT_NODE = nullptr;
    _PointCloud Points_deleted;
    _PointCloud Downsample_Storage;
    _PointCloud Multithread_Points_deleted;
};
