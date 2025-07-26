#include "ikdtree.h"

#include <chrono>

#include <pcl/point_types.h>

using namespace std::chrono_literals;

#define EPSS 1e-6
#define Minimal_Unbalanced_Tree_Size 10
#define Multi_Thread_Rebuild_Point_Num 1500
#define DOWNSAMPLE_SWITCH true
#define ForceRebuildPercentage 0.2

iKdTree::iKdTree(float delete_param, float balance_param, float box_length)
{
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;

    start_thread();
}

iKdTree::~iKdTree()
{
    stop_thread();
    delete_tree_nodes(&root_);
    // TODO: Do we need to clear this in dtor?
    _PointCloud().swap(PCL_Storage);
}

void iKdTree::SetDeleteCriterion(float delete_param)
{
    delete_criterion_param = delete_param;
}

void iKdTree::SetBalanceCriterion(float balance_param)
{
    balance_criterion_param = balance_param;
}

void iKdTree::SetDownsampleBoxLength(float downsample_param)
{
    downsample_size = downsample_param;
}

void iKdTree::InitializeKDTree(float delete_param, float balance_param,
                               float box_length)
{
    SetDeleteCriterion(delete_param);
    SetBalanceCriterion(balance_param);
    SetDownsampleBoxLength(box_length);
}

void iKdTree::Node::Init()
{
    point.x() = 0.0f;
    point.y() = 0.0f;
    point.z() = 0.0f;
    range = {};
    division_axis = 0;
    parent = nullptr;
    left = nullptr;
    right = nullptr;
    tree_size = 0;
    invalid_point_num = 0;
    down_del_num = 0;
    point_deleted = false;
    tree_deleted = false;
    need_push_down_to_left = false;
    need_push_down_to_right = false;
    point_downsample_deleted = false;
    working_flag = false;
}

float iKdTree::Node::CalcBoxDistance(const PointType &point) const
{
    return range.squaredExteriorDistance(point);
}

void iKdTree::Node::Update()
{
    // Update Tree Size
    auto _UpdateFromChild = [this](Node *node) -> Eigen::AlignedBox3f {
        tree_size = node->tree_size + 1;
        invalid_point_num = node->invalid_point_num + point_deleted;
        down_del_num = node->down_del_num + point_downsample_deleted;
        tree_downsample_deleted =
            node->tree_downsample_deleted & point_downsample_deleted;
        tree_deleted = node->tree_deleted && point_deleted;

        Eigen::AlignedBox3f new_range;
        if (tree_deleted || (!node->tree_deleted && !point_deleted)) {
            auto node_range = node->range;
            new_range = node_range.extend(point);
        }
        else {
            if (!node->tree_deleted) {
                new_range.extend(node->range);
            }
            if (!point_deleted) {
                new_range.extend(point);
            }
        }
        return new_range;
    };

    Eigen::AlignedBox3f new_range;
    if (left && right) {
        tree_size = left->tree_size + right->tree_size + 1;
        invalid_point_num =
            left->invalid_point_num + right->invalid_point_num + point_deleted;
        down_del_num = left->down_del_num + right->down_del_num +
                       (point_downsample_deleted ? 1 : 0);
        tree_downsample_deleted = left->tree_downsample_deleted &
                                  right->tree_downsample_deleted &
                                  point_downsample_deleted;
        tree_deleted =
            left->tree_deleted && right->tree_deleted && point_deleted;
        if (tree_deleted ||
            (!left->tree_deleted && !right->tree_deleted && !point_deleted)) {
            new_range = left->range.merged(right->range).extend(point);
        }
        else {
            if (!left->tree_deleted) {
                new_range.extend(left->range);
            }
            if (!right->tree_deleted) {
                new_range.extend(right->range);
            }
            if (!point_deleted) {
                new_range.extend(point);
            }
        }
    }
    else if (left) {
        new_range = _UpdateFromChild(left);
    }
    else if (right) {
        new_range = _UpdateFromChild(right);
    }
    else {
        tree_size = 1;
        invalid_point_num = point_deleted;
        down_del_num = point_downsample_deleted;
        tree_downsample_deleted = point_downsample_deleted;
        tree_deleted = point_deleted;
        new_range = {point, point};
    }
    range = new_range;

    radius_sq = (range.diagonal() * 0.5).squaredNorm();

    if (left) {
        left->parent = this;
    }
    if (right) {
        right->parent = this;
    }
}

int iKdTree::size() const
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
        if (root_) {
            return root_->tree_size;
        }

        return 0;
    }

    if (working_flag_mutex.try_lock()) {
        int s = root_->tree_size;
        working_flag_mutex.unlock();
        return s;
    }

    return Treesize_tmp;
}

int iKdTree::validnum() const
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
        if (!root_) {
            return 0;
        }

        return root_->tree_size - root_->invalid_point_num;
    }

    if (working_flag_mutex.try_lock()) {
        int s = root_->tree_size - root_->invalid_point_num;
        working_flag_mutex.unlock();
        return s;
    }

    return -1;
}

Eigen::AlignedBox3f iKdTree::tree_range() const
{
    Eigen::AlignedBox3f range;
    if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
        if (root_) {
            range = root_->range;
        }
    }
    else {
        if (working_flag_mutex.try_lock()) {
            range = root_->range;
            working_flag_mutex.unlock();
        }
    }
    return range;
}

void iKdTree::root_alpha(float &alpha_bal, float &alpha_del)
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
        alpha_bal = root_->alpha_bal;
        alpha_del = root_->alpha_del;
        return;
    }

    if (working_flag_mutex.try_lock()) {
        alpha_bal = root_->alpha_bal;
        alpha_del = root_->alpha_del;
        working_flag_mutex.unlock();
        return;
    }

    alpha_bal = alpha_bal_tmp;
    alpha_del = alpha_del_tmp;
}

void iKdTree::start_thread()
{
    rebuild_thread = std::thread{iKdTree::multi_thread_ptr, this};
    printf("Multi thread started \n");
}

void iKdTree::stop_thread()
{
    termination_flag = true;

    if (rebuild_thread.joinable()) {
        rebuild_thread.join();
    }
}

void *iKdTree::multi_thread_ptr(void *arg)
{
    iKdTree *handle = (iKdTree *)arg;
    handle->multi_thread_rebuild();
    return nullptr;
}

void iKdTree::multi_thread_rebuild()
{
    Node *parent;
    Node **new_node_ptr4;
    auto terminated = termination_flag.load();

    while (!terminated) {
        if (Rebuild_Ptr) {
            std::lock_guard rebuild_lock{rebuild_ptr_mutex_lock};
            std::lock_guard working_lock{working_flag_mutex};
            /* Traverse and copy */
            if (!Rebuild_Logger.empty()) {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            if (*Rebuild_Ptr == root_) {
                Treesize_tmp = root_->tree_size;
                Validnum_tmp = root_->tree_size - root_->invalid_point_num;
                alpha_bal_tmp = root_->alpha_bal;
                alpha_del_tmp = root_->alpha_del;
            }
            Node *old_root_node = (*Rebuild_Ptr);
            parent = (*Rebuild_Ptr)->parent;
            _PointCloud().swap(Rebuild_PCL_Storage);
            // Lock Search
            {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter != 0) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter = -1;
            }

            {
                std::lock_guard delete_lock{points_deleted_rebuild_mutex_lock};
                flatten(*Rebuild_Ptr, MULTI_THREAD_REC, Rebuild_PCL_Storage);
            }

            {
                std::lock_guard search_lock{search_flag_mutex};
                search_mutex_counter = 0;
            }

            working_flag_mutex.unlock();
            /* Rebuild and update missed operations*/

            Node *new_root_node = nullptr;
            if (!Rebuild_PCL_Storage.empty()) {
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1,
                          Rebuild_PCL_Storage);
                // Rebuild has been done. Updates the blocked operations into
                // the new tree
                working_flag_mutex.lock();
                rebuild_logger_mutex_lock.lock();
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty()) {
                    const auto operation = Rebuild_Logger.front();
                    max_queue_size =
                        std::max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    rebuild_logger_mutex_lock.unlock();
                    working_flag_mutex.unlock();
                    run_operation(&new_root_node, operation);
                    tmp_counter++;
                    if (tmp_counter % 10 == 0) {
                        std::this_thread::sleep_for(1us);
                    }
                    working_flag_mutex.lock();
                    rebuild_logger_mutex_lock.lock();
                }
                rebuild_logger_mutex_lock.unlock();
            }
            /* Replace to original tree*/
            {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter != 0) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter = -1;
            }

            if (parent->left == *Rebuild_Ptr) {
                parent->left = new_root_node;
            }
            else if (parent->right == *Rebuild_Ptr) {
                parent->right = new_root_node;
            }
            else {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node) {
                new_root_node->parent = parent;
            }

            (*Rebuild_Ptr) = new_root_node;
            int valid_old =
                old_root_node->tree_size - old_root_node->invalid_point_num;
            int valid_new =
                new_root_node->tree_size - new_root_node->invalid_point_num;
            if (parent == STATIC_ROOT_NODE) {
                root_ = STATIC_ROOT_NODE->left;
            }
            Node *update_root = *Rebuild_Ptr;
            while (update_root && update_root != root_) {
                update_root = update_root->parent;
                if (update_root->working_flag)
                    break;
                if (update_root == update_root->parent->left &&
                    update_root->parent->need_push_down_to_left)
                    break;
                if (update_root == update_root->parent->right &&
                    update_root->parent->need_push_down_to_right)
                    break;
                Update(update_root);
            }

            {
                std::lock_guard search_lock{search_flag_mutex};
                search_mutex_counter = 0;
            }

            Rebuild_Ptr = nullptr;
            working_flag_mutex.unlock();
            rebuild_flag = false;
            /* Delete discarded tree nodes */
            delete_tree_nodes(&old_root_node);
        }

        terminated = termination_flag.load();

        std::this_thread::sleep_for(100us);
    }
    printf("Rebuild thread terminated normally\n");
}

void iKdTree::run_operation(Node **root, const Operation &operation)
{
    using Type = Operation::Type;
    switch (operation.type) {
        case Type::kAddPoint:
            AddByPoint(root, operation.point, false, (*root)->division_axis);
            break;
        case Type::kDeletePoint:
            DeleteByPoint(root, operation.point, false);
            break;
        case Type::kAddBox:
            AddByRange(root, operation.box, false);
            break;
        case Type::kDeleteBox:
            DeleteByRange(root, operation.box, false, false);
            break;
        case Type::kDownsampleDelete:
            DeleteByRange(root, operation.box, false, true);
            break;
        case Type::kPushDown:
            (*root)->tree_downsample_deleted |=
                operation.tree_downsample_deleted;
            (*root)->point_downsample_deleted |=
                operation.tree_downsample_deleted;
            (*root)->tree_deleted =
                operation.tree_deleted || (*root)->tree_downsample_deleted;
            (*root)->point_deleted =
                (*root)->tree_deleted || (*root)->point_downsample_deleted;
            if (operation.tree_downsample_deleted) {
                (*root)->down_del_num = (*root)->tree_size;
            }

            (*root)->invalid_point_num = operation.tree_deleted
                                             ? (*root)->tree_size
                                             : (*root)->down_del_num;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            break;
        default:
            break;
    }
}

void iKdTree::Build(_PointCloud &point_cloud)
{
    if (point_cloud.empty()) {
        return;
    }

    if (root_) {
        delete_tree_nodes(&root_);
    }

    STATIC_ROOT_NODE = new Node;
    STATIC_ROOT_NODE->Init();
    BuildTree(&STATIC_ROOT_NODE->left, 0, point_cloud.size() - 1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->tree_size = 0;
    root_ = STATIC_ROOT_NODE->left;
}

void iKdTree::NearestSearch(const PointType &point, int kth,
                            _PointCloud &nearest_points,
                            std::vector<float> &nearest_distances,
                            double max_dist)
{
    MANUAL_HEAP q(2 * kth);
    if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
        Search(root_, kth, point, q, max_dist);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        while (search_mutex_counter == -1) {
            search_flag_mutex.unlock();
            std::this_thread::sleep_for(1us);
            search_flag_mutex.lock();
        }
        search_mutex_counter += 1;
        search_flag_mutex.unlock();
        Search(root_, kth, point, q, max_dist);
        search_flag_mutex.lock();
        search_mutex_counter -= 1;
    }

    const int k_found = std::min(kth, int(q.size()));
    for (int i = 0; i < k_found; i++) {
        nearest_points.insert(nearest_points.begin(), q.top().point);
        nearest_distances.insert(nearest_distances.begin(), q.top().dist);
        q.pop();
    }
}

void iKdTree::BoxSearch(const Eigen::AlignedBox3f &box, _PointCloud &points)
{
    SearchByRange(root_, box, points);
}

void iKdTree::RadiusSearch(const PointType &point, float radius,
                           _PointCloud &points)
{
    SearchByRadius(root_, point, radius, points);
}

int iKdTree::AddPoints(const _PointCloud &points, bool downsample)
{
    const bool downsample_switch = downsample && DOWNSAMPLE_SWITCH;

    int counter = 0;
    for (const auto &point : points) {
        if (downsample_switch) {
            Eigen::AlignedBox3f bbox;
            bbox.min()[0] =
                std::floor(point.x() / downsample_size) * downsample_size;
            bbox.max()[0] = bbox.min()[0] + downsample_size;
            bbox.min()[1] =
                std::floor(point.y() / downsample_size) * downsample_size;
            bbox.max()[1] = bbox.min()[1] + downsample_size;
            bbox.min()[2] =
                std::floor(point.z() / downsample_size) * downsample_size;
            bbox.max()[2] = bbox.min()[2] + downsample_size;

            const PointType mid_point = bbox.center();

            _PointCloud().swap(Downsample_Storage);
            SearchByRange(root_, bbox, Downsample_Storage);
            auto min_dist = (point - mid_point).squaredNorm();
            auto downsample_result = point;
            for (const auto &pt : Downsample_Storage) {
                if (const auto tmp_dist = (pt - mid_point).squaredNorm();
                    tmp_dist < min_dist) {
                    min_dist = tmp_dist;
                    downsample_result = pt;
                }
            }

            if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
                if (Downsample_Storage.size() > 1 ||
                    point.isApprox(downsample_result)) {
                    if (!Downsample_Storage.empty()) {
                        DeleteByRange(&root_, bbox, true, true);
                    }

                    AddByPoint(&root_, downsample_result, true,
                               root_->division_axis);
                    counter++;
                }
            }
            else {
                if (Downsample_Storage.size() > 1 ||
                    point.isApprox(downsample_result)) {
                    std::lock_guard working_lock{working_flag_mutex};
                    if (!Downsample_Storage.empty()) {
                        DeleteByRange(&root_, bbox, false, true);
                    }
                    AddByPoint(&root_, downsample_result, false,
                               root_->division_axis);
                    counter++;
                    if (rebuild_flag) {
                        std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                        if (!Downsample_Storage.empty()) {
                            Operation op;
                            op.type = Operation::Type::kDownsampleDelete;
                            op.box = bbox;
                            Rebuild_Logger.push(op);
                        }
                        else {
                            Operation op;
                            op.type = Operation::Type::kAddPoint;
                            op.point = downsample_result;
                            Rebuild_Logger.push(op);
                        }
                    }
                };
            }
        }
        else {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
                AddByPoint(&root_, point, true, root_->division_axis);
            }
            else {
                std::lock_guard working_lock{working_flag_mutex};
                AddByPoint(&root_, point, false, root_->division_axis);
                if (rebuild_flag) {
                    Operation op;
                    op.type = Operation::Type::kAddPoint;
                    op.point = point;
                    std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                    Rebuild_Logger.push(op);
                }
            }
        }
    }
    return counter;
}

void iKdTree::AddBoxes(const std::vector<Eigen::AlignedBox3f> &boxes)
{
    for (const auto &box : boxes) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
            AddByRange(&root_, box, true);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            AddByRange(&root_, box, false);
            if (rebuild_flag) {
                Operation op;
                op.type = Operation::Type::kAddPoint;
                op.box = box;
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(op);
            }
        }
    }
}

void iKdTree::DeletePoints(const _PointCloud &points)
{
    for (const auto &point : points) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
            DeleteByPoint(&root_, point, true);
        }
        else {
            std::scoped_lock working_lock{working_flag_mutex};
            DeleteByPoint(&root_, point, false);
            if (rebuild_flag) {
                Operation op;
                op.type = Operation::Type::kDeletePoint;
                op.point = point;
                std::lock_guard lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(op);
            }
        }
    }
}

int iKdTree::DeleteBoxes(const std::vector<Eigen::AlignedBox3f> &boxes)
{
    int counter = 0;
    for (const auto &box : boxes) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root_) {
            counter += DeleteByRange(&root_, box, true, false);
        }
        else {
            std::scoped_lock working_lock{working_flag_mutex};
            counter += DeleteByRange(&root_, box, false, false);
            if (rebuild_flag) {
                Operation op;
                op.type = Operation::Type::kDeleteBox;
                op.box = box;
                std::lock_guard lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(op);
            }
        }
    }
    return counter;
}

_PointCloud iKdTree::acquire_removed_points()
{
    _PointCloud removed_points;

    std::scoped_lock locker{points_deleted_rebuild_mutex_lock};
    for (const auto &point : Points_deleted) {
        removed_points.push_back(point);
    }
    Points_deleted.clear();
    for (const auto &point : Multithread_Points_deleted) {
        removed_points.push_back(point);
    }
    Multithread_Points_deleted.clear();

    return removed_points;
}

void iKdTree::BuildTree(Node **root, int l, int r, _PointCloud &Storage)
{
    if (l > r) {
        return;
    }

    *root = new Node;
    (*root)->Init();
    int mid = (l + r) >> 1;
    int div_axis = 0;
    int i;
    // Find the best division Axis
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    for (i = l; i <= r; i++) {
        min_value[0] = std::min(min_value[0], Storage[i].x());
        min_value[1] = std::min(min_value[1], Storage[i].y());
        min_value[2] = std::min(min_value[2], Storage[i].z());
        max_value[0] = std::max(max_value[0], Storage[i].x());
        max_value[1] = std::max(max_value[1], Storage[i].y());
        max_value[2] = std::max(max_value[2], Storage[i].z());
    }

    // Select the longest dimension as division axis
    for (i = 0; i < 3; i++) {
        dim_range[i] = max_value[i] - min_value[i];
    }
    for (i = 1; i < 3; i++) {
        if (dim_range[i] > dim_range[div_axis]) {
            div_axis = i;
        }
    }

    // Divide by the division axis and recursively build.
    (*root)->division_axis = div_axis;
    switch (div_axis) {
        case 0:
        default:
            std::nth_element(
                Storage.begin() + l, Storage.begin() + mid,
                Storage.begin() + r + 1,
                [](const auto &a, const auto &b) { return a.x() < b.x(); });
            break;
        case 1:
            std::nth_element(
                Storage.begin() + l, Storage.begin() + mid,
                Storage.begin() + r + 1,
                [](const auto &a, const auto &b) { return a.y() < b.y(); });
            break;
        case 2:
            std::nth_element(
                Storage.begin() + l, Storage.begin() + mid,
                Storage.begin() + r + 1,
                [](const auto &a, const auto &b) { return a.z() < b.z(); });
            break;
    }
    (*root)->point = Storage[mid];
    Node *left_son = nullptr, *right_son = nullptr;
    BuildTree(&left_son, l, mid - 1, Storage);
    BuildTree(&right_son, mid + 1, r, Storage);
    (*root)->left = left_son;
    (*root)->right = right_son;
    Update((*root));
}

void iKdTree::Rebuild(Node **root)
{
    Node *parent;
    if ((*root)->tree_size >= Multi_Thread_Rebuild_Point_Num) {
        if (rebuild_ptr_mutex_lock.try_lock()) {
            if (!Rebuild_Ptr ||
                ((*root)->tree_size > (*Rebuild_Ptr)->tree_size)) {
                Rebuild_Ptr = root;
            }
            rebuild_ptr_mutex_lock.unlock();
        }
    }
    else {
        parent = (*root)->parent;
        int size_rec = (*root)->tree_size;
        PCL_Storage.clear();
        flatten(*root, DELETE_POINTS_REC, PCL_Storage);
        delete_tree_nodes(root);
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
        if (*root) {
            (*root)->parent = parent;
        }
        if (*root == root_) {
            STATIC_ROOT_NODE->left = *root;
        }
    }
}

int iKdTree::DeleteByRange(Node **root, const Eigen::AlignedBox3f &range,
                           bool rebuild, bool downsample)
{
    if (!*root || (*root)->tree_deleted) {
        return 0;
    }

    (*root)->working_flag = true;
    Push_Down(*root);

    if (!range.intersects((*root)->range)) {
        return 0;
    }

    if (range.contains((*root)->range)) {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        const auto counter = (*root)->tree_size - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->tree_size;
        if (downsample) {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->tree_size;
        }
        return counter;
    }

    int counter = 0;
    if (!(*root)->point_deleted && range.contains((*root)->point)) {
        (*root)->point_deleted = true;
        counter += 1;
        if (downsample) {
            (*root)->point_downsample_deleted = true;
        }
    }

    Operation operation;
    operation.type = downsample ? Operation::Type::kDownsampleDelete
                                : Operation::Type::kDeleteBox;
    operation.box = range;

    if (!Rebuild_Ptr || (*root)->left != *Rebuild_Ptr) {
        counter += DeleteByRange(&((*root)->left), range, rebuild, downsample);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        counter += DeleteByRange(&((*root)->left), range, false, downsample);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(operation);
        }
    }

    if (!Rebuild_Ptr || (*root)->right != *Rebuild_Ptr) {
        counter += DeleteByRange(&((*root)->right), range, rebuild, downsample);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        counter += DeleteByRange(&((*root)->right), range, false, downsample);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(operation);
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->tree_size < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (!*root) {
        (*root)->working_flag = false;
    }

    return counter;
}

void iKdTree::DeleteByPoint(Node **root, const PointType &point,
                            bool allow_rebuild)
{
    if (!*root || (*root)->tree_deleted) {
        return;
    }

    (*root)->working_flag = true;
    Push_Down(*root);
    if ((*root)->point.isApprox(point) && !(*root)->point_deleted) {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->tree_size) {
            (*root)->tree_deleted = true;
        }
        return;
    }

    Operation operation;
    operation.type = Operation::Type::kDeletePoint;
    operation.point = point;
    if (((*root)->division_axis == 0 && point.x() < (*root)->point.x()) ||
        ((*root)->division_axis == 1 && point.y() < (*root)->point.y()) ||
        ((*root)->division_axis == 2 && point.z() < (*root)->point.z())) {
        if (!Rebuild_Ptr || (*root)->left != *Rebuild_Ptr) {
            DeleteByPoint(&(*root)->left, point, allow_rebuild);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            DeleteByPoint(&(*root)->left, point, false);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
        }
    }
    else {
        if (!Rebuild_Ptr || (*root)->right != *Rebuild_Ptr) {
            DeleteByPoint(&(*root)->right, point, allow_rebuild);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            DeleteByPoint(&(*root)->right, point, false);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
        }
    }
    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->tree_size < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

void iKdTree::AddByRange(Node **root, const Eigen::AlignedBox3f &range,
                         bool allow_rebuild)
{
    if (!*root) {
        return;
    }

    (*root)->working_flag = true;
    Push_Down(*root);
    if (!range.intersects((*root)->range)) {
        return;
    }

    if (range.contains((*root)->range)) {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }

    if (range.contains((*root)->point)) {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }

    Operation operation;
    operation.type = Operation::Type::kAddBox;
    operation.box = range;
    if (!Rebuild_Ptr || (*root)->left != *Rebuild_Ptr) {
        AddByRange(&((*root)->left), range, allow_rebuild);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        AddByRange(&((*root)->left), range, false);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(operation);
        }
    }

    if (!Rebuild_Ptr || (*root)->right != *Rebuild_Ptr) {
        AddByRange(&((*root)->right), range, allow_rebuild);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        AddByRange(&((*root)->right), range, false);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(operation);
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->tree_size < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

void iKdTree::AddByPoint(Node **root, const PointType &point,
                         bool allow_rebuild, int father_axis)
{
    if (!*root) {
        *root = new Node;
        (*root)->Init();
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }

    (*root)->working_flag = true;
    Operation operation;
    operation.type = Operation::Type::kAddPoint;
    operation.point = point;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x() < (*root)->point.x()) ||
        ((*root)->division_axis == 1 && point.y() < (*root)->point.y()) ||
        ((*root)->division_axis == 2 && point.z() < (*root)->point.z())) {
        if (!Rebuild_Ptr || (*root)->left != *Rebuild_Ptr) {
            AddByPoint(&(*root)->left, point, allow_rebuild,
                       (*root)->division_axis);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            AddByPoint(&(*root)->left, point, false, (*root)->division_axis);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
        }
    }
    else {
        if (!Rebuild_Ptr || (*root)->right != *Rebuild_Ptr) {
            AddByPoint(&(*root)->right, point, allow_rebuild,
                       (*root)->division_axis);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            AddByPoint(&(*root)->right, point, false, (*root)->division_axis);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->tree_size < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

void iKdTree::Search(Node *root, int k_nearest, const PointType &point,
                     MANUAL_HEAP &q, double max_dist)
{
    if (!root || root->tree_deleted) {
        return;
    }

    double cur_dist = root->CalcBoxDistance(point);
    double max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr) {
        return;
    }

    if (root->need_push_down_to_left || root->need_push_down_to_right) {
        if (root->push_down_mutex_lock.try_lock()) {
            Push_Down(root);
            root->push_down_mutex_lock.unlock();
        }
        else {
            root->push_down_mutex_lock.lock();
            root->push_down_mutex_lock.unlock();
        }
    }

    if (!root->point_deleted) {
        float dist = (point - root->point).squaredNorm();
        if (dist <= max_dist_sqr &&
            (q.size() < k_nearest || dist < q.top().dist)) {
            if (q.size() >= k_nearest) {
                q.pop();
            }

            PointType_CMP current_point{root->point, dist};
            q.push(current_point);
        }
    }

    int cur_search_counter;
    float dist_left_node = root->left->CalcBoxDistance(point);
    float dist_right_node = root->right->CalcBoxDistance(point);
    if (q.size() < k_nearest ||
        dist_left_node < q.top().dist && dist_right_node < q.top().dist) {
        if (dist_left_node <= dist_right_node) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->left) {
                Search(root->left, k_nearest, point, q, max_dist);
            }
            else {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter == -1) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter += 1;
                search_flag_mutex.unlock();
                Search(root->left, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (!Rebuild_Ptr || *Rebuild_Ptr != root->right) {
                    Search(root->right, k_nearest, point, q, max_dist);
                }
                else {
                    std::lock_guard search_lock{search_flag_mutex};
                    while (search_mutex_counter == -1) {
                        search_flag_mutex.unlock();
                        std::this_thread::sleep_for(1us);
                        search_flag_mutex.lock();
                    }
                    search_mutex_counter += 1;
                    search_flag_mutex.unlock();
                    Search(root->right, k_nearest, point, q, max_dist);
                    search_flag_mutex.lock();
                    search_mutex_counter -= 1;
                }
            }
        }
        else {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->right) {
                Search(root->right, k_nearest, point, q, max_dist);
            }
            else {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter == -1) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter += 1;
                search_flag_mutex.unlock();
                Search(root->right, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {
                if (!Rebuild_Ptr || *Rebuild_Ptr != root->left) {
                    Search(root->left, k_nearest, point, q, max_dist);
                }
                else {
                    std::lock_guard search_lock{search_flag_mutex};
                    while (search_mutex_counter == -1) {
                        search_flag_mutex.unlock();
                        std::this_thread::sleep_for(1us);
                        search_flag_mutex.lock();
                    }
                    search_mutex_counter += 1;
                    search_flag_mutex.unlock();
                    Search(root->left, k_nearest, point, q, max_dist);
                    search_flag_mutex.lock();
                    search_mutex_counter -= 1;
                }
            }
        }
    }
    else {
        if (dist_left_node < q.top().dist) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->left) {
                Search(root->left, k_nearest, point, q, max_dist);
            }
            else {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter == -1) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter += 1;
                search_flag_mutex.unlock();
                Search(root->left, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
        }
        if (dist_right_node < q.top().dist) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->right) {
                Search(root->right, k_nearest, point, q, max_dist);
            }
            else {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter == -1) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter += 1;
                search_flag_mutex.unlock();
                Search(root->right, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
        }
    }
}

void iKdTree::SearchByRange(Node *root, const Eigen::AlignedBox3f &range,
                            _PointCloud &Storage)
{
    if (!root) {
        return;
    }

    Push_Down(root);

    if (!range.intersects(root->range)) {
        return;
    }

    if (range.contains(root->range)) {
        flatten(root, NOT_RECORD, Storage);
        return;
    }

    if (range.contains(root->point)) {
        if (!root->point_deleted) {
            Storage.push_back(root->point);
        }
    }

    if (!Rebuild_Ptr || root->left != *Rebuild_Ptr) {
        SearchByRange(root->left, range, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        SearchByRange(root->left, range, Storage);
    }

    if (!Rebuild_Ptr || root->right != *Rebuild_Ptr) {
        SearchByRange(root->right, range, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        SearchByRange(root->right, range, Storage);
    }
}

void iKdTree::SearchByRadius(Node *root, const PointType &point, float radius,
                             _PointCloud &Storage)
{
    if (!root) {
        return;
    }

    Push_Down(root);
    const PointType range_center = root->range.center();

    float dist = (range_center - point).norm();
    if (dist > radius + std::sqrt(root->radius_sq)) {
        return;
    }
    if (dist <= radius - std::sqrt(root->radius_sq)) {
        flatten(root, NOT_RECORD, Storage);
        return;
    }

    if (!root->point_deleted &&
        (root->point - point).squaredNorm() <= radius * radius) {
        Storage.push_back(root->point);
    }

    if (!Rebuild_Ptr || root->left != *Rebuild_Ptr) {
        SearchByRadius(root->left, point, radius, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        SearchByRadius(root->left, point, radius, Storage);
    }

    if (!Rebuild_Ptr || root->right != *Rebuild_Ptr) {
        SearchByRadius(root->right, point, radius, Storage);
    }
    else {
        SearchByRadius(root->right, point, radius, Storage);
    }
}

bool iKdTree::Criterion_Check(Node *root) const
{
    if (root->tree_size <= Minimal_Unbalanced_Tree_Size) {
        return false;
    }

    const auto child = root->left ? root->left : root->right;
    float delete_evaluation = float(root->invalid_point_num) / root->tree_size;
    float balance_evaluation = float(child->tree_size) / (root->tree_size - 1);
    if (delete_evaluation > delete_criterion_param) {
        return true;
    }
    if (balance_evaluation > balance_criterion_param ||
        balance_evaluation < 1 - balance_criterion_param) {
        return true;
    }
    return false;
}

void iKdTree::Push_Down(Node *root)
{
    if (!root) {
        return;
    }

    Operation operation;
    operation.type = Operation::Type::kPushDown;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
    if (root->need_push_down_to_left && root->left) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root->left) {
            root->left->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left->tree_deleted =
                root->tree_deleted || root->left->tree_downsample_deleted;
            root->left->point_deleted = root->left->tree_deleted ||
                                        root->left->point_downsample_deleted;
            if (root->tree_downsample_deleted) {
                root->left->down_del_num = root->left->tree_size;
            }
            if (root->tree_deleted) {
                root->left->invalid_point_num = root->left->tree_size;
            }
            else {
                root->left->invalid_point_num = root->left->down_del_num;
            }
            root->left->need_push_down_to_left = true;
            root->left->need_push_down_to_right = true;
            root->need_push_down_to_left = false;
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            root->left->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left->tree_deleted =
                root->tree_deleted || root->left->tree_downsample_deleted;
            root->left->point_deleted = root->left->tree_deleted ||
                                        root->left->point_downsample_deleted;
            if (root->tree_downsample_deleted) {
                root->left->down_del_num = root->left->tree_size;
            }
            if (root->tree_deleted) {
                root->left->invalid_point_num = root->left->tree_size;
            }
            else {
                root->left->invalid_point_num = root->left->down_del_num;
            }
            root->left->need_push_down_to_left = true;
            root->left->need_push_down_to_right = true;
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
            root->need_push_down_to_left = false;
        }
    }

    // FIXME: Duplicated code
    if (root->need_push_down_to_right && root->right) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root->right) {
            root->right->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right->tree_deleted =
                root->tree_deleted || root->right->tree_downsample_deleted;
            root->right->point_deleted = root->right->tree_deleted ||
                                         root->right->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right->down_del_num = root->right->tree_size;
            if (root->tree_deleted)
                root->right->invalid_point_num = root->right->tree_size;
            else
                root->right->invalid_point_num = root->right->down_del_num;
            root->right->need_push_down_to_left = true;
            root->right->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            root->right->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right->tree_deleted =
                root->tree_deleted || root->right->tree_downsample_deleted;
            root->right->point_deleted = root->right->tree_deleted ||
                                         root->right->point_downsample_deleted;
            if (root->tree_downsample_deleted) {
                root->right->down_del_num = root->right->tree_size;
            }
            if (root->tree_deleted) {
                root->right->invalid_point_num = root->right->tree_size;
            }
            else {
                root->right->invalid_point_num = root->right->down_del_num;
            }
            root->right->need_push_down_to_left = true;
            root->right->need_push_down_to_right = true;
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
            root->need_push_down_to_right = false;
        }
    }
}

void iKdTree::Update(Node *root)
{
    root->Update();

    // NOTE: Debug only
    if (root == root_ && root->tree_size > 3) {
        const auto child = root->left ? root->left : root->right;
        float tmp_bal = float(child->tree_size) / (root->tree_size - 1);
        root->alpha_del = float(root->invalid_point_num) / root->tree_size;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
}

void iKdTree::flatten(Node *root, delete_point_storage_set storage_type,
                      _PointCloud &Storage)
{
    if (!root) {
        return;
    }

    Push_Down(root);
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }
    flatten(root->left, storage_type, Storage);
    flatten(root->right, storage_type, Storage);
    switch (storage_type) {
        case NOT_RECORD:
            break;
        case DELETE_POINTS_REC:
            if (root->point_deleted && !root->point_downsample_deleted) {
                Points_deleted.push_back(root->point);
            }
            break;
        case MULTI_THREAD_REC:
            if (root->point_deleted && !root->point_downsample_deleted) {
                Multithread_Points_deleted.push_back(root->point);
            }
            break;
        default:
            break;
    }
}

void iKdTree::delete_tree_nodes(Node **root)
{
    if (!*root) {
        return;
    }

    Push_Down(*root);
    delete_tree_nodes(&(*root)->left);
    delete_tree_nodes(&(*root)->right);

    delete *root;
    *root = nullptr;
}

// manual queue
template <typename T>
void MANUAL_Q<T>::clear()
{
    head = 0;
    tail = 0;
    counter = 0;
    is_empty = true;
}

template <typename T>
void MANUAL_Q<T>::pop()
{
    if (counter == 0) {
        return;
    }

    head++;
    head %= Q_LEN;
    counter--;
    if (counter == 0) {
        is_empty = true;
    }
}

template <typename T>
T MANUAL_Q<T>::front() const
{
    return q[head];
}

template <typename T>
T MANUAL_Q<T>::back() const
{
    return q[tail];
}

template <typename T>
void MANUAL_Q<T>::push(T op)
{
    q[tail] = op;
    counter++;
    if (is_empty) {
        is_empty = false;
    }

    tail++;
    tail %= Q_LEN;
}

template <typename T>
bool MANUAL_Q<T>::empty() const
{
    return is_empty;
}

template <typename T>
int MANUAL_Q<T>::size() const
{
    return counter;
}
