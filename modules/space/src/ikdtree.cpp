#include "ikdtree.h"

#include <chrono>

#include <pcl/point_types.h>

using namespace std::chrono_literals;

template <typename PointType>
iKdTree<PointType>::iKdTree(float delete_param, float balance_param,
                            float box_length)
{
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    Rebuild_Logger.clear();
    termination_flag = false;
    start_thread();
}

template <typename PointType>
iKdTree<PointType>::~iKdTree()
{
    stop_thread();
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node);
    PointVector().swap(PCL_Storage);
    Rebuild_Logger.clear();
}

template <typename PointType>
void iKdTree<PointType>::SetDeleteCriterion(float delete_param)
{
    delete_criterion_param = delete_param;
}

template <typename PointType>
void iKdTree<PointType>::SetBalanceCriterion(float balance_param)
{
    balance_criterion_param = balance_param;
}

template <typename PointType>
void iKdTree<PointType>::SetDownsampleBoxLength(float downsample_param)
{
    downsample_size = downsample_param;
}

template <typename PointType>
void iKdTree<PointType>::InitializeKDTree(float delete_param,
                                          float balance_param, float box_length)
{
    SetDeleteCriterion(delete_param);
    SetBalanceCriterion(balance_param);
    SetDownsampleBoxLength(box_length);
}

template <typename PointType>
void iKdTree<PointType>::InitTreeNode(iKdTreeNode *root)
{
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
}

template <typename PointType>
int iKdTree<PointType>::size() const
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
        if (Root_Node) {
            return Root_Node->TreeSize;
        }

        return 0;
    }

    if (working_flag_mutex.try_lock()) {
        int s = Root_Node->TreeSize;
        working_flag_mutex.unlock();
        return s;
    }

    return Treesize_tmp;
}

template <typename PointType>
int iKdTree<PointType>::validnum() const
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
        if (!Root_Node) {
            return 0;
        }

        return Root_Node->TreeSize - Root_Node->invalid_point_num;
    }

    if (working_flag_mutex.try_lock()) {
        int s = Root_Node->TreeSize - Root_Node->invalid_point_num;
        working_flag_mutex.unlock();
        return s;
    }

    return -1;
}

template <typename PointType>
BoxPointType iKdTree<PointType>::tree_range() const
{
    BoxPointType range;
    if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
        if (Root_Node) {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
        }
        else {
            memset(&range, 0, sizeof(range));
        }
    }
    else {
        if (working_flag_mutex.try_lock()) {
            range.vertex_min[0] = Root_Node->node_range_x[0];
            range.vertex_min[1] = Root_Node->node_range_y[0];
            range.vertex_min[2] = Root_Node->node_range_z[0];
            range.vertex_max[0] = Root_Node->node_range_x[1];
            range.vertex_max[1] = Root_Node->node_range_y[1];
            range.vertex_max[2] = Root_Node->node_range_z[1];
            working_flag_mutex.unlock();
        }
        else {
            memset(&range, 0, sizeof(range));
        }
    }
    return range;
}

template <typename PointType>
void iKdTree<PointType>::root_alpha(float &alpha_bal, float &alpha_del)
{
    if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        return;
    }

    if (working_flag_mutex.try_lock()) {
        alpha_bal = Root_Node->alpha_bal;
        alpha_del = Root_Node->alpha_del;
        working_flag_mutex.unlock();
        return;
    }

    alpha_bal = alpha_bal_tmp;
    alpha_del = alpha_del_tmp;
}

template <typename PointType>
void iKdTree<PointType>::start_thread()
{
    rebuild_thread = std::thread{iKdTree<PointType>::multi_thread_ptr, this};
    printf("Multi thread started \n");
}

template <typename PointType>
void iKdTree<PointType>::stop_thread()
{
    {
        std::lock_guard locker{termination_flag_mutex_lock};
        termination_flag = true;
    }

    if (rebuild_thread.joinable()) {
        rebuild_thread.join();
    }
}

template <typename PointType>
void *iKdTree<PointType>::multi_thread_ptr(void *arg)
{
    iKdTree *handle = (iKdTree *)arg;
    handle->multi_thread_rebuild();
    return nullptr;
}

template <typename PointType>
void iKdTree<PointType>::multi_thread_rebuild()
{
    bool terminated = false;
    iKdTreeNode *father_ptr, **new_node_ptr;
    {
        std::lock_guard lock{termination_flag_mutex_lock};
        terminated = termination_flag;
    }

    while (!terminated) {
        if (Rebuild_Ptr) {
            std::lock_guard rebuild_lock{rebuild_ptr_mutex_lock};
            std::lock_guard working_lock{working_flag_mutex};
            /* Traverse and copy */
            if (!Rebuild_Logger.empty()) {
                printf("\n\n\n\n\n\n\n\n\n\n\n ERROR!!! \n\n\n\n\n\n\n\n\n");
            }
            rebuild_flag = true;
            if (*Rebuild_Ptr == Root_Node) {
                Treesize_tmp = Root_Node->TreeSize;
                Validnum_tmp =
                    Root_Node->TreeSize - Root_Node->invalid_point_num;
                alpha_bal_tmp = Root_Node->alpha_bal;
                alpha_del_tmp = Root_Node->alpha_del;
            }
            iKdTreeNode *old_root_node = (*Rebuild_Ptr);
            father_ptr = (*Rebuild_Ptr)->father_ptr;
            PointVector().swap(Rebuild_PCL_Storage);
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
                flatten(*Rebuild_Ptr, Rebuild_PCL_Storage, MULTI_THREAD_REC);
            }

            {
                std::lock_guard search_lock{search_flag_mutex};
                search_mutex_counter = 0;
            }

            working_flag_mutex.unlock();
            /* Rebuild and update missed operations*/
            Operation_Logger_Type Operation;
            iKdTreeNode *new_root_node = nullptr;
            if (!Rebuild_PCL_Storage.empty()) {
                BuildTree(&new_root_node, 0, Rebuild_PCL_Storage.size() - 1,
                          Rebuild_PCL_Storage);
                // Rebuild has been done. Updates the blocked operations into
                // the new tree
                working_flag_mutex.lock();
                rebuild_logger_mutex_lock.lock();
                int tmp_counter = 0;
                while (!Rebuild_Logger.empty()) {
                    Operation = Rebuild_Logger.front();
                    max_queue_size =
                        std::max(max_queue_size, Rebuild_Logger.size());
                    Rebuild_Logger.pop();
                    rebuild_logger_mutex_lock.unlock();
                    working_flag_mutex.unlock();
                    run_operation(&new_root_node, Operation);
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
            // pthread_mutex_lock(&working_flag_mutex);
            {
                std::lock_guard search_lock{search_flag_mutex};
                while (search_mutex_counter != 0) {
                    search_flag_mutex.unlock();
                    std::this_thread::sleep_for(1us);
                    search_flag_mutex.lock();
                }
                search_mutex_counter = -1;
            }

            if (father_ptr->left_son_ptr == *Rebuild_Ptr) {
                father_ptr->left_son_ptr = new_root_node;
            }
            else if (father_ptr->right_son_ptr == *Rebuild_Ptr) {
                father_ptr->right_son_ptr = new_root_node;
            }
            else {
                throw "Error: Father ptr incompatible with current node\n";
            }
            if (new_root_node) {
                new_root_node->father_ptr = father_ptr;
            }

            (*Rebuild_Ptr) = new_root_node;
            int valid_old =
                old_root_node->TreeSize - old_root_node->invalid_point_num;
            int valid_new =
                new_root_node->TreeSize - new_root_node->invalid_point_num;
            if (father_ptr == STATIC_ROOT_NODE) {
                Root_Node = STATIC_ROOT_NODE->left_son_ptr;
            }
            iKdTreeNode *update_root = *Rebuild_Ptr;
            while (update_root && update_root != Root_Node) {
                update_root = update_root->father_ptr;
                if (update_root->working_flag)
                    break;
                if (update_root == update_root->father_ptr->left_son_ptr &&
                    update_root->father_ptr->need_push_down_to_left)
                    break;
                if (update_root == update_root->father_ptr->right_son_ptr &&
                    update_root->father_ptr->need_push_down_to_right)
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

        {
            std::lock_guard locker{termination_flag_mutex_lock};
            terminated = termination_flag;
        }

        std::this_thread::sleep_for(100us);
    }
    printf("Rebuild thread terminated normally\n");
}

template <typename PointType>
void iKdTree<PointType>::run_operation(iKdTreeNode **root,
                                       Operation_Logger_Type operation)
{
    switch (operation.op) {
        case ADD_POINT:
            Add_by_point(root, operation.point, false, (*root)->division_axis);
            break;
        case ADD_BOX:
            Add_by_range(root, operation.boxpoint, false);
            break;
        case DELETE_POINT:
            Delete_by_point(root, operation.point, false);
            break;
        case DELETE_BOX:
            Delete_by_range(root, operation.boxpoint, false, false);
            break;
        case DOWNSAMPLE_DELETE:
            Delete_by_range(root, operation.boxpoint, false, true);
            break;
        case PUSH_DOWN:
            (*root)->tree_downsample_deleted |=
                operation.tree_downsample_deleted;
            (*root)->point_downsample_deleted |=
                operation.tree_downsample_deleted;
            (*root)->tree_deleted =
                operation.tree_deleted || (*root)->tree_downsample_deleted;
            (*root)->point_deleted =
                (*root)->tree_deleted || (*root)->point_downsample_deleted;
            if (operation.tree_downsample_deleted)
                (*root)->down_del_num = (*root)->TreeSize;
            if (operation.tree_deleted)
                (*root)->invalid_point_num = (*root)->TreeSize;
            else
                (*root)->invalid_point_num = (*root)->down_del_num;
            (*root)->need_push_down_to_left = true;
            (*root)->need_push_down_to_right = true;
            break;
        default:
            break;
    }
}

template <typename PointType>
void iKdTree<PointType>::Build(PointVector point_cloud)
{
    if (point_cloud.empty()) {
        return;
    }

    if (Root_Node) {
        delete_tree_nodes(&Root_Node);
    }

    STATIC_ROOT_NODE = new iKdTreeNode;
    InitTreeNode(STATIC_ROOT_NODE);
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1,
              point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

template <typename PointType>
void iKdTree<PointType>::Nearest_Search(PointType point, int k_nearest,
                                        PointVector &Nearest_Points,
                                        std::vector<float> &Point_Distance,
                                        double max_dist)
{
    MANUAL_HEAP q(2 * k_nearest);
    q.clear();
    std::vector<float>().swap(Point_Distance);
    if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
        Search(Root_Node, k_nearest, point, q, max_dist);
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
        Search(Root_Node, k_nearest, point, q, max_dist);
        search_flag_mutex.lock();
        search_mutex_counter -= 1;
    }
    int k_found = std::min(k_nearest, int(q.size()));
    PointVector().swap(Nearest_Points);
    std::vector<float>().swap(Point_Distance);
    for (int i = 0; i < k_found; i++) {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
}

template <typename PointType>
void iKdTree<PointType>::Box_Search(const BoxPointType &Box_of_Point,
                                    PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

template <typename PointType>
void iKdTree<PointType>::Radius_Search(PointType point, float radius,
                                       PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

template <typename PointType>
int iKdTree<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on)
{
    int NewPointSize = PointToAdd.size();
    int tree_size = size();
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    int tmp_counter = 0;
    for (int i = 0; i < PointToAdd.size(); i++) {
        if (downsample_switch) {
            Box_of_Point.vertex_min[0] =
                floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] =
                Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] =
                floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] =
                Box_of_Point.vertex_min[1] + downsample_size;
            Box_of_Point.vertex_min[2] =
                floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] =
                Box_of_Point.vertex_min[2] + downsample_size;
            mid_point.x =
                Box_of_Point.vertex_min[0] +
                (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y =
                Box_of_Point.vertex_min[1] +
                (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z =
                Box_of_Point.vertex_min[2] +
                (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;
            PointVector().swap(Downsample_Storage);
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];
            for (int index = 0; index < Downsample_Storage.size(); index++) {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist) {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
                if (Downsample_Storage.size() > 1 ||
                    same_point(PointToAdd[i], downsample_result)) {
                    if (!Downsample_Storage.empty()) {
                        Delete_by_range(&Root_Node, Box_of_Point, true, true);
                    }

                    Add_by_point(&Root_Node, downsample_result, true,
                                 Root_Node->division_axis);
                    tmp_counter++;
                }
            }
            else {
                if (Downsample_Storage.size() > 1 ||
                    same_point(PointToAdd[i], downsample_result)) {
                    Operation_Logger_Type operation_delete, operation;
                    operation_delete.boxpoint = Box_of_Point;
                    operation_delete.op = DOWNSAMPLE_DELETE;
                    operation.point = downsample_result;
                    operation.op = ADD_POINT;
                    std::lock_guard working_lock{working_flag_mutex};
                    if (!Downsample_Storage.empty()) {
                        Delete_by_range(&Root_Node, Box_of_Point, false, true);
                    }
                    Add_by_point(&Root_Node, downsample_result, false,
                                 Root_Node->division_axis);
                    tmp_counter++;
                    if (rebuild_flag) {
                        std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                        if (!Downsample_Storage.empty()) {
                            Rebuild_Logger.push(operation_delete);
                        }
                        Rebuild_Logger.push(operation);
                    }
                };
            }
        }
        else {
            if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
                Add_by_point(&Root_Node, PointToAdd[i], true,
                             Root_Node->division_axis);
            }
            else {
                Operation_Logger_Type operation;
                operation.point = PointToAdd[i];
                operation.op = ADD_POINT;
                std::lock_guard working_lock{working_flag_mutex};
                Add_by_point(&Root_Node, PointToAdd[i], false,
                             Root_Node->division_axis);
                if (rebuild_flag) {
                    std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                    Rebuild_Logger.push(operation);
                }
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void iKdTree<PointType>::Add_Point_Boxes(std::vector<BoxPointType> &BoxPoints)
{
    for (int i = 0; i < BoxPoints.size(); i++) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
            Add_by_range(&Root_Node, BoxPoints[i], true);
        }
        else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = ADD_BOX;
            std::lock_guard working_lock{working_flag_mutex};
            Add_by_range(&Root_Node, BoxPoints[i], false);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
        }
    }
}

template <typename PointType>
void iKdTree<PointType>::Delete_Points(PointVector &PointToDel)
{
    for (int i = 0; i < PointToDel.size(); i++) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
            Delete_by_point(&Root_Node, PointToDel[i], true);
        }
        else {
            Operation_Logger_Type operation;
            operation.point = PointToDel[i];
            operation.op = DELETE_POINT;

            {
                std::scoped_lock working_lock{working_flag_mutex};
                Delete_by_point(&Root_Node, PointToDel[i], false);
                if (rebuild_flag) {
                    std::lock_guard lock{rebuild_logger_mutex_lock};
                    Rebuild_Logger.push(operation);
                }
            }
        }
    }
}

template <typename PointType>
int iKdTree<PointType>::Delete_Point_Boxes(std::vector<BoxPointType> &BoxPoints)
{
    int tmp_counter = 0;
    for (int i = 0; i < BoxPoints.size(); i++) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != Root_Node) {
            tmp_counter +=
                Delete_by_range(&Root_Node, BoxPoints[i], true, false);
        }
        else {
            Operation_Logger_Type operation;
            operation.boxpoint = BoxPoints[i];
            operation.op = DELETE_BOX;

            {
                std::scoped_lock working_lock{working_flag_mutex};
                tmp_counter +=
                    Delete_by_range(&Root_Node, BoxPoints[i], false, false);
                if (rebuild_flag) {
                    std::lock_guard lock{rebuild_logger_mutex_lock};
                    Rebuild_Logger.push(operation);
                }
            }
        }
    }
    return tmp_counter;
}

template <typename PointType>
void iKdTree<PointType>::acquire_removed_points(PointVector &removed_points)
{
    std::scoped_lock locker{points_deleted_rebuild_mutex_lock};
    for (int i = 0; i < Points_deleted.size(); i++) {
        removed_points.push_back(Points_deleted[i]);
    }
    for (int i = 0; i < Multithread_Points_deleted.size(); i++) {
        removed_points.push_back(Multithread_Points_deleted[i]);
    }
    Points_deleted.clear();
    Multithread_Points_deleted.clear();
}

template <typename PointType>
void iKdTree<PointType>::BuildTree(iKdTreeNode **root, int l, int r,
                                   PointVector &Storage)
{
    if (l > r) {
        return;
    }

    *root = new iKdTreeNode;
    InitTreeNode(*root);
    int mid = (l + r) >> 1;
    int div_axis = 0;
    int i;
    // Find the best division Axis
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    for (i = l; i <= r; i++) {
        min_value[0] = std::min(min_value[0], Storage[i].x);
        min_value[1] = std::min(min_value[1], Storage[i].y);
        min_value[2] = std::min(min_value[2], Storage[i].z);
        max_value[0] = std::max(max_value[0], Storage[i].x);
        max_value[1] = std::max(max_value[1], Storage[i].y);
        max_value[2] = std::max(max_value[2], Storage[i].z);
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
            std::nth_element(begin(Storage) + l, begin(Storage) + mid,
                             begin(Storage) + r + 1, point_cmp_x);
            break;
        case 1:
            std::nth_element(begin(Storage) + l, begin(Storage) + mid,
                             begin(Storage) + r + 1, point_cmp_y);
            break;
        case 2:
            std::nth_element(begin(Storage) + l, begin(Storage) + mid,
                             begin(Storage) + r + 1, point_cmp_z);
            break;
        default:
            std::nth_element(begin(Storage) + l, begin(Storage) + mid,
                             begin(Storage) + r + 1, point_cmp_x);
            break;
    }
    (*root)->point = Storage[mid];
    iKdTreeNode *left_son = nullptr, *right_son = nullptr;
    BuildTree(&left_son, l, mid - 1, Storage);
    BuildTree(&right_son, mid + 1, r, Storage);
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update((*root));
}

template <typename PointType>
void iKdTree<PointType>::Rebuild(iKdTreeNode **root)
{
    iKdTreeNode *father_ptr;
    if ((*root)->TreeSize >= Multi_Thread_Rebuild_Point_Num) {
        if (rebuild_ptr_mutex_lock.try_lock()) {
            if (!Rebuild_Ptr ||
                ((*root)->TreeSize > (*Rebuild_Ptr)->TreeSize)) {
                Rebuild_Ptr = root;
            }
            rebuild_ptr_mutex_lock.unlock();
        }
    }
    else {
        father_ptr = (*root)->father_ptr;
        int size_rec = (*root)->TreeSize;
        PCL_Storage.clear();
        flatten(*root, PCL_Storage, DELETE_POINTS_REC);
        delete_tree_nodes(root);
        BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
        if (*root) {
            (*root)->father_ptr = father_ptr;
        }
        if (*root == Root_Node) {
            STATIC_ROOT_NODE->left_son_ptr = *root;
        }
    }
}

template <typename PointType>
int iKdTree<PointType>::Delete_by_range(iKdTreeNode **root,
                                        BoxPointType boxpoint,
                                        bool allow_rebuild, bool is_downsample)
{
    if (!*root || (*root)->tree_deleted) {
        return 0;
    }

    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] ||
        boxpoint.vertex_min[0] > (*root)->node_range_x[1]) {
        return 0;
    }
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] ||
        boxpoint.vertex_min[1] > (*root)->node_range_y[1]) {
        return 0;
    }
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] ||
        boxpoint.vertex_min[2] > (*root)->node_range_z[1]) {
        return 0;
    }
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] &&
        boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] &&
        boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] &&
        boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample) {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }

    if (!(*root)->point_deleted && boxpoint.vertex_min[0] <= (*root)->point.x &&
        boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y &&
        boxpoint.vertex_max[1] > (*root)->point.y &&
        boxpoint.vertex_min[2] <= (*root)->point.z &&
        boxpoint.vertex_max[2] > (*root)->point.z) {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample) {
            (*root)->point_downsample_deleted = true;
        }
    }

    Operation_Logger_Type delete_box_log;
    struct timespec Timeout;
    if (is_downsample)
        delete_box_log.op = DOWNSAMPLE_DELETE;
    else
        delete_box_log.op = DELETE_BOX;
    delete_box_log.boxpoint = boxpoint;

    if (!Rebuild_Ptr || (*root)->left_son_ptr != *Rebuild_Ptr) {
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint,
                                       allow_rebuild, is_downsample);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint,
                                       false, is_downsample);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(delete_box_log);
        }
    }

    if (!Rebuild_Ptr || (*root)->right_son_ptr != *Rebuild_Ptr) {
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint,
                                       allow_rebuild, is_downsample);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint,
                                       false, is_downsample);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(delete_box_log);
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (!*root) {
        (*root)->working_flag = false;
    }

    return tmp_counter;
}

template <typename PointType>
void iKdTree<PointType>::Delete_by_point(iKdTreeNode **root, PointType point,
                                         bool allow_rebuild)
{
    if (!*root || (*root)->tree_deleted) {
        return;
    }

    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted) {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize) {
            (*root)->tree_deleted = true;
        }
        return;
    }

    Operation_Logger_Type delete_log;
    struct timespec Timeout;
    delete_log.op = DELETE_POINT;
    delete_log.point = point;
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) ||
        ((*root)->division_axis == 2 && point.z < (*root)->point.z)) {
        if (!Rebuild_Ptr || (*root)->left_son_ptr != *Rebuild_Ptr) {
            Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            Delete_by_point(&(*root)->left_son_ptr, point, false);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(delete_log);
            }
        }
    }
    else {
        if (!Rebuild_Ptr || (*root)->right_son_ptr != *Rebuild_Ptr) {
            Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            Delete_by_point(&(*root)->right_son_ptr, point, false);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(delete_log);
            }
        }
    }
    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

template <typename PointType>
void iKdTree<PointType>::Add_by_range(iKdTreeNode **root, BoxPointType boxpoint,
                                      bool allow_rebuild)
{
    if (!*root) {
        return;
    }

    (*root)->working_flag = true;
    Push_Down(*root);
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] ||
        boxpoint.vertex_min[0] > (*root)->node_range_x[1]) {
        return;
    }
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] ||
        boxpoint.vertex_min[1] > (*root)->node_range_y[1]) {
        return;
    }
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] ||
        boxpoint.vertex_min[2] > (*root)->node_range_z[1]) {
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] &&
        boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] &&
        boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] &&
        boxpoint.vertex_max[2] > (*root)->node_range_z[1]) {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }

    if (boxpoint.vertex_min[0] <= (*root)->point.x &&
        boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y &&
        boxpoint.vertex_max[1] > (*root)->point.y &&
        boxpoint.vertex_min[2] <= (*root)->point.z &&
        boxpoint.vertex_max[2] > (*root)->point.z) {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }

    Operation_Logger_Type add_box_log;
    struct timespec Timeout;
    add_box_log.op = ADD_BOX;
    add_box_log.boxpoint = boxpoint;
    if (!Rebuild_Ptr || (*root)->left_son_ptr != *Rebuild_Ptr) {
        Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        Add_by_range(&((*root)->left_son_ptr), boxpoint, false);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(add_box_log);
        }
    }

    if (!Rebuild_Ptr || (*root)->right_son_ptr != *Rebuild_Ptr) {
        Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    }
    else {
        std::lock_guard working_lock{working_flag_mutex};
        Add_by_range(&((*root)->right_son_ptr), boxpoint, false);
        if (rebuild_flag) {
            std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
            Rebuild_Logger.push(add_box_log);
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

template <typename PointType>
void iKdTree<PointType>::Add_by_point(iKdTreeNode **root, PointType point,
                                      bool allow_rebuild, int father_axis)
{
    if (!*root) {
        *root = new iKdTreeNode;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }

    (*root)->working_flag = true;
    Operation_Logger_Type add_log;
    struct timespec Timeout;
    add_log.op = ADD_POINT;
    add_log.point = point;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) ||
        ((*root)->division_axis == 2 && point.z < (*root)->point.z)) {
        if (!Rebuild_Ptr || (*root)->left_son_ptr != *Rebuild_Ptr) {
            Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild,
                         (*root)->division_axis);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            Add_by_point(&(*root)->left_son_ptr, point, false,
                         (*root)->division_axis);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(add_log);
            }
        }
    }
    else {
        if (!Rebuild_Ptr || (*root)->right_son_ptr != *Rebuild_Ptr) {
            Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild,
                         (*root)->division_axis);
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            Add_by_point(&(*root)->right_son_ptr, point, false,
                         (*root)->division_axis);
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(add_log);
            }
        }
    }

    Update(*root);

    if (Rebuild_Ptr && *Rebuild_Ptr == *root &&
        (*root)->TreeSize < Multi_Thread_Rebuild_Point_Num) {
        Rebuild_Ptr = nullptr;
    }

    if (allow_rebuild & Criterion_Check((*root))) {
        Rebuild(root);
    }

    if (*root) {
        (*root)->working_flag = false;
    }
}

template <typename PointType>
void iKdTree<PointType>::Search(iKdTreeNode *root, int k_nearest,
                                PointType point, MANUAL_HEAP &q,
                                double max_dist)
{
    if (!root || root->tree_deleted) {
        return;
    }

    double cur_dist = calc_box_dist(root, point);
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
        float dist = calc_dist(point, root->point);
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
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size() < k_nearest ||
        dist_left_node < q.top().dist && dist_right_node < q.top().dist) {
        if (dist_left_node <= dist_right_node) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->left_son_ptr) {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
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
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist) {
                if (!Rebuild_Ptr || *Rebuild_Ptr != root->right_son_ptr) {
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
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
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    search_flag_mutex.lock();
                    search_mutex_counter -= 1;
                }
            }
        }
        else {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->right_son_ptr) {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
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
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist) {
                if (!Rebuild_Ptr || *Rebuild_Ptr != root->left_son_ptr) {
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
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
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    search_flag_mutex.lock();
                    search_mutex_counter -= 1;
                }
            }
        }
    }
    else {
        if (dist_left_node < q.top().dist) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->left_son_ptr) {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
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
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
        }
        if (dist_right_node < q.top().dist) {
            if (!Rebuild_Ptr || *Rebuild_Ptr != root->right_son_ptr) {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
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
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                search_flag_mutex.lock();
                search_mutex_counter -= 1;
            }
        }
    }
}

template <typename PointType>
void iKdTree<PointType>::Search_by_range(iKdTreeNode *root,
                                         BoxPointType boxpoint,
                                         PointVector &Storage)
{
    if (!root) {
        return;
    }

    Push_Down(root);

    if (boxpoint.vertex_max[0] <= root->node_range_x[0] ||
        boxpoint.vertex_min[0] > root->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] ||
        boxpoint.vertex_min[1] > root->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] ||
        boxpoint.vertex_min[2] > root->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] &&
        boxpoint.vertex_max[0] > root->node_range_x[1] &&
        boxpoint.vertex_min[1] <= root->node_range_y[0] &&
        boxpoint.vertex_max[1] > root->node_range_y[1] &&
        boxpoint.vertex_min[2] <= root->node_range_z[0] &&
        boxpoint.vertex_max[2] > root->node_range_z[1]) {
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    if (boxpoint.vertex_min[0] <= root->point.x &&
        boxpoint.vertex_max[0] > root->point.x &&
        boxpoint.vertex_min[1] <= root->point.y &&
        boxpoint.vertex_max[1] > root->point.y &&
        boxpoint.vertex_min[2] <= root->point.z &&
        boxpoint.vertex_max[2] > root->point.z) {
        if (!root->point_deleted) {
            Storage.push_back(root->point);
        }
    }

    if (!Rebuild_Ptr || root->left_son_ptr != *Rebuild_Ptr) {
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        Search_by_range(root->left_son_ptr, boxpoint, Storage);
    }

    if (!Rebuild_Ptr || root->right_son_ptr != *Rebuild_Ptr) {
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        Search_by_range(root->right_son_ptr, boxpoint, Storage);
    }
}

template <typename PointType>
void iKdTree<PointType>::Search_by_radius(iKdTreeNode *root, PointType point,
                                          float radius, PointVector &Storage)
{
    if (!root) {
        return;
    }

    Push_Down(root);
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    float dist = sqrt(calc_dist(range_center, point));
    if (dist > radius + sqrt(root->radius_sq))
        return;
    if (dist <= radius - sqrt(root->radius_sq)) {
        flatten(root, Storage, NOT_RECORD);
        return;
    }

    if (!root->point_deleted &&
        calc_dist(root->point, point) <= radius * radius) {
        Storage.push_back(root->point);
    }

    if (!Rebuild_Ptr || root->left_son_ptr != *Rebuild_Ptr) {
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }
    else {
        std::lock_guard search_lock{search_flag_mutex};
        Search_by_radius(root->left_son_ptr, point, radius, Storage);
    }

    if (!Rebuild_Ptr || root->right_son_ptr != *Rebuild_Ptr) {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
    else {
        Search_by_radius(root->right_son_ptr, point, radius, Storage);
    }
}

template <typename PointType>
bool iKdTree<PointType>::Criterion_Check(iKdTreeNode *root)
{
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size) {
        return false;
    }

    iKdTreeNode *son_ptr = root->left_son_ptr;
    if (!son_ptr) {
        son_ptr = root->right_son_ptr;
    }

    float delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    float balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
    if (delete_evaluation > delete_criterion_param) {
        return true;
    }
    if (balance_evaluation > balance_criterion_param ||
        balance_evaluation < 1 - balance_criterion_param) {
        return true;
    }
    return false;
}

template <typename PointType>
void iKdTree<PointType>::Push_Down(iKdTreeNode *root)
{
    if (!root) {
        return;
    }

    Operation_Logger_Type operation;
    operation.op = PUSH_DOWN;
    operation.tree_deleted = root->tree_deleted;
    operation.tree_downsample_deleted = root->tree_downsample_deleted;
    if (root->need_push_down_to_left && root->left_son_ptr) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root->left_son_ptr) {
            root->left_son_ptr->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted =
                root->tree_deleted ||
                root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted =
                root->left_son_ptr->tree_deleted ||
                root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) {
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            }
            if (root->tree_deleted) {
                root->left_son_ptr->invalid_point_num =
                    root->left_son_ptr->TreeSize;
            }
            else {
                root->left_son_ptr->invalid_point_num =
                    root->left_son_ptr->down_del_num;
            }
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_left = false;
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            root->left_son_ptr->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left_son_ptr->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->left_son_ptr->tree_deleted =
                root->tree_deleted ||
                root->left_son_ptr->tree_downsample_deleted;
            root->left_son_ptr->point_deleted =
                root->left_son_ptr->tree_deleted ||
                root->left_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted) {
                root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
            }
            if (root->tree_deleted) {
                root->left_son_ptr->invalid_point_num =
                    root->left_son_ptr->TreeSize;
            }
            else {
                root->left_son_ptr->invalid_point_num =
                    root->left_son_ptr->down_del_num;
            }
            root->left_son_ptr->need_push_down_to_left = true;
            root->left_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
            root->need_push_down_to_left = false;
        }
    }

    // FIXME: Duplicated code
    if (root->need_push_down_to_right && root->right_son_ptr) {
        if (!Rebuild_Ptr || *Rebuild_Ptr != root->right_son_ptr) {
            root->right_son_ptr->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted =
                root->tree_deleted ||
                root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted =
                root->right_son_ptr->tree_deleted ||
                root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num =
                    root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num =
                    root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num =
                    root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            root->need_push_down_to_right = false;
        }
        else {
            std::lock_guard working_lock{working_flag_mutex};
            root->right_son_ptr->tree_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right_son_ptr->point_downsample_deleted |=
                root->tree_downsample_deleted;
            root->right_son_ptr->tree_deleted =
                root->tree_deleted ||
                root->right_son_ptr->tree_downsample_deleted;
            root->right_son_ptr->point_deleted =
                root->right_son_ptr->tree_deleted ||
                root->right_son_ptr->point_downsample_deleted;
            if (root->tree_downsample_deleted)
                root->right_son_ptr->down_del_num =
                    root->right_son_ptr->TreeSize;
            if (root->tree_deleted)
                root->right_son_ptr->invalid_point_num =
                    root->right_son_ptr->TreeSize;
            else
                root->right_son_ptr->invalid_point_num =
                    root->right_son_ptr->down_del_num;
            root->right_son_ptr->need_push_down_to_left = true;
            root->right_son_ptr->need_push_down_to_right = true;
            if (rebuild_flag) {
                std::lock_guard rebuild_lock{rebuild_logger_mutex_lock};
                Rebuild_Logger.push(operation);
            }
            root->need_push_down_to_right = false;
        }
    }
}

template <typename PointType>
void iKdTree<PointType>::Update(iKdTreeNode *root)
{
    iKdTreeNode *left_son_ptr = root->left_son_ptr;
    iKdTreeNode *right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};
    // Update Tree Size
    if (left_son_ptr && right_son_ptr) {
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num +
                                  right_son_ptr->invalid_point_num +
                                  (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num +
                             right_son_ptr->down_del_num +
                             (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted &
                                        right_son_ptr->tree_downsample_deleted &
                                        root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted &&
                             right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted ||
            (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted &&
             !root->point_deleted)) {
            tmp_range_x[0] =
                std::min({left_son_ptr->node_range_x[0],
                          right_son_ptr->node_range_x[0], root->point.x});
            tmp_range_x[1] =
                std::max({left_son_ptr->node_range_x[1],
                          right_son_ptr->node_range_x[1], root->point.x});
            tmp_range_y[0] =
                std::min({left_son_ptr->node_range_y[0],
                          right_son_ptr->node_range_y[0], root->point.y});
            tmp_range_y[1] =
                std::max({left_son_ptr->node_range_y[1],
                          right_son_ptr->node_range_y[1], root->point.y});
            tmp_range_z[0] =
                std::min({left_son_ptr->node_range_z[0],
                          right_son_ptr->node_range_z[0], root->point.z});
            tmp_range_z[1] =
                std::max({left_son_ptr->node_range_z[1],
                          right_son_ptr->node_range_z[1], root->point.z});
        }
        else {
            if (!left_son_ptr->tree_deleted) {
                tmp_range_x[0] =
                    std::min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] =
                    std::max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] =
                    std::min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] =
                    std::max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] =
                    std::min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] =
                    std::max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted) {
                tmp_range_x[0] =
                    std::min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] =
                    std::max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] =
                    std::min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] =
                    std::max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] =
                    std::min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] =
                    std::max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted) {
                tmp_range_x[0] = std::min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = std::max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = std::min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = std::max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = std::min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = std::max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (left_son_ptr) {
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num =
            left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num +
                             (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted &
                                        root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted ||
            (!left_son_ptr->tree_deleted && !root->point_deleted)) {
            tmp_range_x[0] =
                std::min(left_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] =
                std::max(left_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] =
                std::min(left_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] =
                std::max(left_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] =
                std::min(left_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] =
                std::max(left_son_ptr->node_range_z[1], root->point.z);
        }
        else {
            if (!left_son_ptr->tree_deleted) {
                tmp_range_x[0] =
                    std::min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] =
                    std::max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] =
                    std::min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] =
                    std::max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] =
                    std::min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] =
                    std::max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted) {
                tmp_range_x[0] = std::min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = std::max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = std::min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = std::max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = std::min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = std::max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (right_son_ptr) {
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num =
            right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = right_son_ptr->down_del_num +
                             (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted &
                                        root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted ||
            (!right_son_ptr->tree_deleted && !root->point_deleted)) {
            tmp_range_x[0] =
                std::min(right_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] =
                std::max(right_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] =
                std::min(right_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] =
                std::max(right_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] =
                std::min(right_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] =
                std::max(right_son_ptr->node_range_z[1], root->point.z);
        }
        else {
            if (!right_son_ptr->tree_deleted) {
                tmp_range_x[0] =
                    std::min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] =
                    std::max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] =
                    std::min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] =
                    std::max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] =
                    std::min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] =
                    std::max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted) {
                tmp_range_x[0] = std::min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = std::max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = std::min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = std::max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = std::min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = std::max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted ? 1 : 0);
        root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y;
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;
    }

    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L * x_L + y_L * y_L + z_L * z_L;
    if (left_son_ptr) {
        left_son_ptr->father_ptr = root;
    }
    if (right_son_ptr) {
        right_son_ptr->father_ptr = root;
    }
    if (root == Root_Node && root->TreeSize > 3) {
        iKdTreeNode *son_ptr = root->left_son_ptr;
        if (!son_ptr) {
            son_ptr = root->right_son_ptr;
        }
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1 - tmp_bal;
    }
}

template <typename PointType>
void iKdTree<PointType>::flatten(iKdTreeNode *root, PointVector &Storage,
                                 delete_point_storage_set storage_type)
{
    if (!root) {
        return;
    }

    Push_Down(root);
    if (!root->point_deleted) {
        Storage.push_back(root->point);
    }
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);
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

template <typename PointType>
void iKdTree<PointType>::delete_tree_nodes(iKdTreeNode **root)
{
    if (!*root) {
        return;
    }

    Push_Down(*root);
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);

    delete *root;
    *root = nullptr;
}

template <typename PointType>
bool iKdTree<PointType>::same_point(PointType a, PointType b)
{
    return (std::abs(a.x - b.x) < EPSS && std::abs(a.y - b.y) < EPSS &&
            std::abs(a.z - b.z) < EPSS);
}

template <typename PointType>
float iKdTree<PointType>::calc_dist(PointType a, PointType b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
           (a.z - b.z) * (a.z - b.z);
}

template <typename PointType>
float iKdTree<PointType>::calc_box_dist(iKdTreeNode *node, PointType point)
{
    if (!node) {
        return INFINITY;
    }

    float min_dist = 0.f;
    if (point.x < node->node_range_x[0])
        min_dist += (point.x - node->node_range_x[0]) *
                    (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
        min_dist += (point.x - node->node_range_x[1]) *
                    (point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0])
        min_dist += (point.y - node->node_range_y[0]) *
                    (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
        min_dist += (point.y - node->node_range_y[1]) *
                    (point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0])
        min_dist += (point.z - node->node_range_z[0]) *
                    (point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1])
        min_dist += (point.z - node->node_range_z[1]) *
                    (point.z - node->node_range_z[1]);
    return min_dist;
}

template <typename PointType>
bool iKdTree<PointType>::point_cmp_x(PointType a, PointType b)
{
    return a.x < b.x;
}
template <typename PointType>
bool iKdTree<PointType>::point_cmp_y(PointType a, PointType b)
{
    return a.y < b.y;
}
template <typename PointType>
bool iKdTree<PointType>::point_cmp_z(PointType a, PointType b)
{
    return a.z < b.z;
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
bool MANUAL_Q<T>::empty()
{
    return is_empty;
}

template <typename T>
int MANUAL_Q<T>::size()
{
    return counter;
}

template class iKdTree<ikdTree_PointType>;
template class iKdTree<pcl::PointXYZ>;
template class iKdTree<pcl::PointXYZI>;
template class iKdTree<pcl::PointXYZINormal>;
