#include "msqtree.h"
MSQtreeManagers::MSQtreeManagers(Layout &layout)
{
    qtree_num_layer = layout.num_layer_layout;
    for (int i = 0; i < qtree_num_layer; i++)
    {
        int depth; // 需要根据layout_layout的poly数量进行修改
        if (layout.layer_poly_num[i] < 10000)
            depth = 6;
        else if (layout.layer_poly_num[i] < 100000)
            depth = 7;
        else
            depth = 8;
        // std::cout << "Tree_depth:" << depth << std::endl;
        // std::cout << "qt_layer_label:" << i + 1 << std::endl;
        // std::cout << "layer_poly_num:" << layout.layer_poly_num[i] << std::endl;
        QtreeBase::NodeBounds root_bounds(layout.layout_box_[i]);
        // std::cout << "layout.layout_box_[i]:" << layout.layout_box_[i].lx << ',' << layout.layout_box_[i].ly << ',' << layout.layout_box_[i].rx << ',' << layout.layout_box_[i].uy << std::endl;
        // std::cout << "end_create_qree_root_bounds" << std::endl;

        // TICK(markHotSpots);
        //  hot_spots是热点markers区域，范围与pattern的marker相同
        // sptr<QtreeBase::HotSpotMap> hot_spot_map(new QtreeBase::HotSpotMap(root_bounds, depth, hot_spots));
        // std::cout << "end_create_hot_spot_map" << std::endl;
        //  I4TOCK(markHotSpots);
        qtrees_.push_back(std::make_unique<MSQtree>(layout.layout_iboxes[i], layout.layer_poly_num[i], root_bounds, depth));
    }
}

std::vector<PolygonId> MSQtreeManagers::query(BoundingBox search_box, int layer_label)
{
    std::vector<PolygonId> ret;
    // if(search_box.lx == 302857 && search_box.ly ==156894 && search_box.rx == 313257 && search_box.uy == 163694)
    //         std::cout <<"1"<<std::endl;
    auto nodes = getIntersectNodes(search_box, layer_label);
    // if(search_box.lx == 302857 && search_box.ly ==156894 && search_box.rx == 313257 && search_box.uy == 163694)
    //     std::cout <<"1"<<std::endl;
    // std::cout << "start" << std::endl;
    qtrees_[layer_label - 1]->queryIntersect(nodes, search_box, ret);
    // std::cout << "end" << std::endl;
    return ret;
}

MSQtree::MSQtree(const IndexBoxes &container, int poly_num, NodeBounds root_bounds, int depth) : ibox_container_(container),
                                                                                                 poly_num_(poly_num),
                                                                                                 root_bounds_(root_bounds),
                                                                                                 depth_(depth), // 对于layout为8，对于filter是7
                                                                                                 node_contents_(1 << (2 * depth))
{
    for (int i = 0; i < depth; i++)
        off_ += 1 << (2 * i);
    node_contents_count = 1 << (2 * depth);
    // std::cout << "start_builtsubtree" << std::endl;
    // TICK(buildMainTree);
    buildSubtree(0, root_bounds_, depth_);
    //    I4TOCK(buildMainTree);
    // std::cout << "end_builtsubtree" << std::endl;
}

void MSQtree::buildSubtree(uint node_idx, NodeBounds bounds, int depth)
{
    if (depth <= 0)
        return;

    // partition bbox array
    // 对于layout depth=8
    int numSeg = 1 << depth; // num of segments that edge divided into
    // std::cout << "start" << std::endl;
    // std::cout << "bound.hw:" << bounds.hw << std::endl;
    // std::cout << "bound.hh:" << bounds.hh << std::endl;
    vert_t cell_width = int(bounds.hw * 2 / numSeg);
    vert_t cell_height = int(bounds.hh * 2 / numSeg);
    vert_t root_llx = int(bounds.cx - bounds.hw);
    vert_t root_lly = int(bounds.cy - bounds.hh);
    if (cell_width == 0 || cell_height == 0)
    {
        std::cout << "Error: cell width or height is zero." << std::endl;
        // 处理异常情况，例如返回或抛出异常
    }
    else
    {
        for (uint i = 0; i < poly_num_; i++)
        {
            auto &bbox = ibox_container_[i];
            vert_t lx = bbox.lx - root_llx;
            vert_t rx = bbox.rx - root_llx - 1;
            vert_t ly = bbox.ly - root_lly;
            vert_t uy = bbox.uy - root_lly - 1;
            uint lxi = uint(lx / cell_width);
            uint rxi = uint(rx / cell_width);
            uint lyi = uint(ly / cell_height);
            uint uyi = uint(uy / cell_height);
            // search the level and position of bbox
            for (uint row = lyi; row <= uyi; row++)
            {
                for (uint col = lxi; col <= rxi; col++)
                {
                    uint idx = C2I(col, row);
                    node_contents_[idx].ibox_idxes.push_back(i); // node_contents_中的ibox_idxes存的是poly的顺序编号
                }
            }
        }
    }
}

void MSQtree::queryInner(BoundingBox search_box, std::vector<PolygonId> &ret)
{
    std::set<PolygonId> set;
    const auto &rangeQueryNode = [&](std::deque<Node> nodes)
    {
        while (!nodes.empty())
        {
            auto [idx, dep, bounds] = nodes.front();
            nodes.pop_front();

            if (dep != depth_)
            {
                uint first_child = idx * 4 + 1;
                for (int i = ll; i <= ur; i++)
                {
                    auto ci = static_cast<ChildIndex>(i);
                    NodeBounds child_bounds = bounds.childBounds(ci);
                    if (child_bounds.intersect(search_box))
                    {
                        nodes.emplace_back(first_child + i, dep + 1, child_bounds);
                    }
                }
            }
            else
            {
                // 取最后一层中与search_box相交的idx（cell）
                for (auto i : node_contents_[idx - off_].ibox_idxes)
                {
                    auto &ibox = ibox_container_[i];
                    if (ibox.in(search_box))
                    {
                        set.insert(ibox.pid);
                    }
                }
            }
        }
    };

    rangeQueryNode({Node(0, 0, root_bounds_)});
    ret.insert(ret.end(), set.begin(), set.end());
}

void MSQtree::queryIntersect(std::vector<uint> &nodes, BoundingBox search_box, std::vector<PolygonId> &ret)
{
    std::set<PolygonId> set;
    for (auto idx : nodes)
    {
        
        for (auto i : node_contents_[idx].ibox_idxes)
        {
            auto &ibox = ibox_container_[i];
            if (ibox.intersect(search_box))
            {
                set.insert(ibox.pid);
            }
        }
    }
    ret.insert(ret.end(), set.begin(), set.end());
}

int MSQtree::queryInnerCount(std::vector<uint> &nodes, BoundingBox search_box)
{
    std::set<PolygonId> set;
    for (auto idx : nodes)
    {
        for (auto i : node_contents_[idx].ibox_idxes)
        {
            auto &ibox = ibox_container_[i];
            if (ibox.in(search_box))
            {
                set.insert(ibox.pid);
            }
        }
    }
    return (int)set.size();
}

std::vector<uint> MSQtree::getIntersectNodes(BoundingBox search_box) const
{
    // 查找与marker相交的cells
    //  返回
    std::vector<uint> ret;

    int numSeg = 1 << depth_; // num of segments that edge divided into
    vert_t cell_width = root_bounds_.hw / numSeg * 2;
    vert_t cell_height = root_bounds_.hh / numSeg * 2;
    vert_t root_llx = root_bounds_.cx - root_bounds_.hw;
    vert_t root_lly = root_bounds_.cy - root_bounds_.hh;

    auto &bbox = search_box;
    vert_t lx = bbox.lx - root_llx;
    vert_t rx = bbox.rx - root_llx - 1;
    vert_t ly = bbox.ly - root_lly;
    vert_t uy = bbox.uy - root_lly - 1;
    uint lxi = lx / cell_width;
    uint rxi = rx / cell_width;
    uint lyi = ly / cell_height;
    uint uyi = uy / cell_height;
    for (uint row = lyi; row <= uyi; row++)
    {
        for (uint col = lxi; col <= rxi; col++)
        {   
            if(row >= 0 && col >= 0){
                int m = C2I(col, row);
                if(m < node_contents_count)
                    ret.push_back(m); // C2I(col, row)得到的是索引值idx
            }
        }
    }
    // std::cout << "start" << std::endl;

    return ret;
}
