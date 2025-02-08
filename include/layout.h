
#ifndef PATTERNMATCH_LAYOUT_H
#define PATTERNMATCH_LAYOUT_H
#include "geometry.h"
#include <set>
#include <map>
#include <vector>
#include <cstdlib>
#include <cstring>

#include "mio.hpp"
#include "util.h"
#include "clipper.h"

class Reader;

class LayoutPoly
{
public:
    using Point64 = Clipper2Lib::Point64;
    bool head_type;
    std::vector<int> feature;
    std::vector<Point64> points;
    int point_count;
    void update()
    {
        const auto &getEdgeLength = [&](const Point64 &end, const Point64 &start)
        {
            return end.x - start.x + end.y - start.y;
        };
        head_type = bool((points[1].x - points[0].x) == 0);
        point_count = points.size();
        for (int i = 0; i < point_count; i++)
        {
            feature.push_back(getEdgeLength(points[(i + 1) % point_count], points[i]));
        }
    }
};

class Layout
{
    using File = mio::mmap_source;

    // mio::mmap_source 的主要功能
    // 文件读取：将文件的内容映射到内存中，允许用户像读内存一样读取文件内容。
    // 延迟加载：文件内容不会一次性加载到内存中，而是按需从磁盘加载（类似于分页内存管理），这意味着即使文件很大，也不会立即占用大量内存。
    // 自动管理文件生命周期：mmap_source 会根据文件的生命周期自动映射和取消映射文件，不必手动管理文件指针。

public:
    explicit Layout(const std::string &layout_path) : layout_file_(layout_path)
    {
        // layout_iboxes.resize(num_layer_layout);
        // layout_box_.resize(num_layer_layout);
        // polygons_cache_.resize(num_layer_layout);
        // layer_poly_num.resize(num_layer_layout, 0);
    }

    // void cachePolygons(PatternIds &ptids_container, Reader &reader, int &layer_label);
    // void cachePolygons(TContainer<PatternIds> &ptids_container);

    // void cachePolygons();

    // Polygon *getPolygon(PolygonId pid, int &layer_label); // using Polygon = Clipper2Lib::Path64;

    std::vector<Polygon *> getPattern(std::vector<PolygonId> &pids, int &layer_label);

    std::vector<BoundingBox> layout_box_; // 每层总的layout范围
    std::vector<IndexBoxes> layout_iboxes;
    std::vector<int> layer_poly_num;
    std::vector<std::vector<LayoutPoly>> layout_polys;
    int num_layer_layout;
    std::vector<int> square_count;

private:
    File layout_file_;
    // std::vector<std::unordered_map<PolygonId, Polygon>> polygons_cache_;
};

#endif // PATTERNMATCH_LAYOUT_H