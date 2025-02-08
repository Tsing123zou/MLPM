#ifndef PATTERNMATCH_FILTER_H
#define PATTERNMATCH_FILTER_H

#include <vector>
#include <unordered_map>
#include <fstream>
#include <string>
#include <iostream>
#include <tbb/tbb.h>
#include <tbb/parallel_for.h>
#include <mutex>

#include "config.h"
#include "geometry.h"
#include "patternpoly.h"
#include "util.h"
// #include "check.h"
#include "layout.h"

#define TYPE(type) \
    std::string filterType() const override { return #type; }

using Paths64 = Clipper2Lib::Paths64;
using Path64 = Clipper2Lib::Path64;
using Point64 = Clipper2Lib::Point64;
using Strings = std::vector<std::string>;
using Polygon = Clipper2Lib::Path64; // 按点存储的多边形
class Filter
{
public:
    MarkerType Marker_Types[8] = {RAW, X_AXIAL, Y_AXIAL, R180, R90CW, R90CW_X_AXIAL, R90CW_Y_AXIAL, R90CW_R180};
    virtual void getPotentialMarkers(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const = 0;
    virtual void getPotentialMarkers_parallel(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const = 0;
    virtual std::string filterType() const = 0;
    virtual int get_filter_pointcount() const = 0;
    virtual ~Filter() = default;
};

class NOPOINTFIler : public Filter
{
    TYPE(NOPOINTFIler)
    void getPotentialMarkers(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
    }
    void getPotentialMarkers_parallel(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
    }
    int get_filter_pointcount() const override
    {
        return 0;
    }
};

class POLYFilter : public Filter
{
public:
    TYPE(POLYFilter)
    POLYFilter(Paths64 &target_poly_8, Marker marker, int layer_label) : target_poly_8_(target_poly_8), marker_(marker), layer_label_(layer_label)
    {
        const auto &getEdgeLength = [&](const Point64 &start, const Point64 &end)
        {
            return end.x - start.x + end.y - start.y;
        };
        const auto &getHead_type = [&](const Point64 &first, const Point64 &second)
        {
            return bool(second.y - first.y);
        };
        bool head_type = getHead_type(target_poly_8[0][0], target_poly_8[0][1]);
        if (head_type)
            Head_types = {1, 0, 0, 1, 0, 1, 1, 0};
        else
            Head_types = {0, 1, 1, 0, 1, 0, 0, 1};
        std::vector<int> targetLengths_ACW;
        std::vector<int> targetLengths_CW;
        target_poly_8_ = target_poly_8;
        point_count = target_poly_8[0].size();
        for (size_t i = 0; i < point_count; ++i)
        {
            int length = getEdgeLength(target_poly_8_[0][i], target_poly_8_[0][(i + 1) % point_count]);
            targetLengths_ACW.push_back(length);
        }
        std::reverse_copy(targetLengths_ACW.begin(), targetLengths_ACW.end(), std::back_inserter(targetLengths_CW));
        const auto &getfeature = [&](bool head_type, std::vector<int> feature) // 顺时针旋转九十度
        {
            std::vector<int> MIR_feature;
            for (int i = 0; i < feature.size(); i++)
            {
                if (head_type)
                {
                    if (i % 2)
                        MIR_feature.push_back(-feature[i]);
                    else
                    {
                        MIR_feature.push_back(feature[i]);
                    }
                }
                else
                {
                    if (i % 2)
                        MIR_feature.push_back(feature[i]);
                    else
                    {
                        MIR_feature.push_back(-feature[i]);
                    }
                }
            }
            return MIR_feature;
        };
        features.reserve(8);
        features.push_back(targetLengths_ACW);
        features.push_back(getfeature(Head_types[5], getfeature(Head_types[2], getfeature(Head_types[6], targetLengths_CW))));
        features.push_back(getfeature(Head_types[6], targetLengths_CW));
        features.push_back(getfeature(Head_types[4], getfeature(Head_types[0], targetLengths_ACW)));
        features.push_back(getfeature(Head_types[0], targetLengths_ACW));
        features.push_back(getfeature(Head_types[2], getfeature(Head_types[6], targetLengths_CW)));
        features.push_back(targetLengths_CW);
        features.push_back(getfeature(Head_types[3], getfeature(Head_types[4], getfeature(Head_types[0], targetLengths_ACW))));
        // feature_print();

        assert(marker.lx == 0 && marker.ly == 0);
        marker_R90 = BoundingBox(0, 0, marker.uy, marker.rx);
    }

    void getPotentialMarkers(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
        std::vector<LayoutPoly> &polys = layout.layout_polys[layer_label - 1];
        int polys_num = layout.layer_poly_num[layer_label - 1];
        for (int poly_index = 0; poly_index < polys_num; poly_index++)
        {
            LayoutPoly &poly = polys[poly_index];
            if (poly.point_count != point_count)
                continue;
            for (int i = 0; i < 8; ++i)
            {
                // std::vector<int> f = features[i];
                bool flag_feature = false;
                bool flag_feature_headtype = false;
                int pm_pos = 0;
                bool poly_head_type;
                for (int j = 0; j < point_count; ++j)
                {
                    if (features[i][0] == poly.feature[j])
                    {
                        flag_feature = true;
                        pm_pos = j;
                        for (int k = 1; k < point_count; ++k)
                        {
                            if (features[i][k] != poly.feature[(j + k) % point_count])
                            {
                                flag_feature = false;
                                break;
                            }
                        }
                        if (flag_feature)
                        {
                            poly_head_type = (pm_pos % 2) ? !poly.head_type : poly.head_type;
                            if (Head_types[i] == poly_head_type)
                            {
                                flag_feature_headtype = true;
                                break;
                            }
                        }
                    }
                }
                if (flag_feature_headtype)
                {
                    Point64 p = poly.points[pm_pos] - target_poly_8_[i][0];
                    if (i < 4)
                    {
                        // match_success = true;
                        Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy);
                        potential_markers.first.push_back(newm);
                        potential_markers.second.push_back(Marker_Types[i]);
                    }
                    else
                    {
                        // match_success = true;
                        Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);
                        potential_markers.first.push_back(newm);
                        potential_markers.second.push_back(Marker_Types[i]);
                    }
                }
            }
        }
    }

    void getPotentialMarkers_parallel(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
        std::vector<LayoutPoly> &polys = layout.layout_polys[layer_label - 1];
        int polys_num = layout.layer_poly_num[layer_label - 1];
        std::mutex output_mtx;

        tbb::affinity_partitioner aff;

        tbb::parallel_for(tbb::blocked_range<size_t>(0, polys_num), [&](tbb::blocked_range<size_t> r)
                          {
                std::vector<Marker> local_markers;
                std::vector<MarkerType> local_markertypes;
                for (size_t poly_index = r.begin(); poly_index < r.end(); poly_index++)
                {
                    LayoutPoly &poly = polys[poly_index];
                    if (poly.point_count != point_count)
                        continue;
                    for (int i = 0; i < 8; ++i)
                    {
                        // std::vector<int> f = features[i];
                        bool flag_feature = false;
                        bool flag_feature_headtype = false;
                        int pm_pos = 0;
                        bool poly_head_type;
                        for (int j = 0; j < point_count; ++j)
                        {
                            if (features[i][0] == poly.feature[j])
                            {
                                flag_feature = true;
                                pm_pos = j;
                                for (int k = 1; k < point_count; ++k)
                                {
                                    if (features[i][k] != poly.feature[(j + k) % point_count])
                                    {
                                        flag_feature = false;
                                        break;
                                    }
                                }
                                if (flag_feature)
                                {
                                    poly_head_type = (pm_pos % 2) ? !poly.head_type : poly.head_type;
                                    if (Head_types[i] == poly_head_type)
                                    {
                                        flag_feature_headtype = true;
                                        break;
                                    }
                                }
                            }
                        }
                        if (flag_feature_headtype)
                        {
                            Point64 p = poly.points[pm_pos] - target_poly_8_[i][0];
                            if (i < 4)
                            {
                                // match_success = true;
                                Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy);
                                local_markers.push_back(newm);
                                local_markertypes.push_back(Marker_Types[i]);
                            }   
                            else
                            {
                            // match_success = true;
                                Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);
                                local_markers.push_back(newm);
                                local_markertypes.push_back(Marker_Types[i]);
                            }
                        }
                    }
                }
                std::lock_guard<std::mutex> lock(output_mtx);
                potential_markers.first.insert(potential_markers.first.end(),local_markers.begin(),local_markers.end());
                potential_markers.second.insert(potential_markers.second.end(),local_markertypes.begin(),local_markertypes.end()); }, tbb::static_partitioner{});
    }

    int get_filter_pointcount() const override
    {
        return point_count;
    }

private:
    int layer_label_;
    std::vector<bool> Head_types;
    std::vector<std::vector<int>> features;
    int point_count;
    Paths64 target_poly_8_;
    Marker marker_;
    Marker marker_R90;
    void feature_print()
    {
        std::ofstream file("../large/patten_feature_result.txt", std::ios::app);
        if (!file.is_open())
        {
            std::cerr << "Failed to open patten_feature_result.txt for writing.\n";
            return;
        }
        file << "layer" << layer_label_ << "\n";
        for (int mir = 0; mir < 8; mir++)
        {
            file << "mir" << mir << "\n";
            file << "headtype" << Head_types[mir] << "\n";
            for (auto &len : features[mir])
            {
                file << len << ',';
            }
            file << "\n";
        }
        file.close();
    }
};

class PATHFilter : public Filter
{
public:
    TYPE(PATHFilter);
    PATHFilter(Paths64 &target_poly_8, Marker marker, int layer_label) : target_poly_8_(target_poly_8), marker_(marker), layer_label_(layer_label)
    {
        const auto &getEdgeLength = [&](const Point64 &start, const Point64 &end)
        {
            assert(end.x - start.x == 0 || end.y - start.y == 0);
            return end.x - start.x + end.y - start.y;
        };
        const auto &getHead_type = [&](const Point64 &first, const Point64 &second)
        {
            return bool(second.y - first.y);
        };
        bool head_type = getHead_type(target_poly_8[0][0], target_poly_8[0][1]);
        if (head_type)
            Head_types = {1, 0, 0, 1, 0, 1, 1, 0};
        else
            Head_types = {0, 1, 1, 0, 1, 0, 0, 1};
        std::vector<int> targetLengths_ACW; // 逆时针旋转
        std::vector<int> targetLengths_CW(targetLengths_ACW.size());
        target_poly_8_ = target_poly_8;
        point_count = target_poly_8[0].size();
        for (int i = 0; i < point_count; ++i)
        {
            int length = getEdgeLength(target_poly_8_[0][i], target_poly_8_[0][(i + 1) % point_count]);
            targetLengths_ACW.push_back(length);
        }
        std::reverse_copy(targetLengths_ACW.begin(), targetLengths_ACW.end(), std::back_inserter(targetLengths_CW));
        const auto &getfeature = [&](bool head_type, std::vector<int> feature)
        {
            std::vector<int> MIR_feature;
            for (int i = 0; i < feature.size(); i++)
            {
                if (head_type)
                {
                    if (i % 2)
                        MIR_feature.push_back(-feature[i]);
                    else
                    {
                        MIR_feature.push_back(feature[i]);
                    }
                }
                else
                {
                    if (i % 2)
                        MIR_feature.push_back(feature[i]);
                    else
                    {
                        MIR_feature.push_back(-feature[i]);
                    }
                }
            }
            return MIR_feature;
        };
        features.reserve(8);
        features.push_back(targetLengths_ACW);
        features.push_back(getfeature(Head_types[5], getfeature(Head_types[2], getfeature(Head_types[6], targetLengths_CW))));
        features.push_back(getfeature(Head_types[6], targetLengths_CW));
        features.push_back(getfeature(Head_types[4], getfeature(Head_types[0], targetLengths_ACW)));
        features.push_back(getfeature(Head_types[0], targetLengths_ACW));
        features.push_back(getfeature(Head_types[2], getfeature(Head_types[6], targetLengths_CW)));
        features.push_back(targetLengths_CW);
        features.push_back(getfeature(Head_types[3], getfeature(Head_types[4], getfeature(Head_types[0], targetLengths_ACW))));
        // feature_print();

        const auto &isOnBorader = [&](Clipper2Lib::Point64 p, Marker m)
        {
            return (p.x == m.lx || p.x == m.rx || p.y == m.ly || p.y == m.uy);
        };

        for (int i = 0; i < point_count; i++)
        {
            isinside.push_back(!isOnBorader(target_poly_8[0][i], marker_));
        }
        assert(marker.lx == 0 && marker.ly == 0);
        marker_R90 = BoundingBox(0, 0, marker.uy, marker.rx);
        // 找到内点最多的内部边
        std::pair<int, int> max = std::make_pair(0, 0); // 分别最长内部边内点记录位置和count
        int pos;                                        // 记录内点的起始位置
        int count = 0;                                  // 记录目前经过了几个内点
        bool counting = false;                          // 记录是否在数内点个数

        for (int i = 0; i < 2 * point_count; i++)
        {
            if (isinside[i % point_count])
            {
                count++;
                if (!counting)
                {
                    counting = true;
                    pos = i;
                }
            }
            else
            {
                if (counting)
                {
                    counting = false;
                    if (count > max.second)
                    {
                        max.first = pos;
                        max.second = count;
                    }
                    count = 0;
                    pos = 0;
                }
            }
        }
        std::pair<int, int> max_mir = std::make_pair((point_count - ((max.first + max.second - 1) % point_count)) % point_count, max.second);

        for (int i = 0; i < Head_types.size(); i++)
        {
            if (i == 0 || i == 3 || i == 4 || i == 7)
            {
                maxs.push_back(max);
            }
            else
            {
                maxs.push_back(max_mir);
            }
        }
        for (int i = 0; i < features.size(); i++)
        {
            std::vector<int> f;
            int start = (maxs[i].first + point_count - 1) % point_count;
            for (int j = 0; j < maxs[i].second + 1; j++)
            {
                f.push_back(features[i][(start + j) % point_count]);
            }
            ffeatures.push_back(f);
            bool head_type;
            if (maxs[i].first % 2 == 0)
            {
                head_type = !Head_types[i];
            }
            else
            {
                head_type = Head_types[i];
            } // 确定特征向量的首边走向
            fHead_types.push_back(head_type);
        };
    }
    void getPotentialMarkers(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
        std::vector<LayoutPoly> &polys = layout.layout_polys[layer_label - 1];
        int polys_num = layout.layer_poly_num[layer_label - 1];
        // std::cout << "1" << std::endl;
        // std::cout << "polys_num:" << polys_num <<std::endl;
        for (int poly_index = 0; poly_index < polys_num; poly_index++)
        {
            LayoutPoly &poly = polys[poly_index];
            if (poly.point_count < point_count)
                continue;
            // std::cout << "poly_index" << poly_index << std::endl;
            for (int i = 0; i < features.size(); ++i) // 8mir
            {
                const std::vector<int> &f = ffeatures[i];

                const auto &isLonger = [&](int x, int y)
                {
                    return (x * y > 0) && (abs(x) >= abs(y));
                };
                // 开始模糊匹配
                int flag = 0;
                int y = 0;
                if (fHead_types[i] == poly.head_type)
                {
                    for (int j = 0; j < poly.feature.size(); j += 2)
                    {
                        if (isLonger(poly.feature[j], f[0]))
                        {
                            flag = 1;
                            y = j;
                            int k;
                            for (k = 1; k < f.size() - 1; ++k)
                            {
                                if (f[k] != poly.feature[(j + k) % poly.point_count])
                                {
                                    flag = 0;
                                    break;
                                }
                            }
                            if (!(flag && isLonger(poly.feature[(j + k) % poly.point_count], f[k])))
                            {
                                flag = 0;
                            }
                            else
                            {
                                Point64 p = poly.points[(y + 1) % poly.point_count] - target_poly_8_[i][maxs[i].first];
                                // match_success = true;
                                if (i < 4)
                                {
                                    Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy);
                                    potential_markers.first.push_back(newm);
                                    potential_markers.second.push_back(Marker_Types[i]);
                                }
                                else
                                {
                                    Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);
                                    potential_markers.first.push_back(newm);
                                    potential_markers.second.push_back(Marker_Types[i]);
                                }
                            }
                        }
                    }
                }
                else
                {
                    for (int j = 1; j < poly.feature.size(); j += 2)
                    {
                        if (isLonger(poly.feature[j], f[0]))
                        {
                            flag = 1;
                            y = j;
                            int k;
                            for (k = 1; k < f.size() - 1; ++k)
                            {
                                if (f[k] != poly.feature[(j + k) % poly.point_count])
                                {
                                    flag = 0;
                                    break;
                                }
                            }
                            if (!(flag && isLonger(poly.feature[(j + k) % poly.point_count], f[k])))
                            {
                                flag = 0;
                            }
                            else
                            {
                                Point64 p = poly.points[(y + 1) % poly.point_count] - target_poly_8_[i][maxs[i].first];
                                // match_success = true;
                                if (i < 4)
                                {
                                    Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy);
                                    potential_markers.first.push_back(newm);
                                    potential_markers.second.push_back(Marker_Types[i]);
                                }
                                else
                                {
                                    Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);
                                    potential_markers.first.push_back(newm);
                                    potential_markers.second.push_back(Marker_Types[i]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void getPotentialMarkers_parallel(Layout &layout, int &layer_label, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers) const override
    {
        std::vector<LayoutPoly> &polys = layout.layout_polys[layer_label - 1];
        int polys_num = layout.layer_poly_num[layer_label - 1];
        std::mutex output_mutex;
        tbb::affinity_partitioner aff;

        // std::cout << "1" << std::endl;
        // std::cout << "polys_num:" << polys_num <<std::endl;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, polys_num), [&](tbb::blocked_range<size_t> r)
                          {
                std::vector<Marker> local_markers;
                std::vector<MarkerType> local_markertypes;
                for (size_t poly_index = r.begin(); poly_index < r.end(); poly_index++)
                {   
                    LayoutPoly &poly = polys[poly_index];
                    if (poly.point_count < point_count)
                        continue;
                    //std::cout << "poly_index" << poly_index << std::endl;
                    for (int i = 0; i < features.size(); ++i)//8mir
                    {
                        const std::vector<int> &f = ffeatures[i];

                        const auto &isLonger = [&](int x, int y)
                        {
                            return (x * y > 0) && (abs(x) >= abs(y));
                        };
                        // 开始模糊匹配
                        int flag = 0;
                        int y = 0;
                        if (fHead_types[i] == poly.head_type)
                        {   
                            for (int j = 0; j < poly.feature.size(); j += 2)
                            {
                                if (isLonger(poly.feature[j], f[0]))
                                {
                                    flag = 1;
                                    y = j;
                                    int k;
                                    for (k = 1; k < f.size() - 1; ++k)
                                    {
                                        if (f[k] != poly.feature[(j + k) % poly.point_count])
                                        {
                                            flag = 0;
                                            break;
                                        }
                                    }
                                    if (!(flag && isLonger(poly.feature[(j + k) % poly.point_count], f[k])))
                                    {
                                        flag = 0;
                                    }
                                    else
                                    {
                                        Point64 p = poly.points[(y + 1) % poly.point_count] - target_poly_8_[i][maxs[i].first];
                                        // match_success = true;
                                        if (i < 4)
                                        {
                                            Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy);
                                            local_markers.push_back(newm);
                                            local_markertypes.push_back(Marker_Types[i]);
                                        }
                                        else
                                        {
                                            Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);
                                            local_markers.push_back(newm);
                                            local_markertypes.push_back(Marker_Types[i]);
                                        }   
                                    }
                                }
                            }
                        }
                        else
                        {
                            for (int j = 1; j < poly.feature.size(); j += 2)
                            {
                                if (isLonger(poly.feature[j], f[0]))
                                {
                                    flag = 1;
                                    y = j;
                                    int k;
                                    for (k = 1; k < f.size() - 1; ++k)
                                    {
                                        if (f[k] != poly.feature[(j + k) % poly.point_count])
                                        {
                                            flag = 0;
                                            break;
                                        }
                                    }   
                                    if (!(flag && isLonger(poly.feature[(j + k) % poly.point_count], f[k])))
                                    {
                                        flag = 0;
                                    }
                                    else
                                    {
                                        Point64 p = poly.points[(y + 1) % poly.point_count] - target_poly_8_[i][maxs[i].first];
                                        // match_success = true;
                                        if (i < 4)
                                        {
                                            Marker newm = BoundingBox(p.x, p.y, p.x + marker_.rx, p.y + marker_.uy); 
                                            local_markers.push_back(newm);
                                            local_markertypes.push_back(Marker_Types[i]);
                                        }
                                        else
                                        {
                                            Marker newm = BoundingBox(p.x, p.y, p.x + marker_R90.rx, p.y + marker_R90.uy);  
                                            local_markers.push_back(newm);
                                            local_markertypes.push_back(Marker_Types[i]);
                                       }
                                   }
                               }
                           }
                        }      
                    }   
                }
                std::lock_guard<std::mutex> lock(output_mutex);
                potential_markers.first.insert(potential_markers.first.end(),local_markers.begin(),local_markers.end());
                potential_markers.second.insert(potential_markers.second.end(),local_markertypes.begin(),local_markertypes.end()); }, tbb::simple_partitioner{});
    }

    int get_filter_pointcount() const override
    {
        return point_count;
    }

private:
    int layer_label_;
    std::vector<bool> Head_types;
    std::vector<bool> fHead_types;
    Paths64 target_poly_8_;
    int point_count;
    Marker marker_;
    Marker marker_R90;
    std::vector<std::vector<int>> features;
    std::vector<bool> isinside;            // 记录点是否在marker内部
    std::vector<std::pair<int, int>> maxs; // 记录最多内点的起始位置和数目
    std::vector<std::vector<int>> ffeatures;
    bool isLonger(int x, int y) const
    {
        return (x * y > 0) && (abs(x) >= abs(y));
    }
    void feature_print()
    {
        std::ofstream file("../large/patten_feature_result.txt", std::ios::app);
        if (!file.is_open())
        {
            std::cerr << "Failed to open patten_feature_result.txt for writing.\n";
            return;
        }
        file << "layer" << layer_label_ << "\n";
        for (int mir = 0; mir < 8; mir++)
        {
            file << "mir" << mir << "\n";
            file << "headtype" << Head_types[mir] << "\n";
            for (auto &len : features[mir])
            {
                file << len << ',';
            }
            file << "\n";
        }
        file.close();
    }
};

#undef TYPE

#endif // PATTERNMATCH_FILTER_H