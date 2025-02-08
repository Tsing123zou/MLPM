#include "config.h"
#include "read.h"
#include "layout.h"
#include "msqtree.h"

#include <unordered_set>
#include <tuple>
#include <sstream>
#include <regex>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <sys/time.h>
#include <tbb/tbb.h>
#include <mutex>
#include <tbb/parallel_for.h>

using File = mio::mmap_source;
struct Onelayer_FinalResult
{
    std::vector<Marker> markers;
    std::vector<MarkerType> markerTypes;
    int layer_label;
};

std::vector<std::tuple<Marker, MarkerType, std::vector<int>>> newfindFrequentMarkers(const std::vector<Onelayer_FinalResult> &layers_result, int times, int num_layer)
{
    std::vector<std::tuple<Marker, MarkerType, std::vector<int>>> result;

    if (times < 1)
        return result;

    // 用于统计每个 (Marker, MarkerType) 组合的次数和出现的层
    struct MarkerInfo
    {
        int count;
        std::vector<int> layers;
    };

    // 使用 PairHasher 对应的 unordered_map
    std::unordered_map<std::pair<Marker, MarkerType>, MarkerInfo, PairHasher> count_map;

    // 遍历 layers_result 中每一层的数据
    for (const auto &layer : layers_result)
    {
        const auto &markers = layer.markers;
        const auto &types = layer.markerTypes;
        int layer_label = layer.layer_label;

        // 假设 markers 和 types 长度相同
        for (size_t i = 0; i < markers.size(); ++i)
        {
            auto key = std::make_pair(markers[i], types[i]);
            auto &marker_info = count_map[key];

            marker_info.count++;
            // 如果当前层未记录，加入 layers
            if (marker_info.layers.empty() || std::find(marker_info.layers.begin(), marker_info.layers.end(), layer_label) == marker_info.layers.end())
            {
                marker_info.layers.push_back(layer_label);
            }
        }
    }

    // 收集符合条件的 (Marker, MarkerType, 层信息) 组合
    if (times == 3)
    {
        for (const auto &entry : count_map)
        {
            const auto &key = entry.first;
            const auto &marker_info = entry.second;

            if (marker_info.count >= times && marker_info.count < num_layer)
            {
                result.emplace_back(key.first, key.second, marker_info.layers);
            }
        }
    }
    else
    {
        for (const auto &entry : count_map)
        {
            const auto &key = entry.first;
            const auto &marker_info = entry.second;

            if (marker_info.count >= times)
            {
                result.emplace_back(key.first, key.second, marker_info.layers);
            }
        }
    }
    return result;
}

void writeXorResToFile(const std::vector<std::pair<Marker, std::vector<Paths64>>> &Xor_res, const std::string &output_path, int pattern_label)
{
    // 打开文件，使用 std::ios::app 模式以追加内容
    if (pattern_label == 1)
    {
        std::ofstream file(output_path);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }
        int i = 0;
        file << "pattern" << pattern_label << ":" << std::endl;
        for (const auto &entry : Xor_res)
        {
            i++;
            const Marker &marker = entry.first;
            const std::vector<Paths64> &paths = entry.second;

            // 写入 Marker 信息
            std::cout << "marker" << i << ":(" << marker.lx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.uy << ")" << ",(" << marker.lx << ',' << marker.uy << ")" << std::endl;
            file << "marker" << i << ":" << "\n";
            file << "(" << marker.lx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.uy << ")" << ",(" << marker.lx << ',' << marker.uy << ")" << "\n";
            // 写入 Paths64 信息
            int j = 0;
            for (const auto &path : paths)
            {
                j++;
                if (path.size() == 0)
                    continue;
                file << "layer" << j << ":\n";
                for (const auto &poly : path)
                {
                    for (int t = 0; t < poly.size() - 1; t++)
                    {
                        auto &point = poly[t];
                        file << "(" << point.x << ',' << point.y << "),";
                    }
                    file << "(" << poly[poly.size() - 1].x << ',' << poly[poly.size() - 1].y << ")" << "\n";
                }
            }
        }

        file.close();
        std::cout << "Data written to " << output_path << std::endl;
    }
    else
    {
        std::ofstream file(output_path, std::ios::app);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for writing.\n";
            return;
        }
        int i = 0;
        file << "pattern" << pattern_label << ":" << std::endl;
        for (const auto &entry : Xor_res)
        {
            i++;
            const Marker &marker = entry.first;
            const std::vector<Paths64> &paths = entry.second;

            // 写入 Marker 信息
            std::cout << "marker" << i << ":(" << marker.lx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.uy << ")" << ",(" << marker.lx << ',' << marker.uy << ")" << std::endl;
            file << "marker" << i << ":" << "\n";
            file << "(" << marker.lx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.ly << ")" << ",(" << marker.rx << ',' << marker.uy << ")" << ",(" << marker.lx << ',' << marker.uy << ")" << "\n";
            // 写入 Paths64 信息
            int j = 0;
            for (const auto &path : paths)
            {
                j++;
                if (path.size() == 0)
                    continue;
                file << "layer" << j << ":\n";
                for (const auto &poly : path)
                {
                    for (int t = 0; t < poly.size() - 1; t++)
                    {
                        auto &point = poly[t];
                        file << "(" << point.x << ',' << point.y << "),";
                    }
                    file << "(" << poly[poly.size() - 1].x << ',' << poly[poly.size() - 1].y << ")" << "\n";
                }
            }
        }

        file.close();
        std::cout << "Data written to " << output_path << std::endl;
    }
}

void flow()
{
    // struct timeval start0, end0;
    // double duration0 = 0;
    // double duration1 = 0;
    // double duration2 = 0;
    // double duration3 = 0;
    // double duration4 = 0;
    // double duration5 = 0;
    // gettimeofday(&start0, NULL);
    File layout_file(g_config.layout_path);
    Reader read(g_config.pattern_path, g_config.layout_path);
    Layout layout(g_config.layout_path);
    read.readLayout(layout);
    int num_layer = read.num_layer;
    // std::vector<MSQtreeManager> qms;
    // 需要修改MSQtree的定义，不设定热点区域，同时可以直接在一个类里面管理多个层。
    MSQtreeManagers qms(layout);
    // pattern层可能是NOPointFIlter、可能是空层；

    int first_index_maxcount_square_layer = -1;
    int second_index_maxcount_square_layer = -1;
    int first_maxcount_square_layer = 50000;
    int second_maxcount_square_layer = 50000;
    int max_count = 0;
    int many_square_times = 0;
    for (int t = 0; t < num_layer; t++)
    {
        if (layout.square_count[t] > 50000)
        {
            many_square_times++;
            // std::cout << "layout.square_count[t]:" <<layout.square_count[t] <<std::endl;
            if (layout.square_count[t] > first_maxcount_square_layer)
            {
                first_maxcount_square_layer = layout.square_count[t];
                first_index_maxcount_square_layer = t;
            }
            else if (layout.square_count[t] > second_maxcount_square_layer)
            {
                second_maxcount_square_layer = layout.square_count[t];
                second_index_maxcount_square_layer = t;
            }
        }
    }
    // std::cout << "first_index_maxcount_square_layer:" << first_index_maxcount_square_layer <<std::endl;
    // std::cout << "second_index_maxcount_square_layer:" << second_index_maxcount_square_layer <<std::endl;
    // gettimeofday(&end0, NULL);
    // duration0 = (end0.tv_sec - start0.tv_sec) + (end0.tv_usec - start0.tv_usec) / 1000000.0;
    // std::cout << "readLayout and qms Time: " << duration0 << " seconds" << std::endl;

    while (1)
    {

        int pattern_label = 0;
        // struct timeval start1, end1;
        // gettimeofday(&start1, NULL);
        auto checker = read.readPattern(pattern_label);
        // std::cout << "pattern_label:" << pattern_label << std::endl;
        // 对于内部只有矩形的情况，选长方形；
        auto filter = checker.getFilter(); // 得到该pattern的filter
        // 当不存在NOPOINTFilter时，如果出现POLYPATH的特征是正方形，且Layout中出现该方形的数量大于20w时选择跳过该层。
        int NO_point_layer_count = 0;
        for (int t = 0; t < num_layer; t++)
        {
            if (filter[t]->filterType() == "NOPOINTFIler")
                NO_point_layer_count++;
        }
        std::vector<Onelayer_FinalResult> layers_result;
        int continue_times = 0;

        // gettimeofday(&end1, NULL);
        // double durationreadpattern = (end1.tv_sec - start1.tv_sec) + (end1.tv_usec - start1.tv_usec) / 1000000.0;
        // std::cout << "readpattern and getfilter times: " << durationreadpattern << " seconds" << std::endl;
        // duration1 += durationreadpattern;

        for (int i = 0; i < num_layer; i++)
        {
            std::pair<std::vector<Marker>, std::vector<MarkerType>> potential_markers; // 一层的potentialmarker
            Onelayer_FinalResult final_result;                                         // 一层完全匹配的marker
            if (filter[i]->filterType() == "NOPOINTFIler")
            {
                // std::cout << "have NOPOINTFIler" << std::endl;
                continue_times++;
                continue;
            }
            else if (NO_point_layer_count == 0 && filter[i]->get_filter_pointcount() == 4 && (i == first_index_maxcount_square_layer || i == second_index_maxcount_square_layer))
            {
                // std::cout << "have many_square_filter" << std::endl;
                continue_times++;
                continue;
            }
            else if (NO_point_layer_count == 1 && filter[i]->get_filter_pointcount() == 4 && i == first_index_maxcount_square_layer)
            {
                // std::cout << "have many_square_filter" << std::endl;
                continue_times++;
                continue;
            }
            int layer_label = i + 1;
            // struct timeval start2, end2;
            // gettimeofday(&start2, NULL);
            filter[i]->getPotentialMarkers(layout, layer_label, potential_markers);
            // gettimeofday(&end2, NULL);
            // // 计算程序运行时间
            // double durationgetpotential = (end2.tv_sec - start2.tv_sec) + (end2.tv_usec - start2.tv_usec) / 1000000.0;
            // // 输出运行时间
            // std::cout << "GetPotential Time: " << durationgetpotential << " seconds" << std::endl;
            // duration2 += durationgetpotential;
            // std::cout << "potential_markers.size:" << potential_markers.first.size() << std::endl;
            // struct timeval start3, end3;
            // gettimeofday(&start3, NULL);

            PatternIds query_result; // PatternIds = std::vector<std::vector<PolygonId>>;一个元素代表一个marker对应的polys，PolygonId存的是这一层中第几个poly。
            for (auto &search_box : potential_markers.first)
            {
                query_result.push_back(qms.query(search_box, layer_label));
            }
            // gettimeofday(&end3, NULL);
            // double durationquery = (end3.tv_sec - start3.tv_sec) + (end3.tv_usec - start3.tv_usec) / 1000000.0;
            // // 输出运行时间
            // std::cout << "query Time: " << durationquery << " seconds" << std::endl;
            // duration3 += durationquery;

            // struct timeval start4, end4;
            // gettimeofday(&start4, NULL);
            for (size_t m = 0; m < query_result.size(); m++)
            {
                auto &ptid = query_result[m]; // marker相交的poly
                if (ptid.empty())
                {
                    assert(false && " marker have no poly");
                }
                auto &marker = potential_markers.first[m];
                auto &type = potential_markers.second[m];
                Paths64 Xor;

                if (checker.check(layout.getPattern(ptid, layer_label), marker, type, layer_label, Xor))
                {
                    if (Xor.empty())
                    {
                        final_result.markers.push_back(marker);
                        final_result.markerTypes.push_back(type);
                    }
                    else if (Xor.size() == 1)
                    {
                        if (Xor[0].size() % 2 == 1)
                        {
                            final_result.markers.push_back(marker);
                            final_result.markerTypes.push_back(type);
                        }
                        else if (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))
                        {
                            final_result.markers.push_back(marker);
                            final_result.markerTypes.push_back(type);
                        }
                    }
                    else if (Xor.size() == 2)
                    {
                        bool flag1 = false;
                        bool flag2 = false;
                        if (Xor[0].size() % 2 == 1 || (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1)))
                            flag1 = true;
                        if (Xor[1].size() % 2 == 1 || (Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1)))
                            flag2 = true;
                        if (flag1 && flag2)
                        {
                            final_result.markers.push_back(marker);
                            final_result.markerTypes.push_back(type);
                        }
                    }
                }
            }
            // std::cout <<"final_result.size:"<<final_result.first.size()<<std::endl;
            final_result.layer_label = layer_label;
            layers_result.push_back(final_result);
            // gettimeofday(&end4, NULL);
            // // 计算程序运行时间
            // double durationXor = (end4.tv_sec - start4.tv_sec) + (end4.tv_usec - start4.tv_usec) / 1000000.0;
            // // XO运行时间
            // std::cout << "Xor Time: " << durationXor << " seconds" << std::endl;
            // duration4 += durationXor;
        }
        int times = 3 - continue_times;
        std::vector<std::tuple<Marker, MarkerType, std::vector<int>>> potentail_layer_result = newfindFrequentMarkers(layers_result, times, num_layer);
        // std::cout <<"potentail_layer_result.szie"<<potentail_layer_result.size()<<std::endl;
        std::vector<std::pair<Marker, std::vector<Paths64>>> Xor_res;
        // struct timeval start5, end5;
        // gettimeofday(&start5, NULL);
        // potentail_layer_result_print(potentail_layer_result);
        for (int i = 0; i < potentail_layer_result.size(); i++)
        {
            auto &one_result = potentail_layer_result[i];
            auto &final_marker = std::get<0>(one_result);
            auto &final_markertype = std::get<1>(one_result);
            auto &have_check_labels = std::get<2>(one_result);
            std::vector<Paths64> one_marker_res;
            int one_marker_xorempty_layer_count = 0;
            for (int j = 0; j < num_layer; j++)
            {
                int layer_label = j + 1;
                if (std::find(have_check_labels.begin(), have_check_labels.end(), layer_label) != have_check_labels.end())
                {
                    one_marker_xorempty_layer_count++;
                    Paths64 empty_Xor;
                    one_marker_res.push_back(empty_Xor);
                    continue;
                }
                PatternId query_one_marker_patternID = qms.query(final_marker, layer_label);
                std::vector<Polygon *> query_one_marker_polygons = layout.getPattern(query_one_marker_patternID, layer_label);
                // std::vector<Polygon> query_one_marker_polygons = read.check_readPolygons(layout, query_one_marker_patternID, layer_label);
                auto Xor = checker.final_check(query_one_marker_polygons, final_marker, final_markertype, j + 1);
                if (times != 3)
                {
                    if (Xor.empty())
                    {
                        one_marker_xorempty_layer_count++;
                    }
                    if (Xor.size() == 1)
                    {
                        if (Xor[0].size() % 2 == 1)
                        {
                            one_marker_xorempty_layer_count++;
                        }
                        else if (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))
                        {
                            one_marker_xorempty_layer_count++;
                        }
                    }
                    else if (Xor.size() == 2)
                    {
                        bool flag1 = false;
                        bool flag2 = false;
                        if ((Xor[0].size() % 2 == 1) || (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1)))
                            flag1 = true;
                        if ((Xor[1].size() % 2 == 1) || (Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1)))
                            flag2 = true;
                        if (flag1 && flag2)
                        {
                            one_marker_xorempty_layer_count++;
                        }
                        else if ((!flag1) && flag2)
                        {
                            Xor.erase(Xor.begin() + 1);
                        }
                        else if (flag1 && (!flag2))
                        {
                            Xor.erase(Xor.begin());
                        }
                    }
                }
                else
                {
                    if (Xor.empty())
                        one_marker_xorempty_layer_count++;
                    if (Xor.size() == 1)
                    {
                        if (Xor[0].size() % 2 == 1)
                        {
                            one_marker_xorempty_layer_count++;
                        }
                        else if (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))
                        {
                            one_marker_xorempty_layer_count++;
                        }
                    }
                    else if (Xor.size() == 2)
                    {
                        bool flag1 = false;
                        bool flag2 = false;
                        if ((Xor[0].size() % 2 == 1) || (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1)))
                            flag1 = true;
                        if ((Xor[1].size() % 2 == 1) || (Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1)))
                            flag2 = true;

                        if (flag1 && flag2)
                        {
                            one_marker_xorempty_layer_count++;
                        }
                        else if ((!flag1) && flag2)
                        {
                            Xor.erase(Xor.begin() + 1);
                        }
                        else if (flag1 && (!flag2))
                        {
                            Xor.erase(Xor.begin());
                        }
                    }
                }
                one_marker_res.push_back(Xor);
            }
            if (one_marker_xorempty_layer_count >= 3 && one_marker_xorempty_layer_count < num_layer)
                Xor_res.push_back(std::pair(final_marker, one_marker_res));
        }
        // gettimeofday(&end5, NULL);
        // // 计算程序运行时间
        // double durationfinal = (end5.tv_sec - start5.tv_sec) + (end5.tv_usec - start5.tv_usec) / 1000000.0;
        // // XO运行时间
        // std::cout << "final Time: " << durationfinal << " seconds" << std::endl;
        // duration5 += durationfinal;
        writeXorResToFile(Xor_res, g_config.output_path, pattern_label);
        if (read.readPatternchar() != 'p')
            break;
    }
    // std::cout << "readLayout_and qms times: " << duration0 << " seconds" << std::endl;
    // std::cout << "readpattern and getfilter times: " << duration1 << " seconds" << std::endl;
    // std::cout << "GetPotential Time: " << duration2 << " seconds" << std::endl;
    // std::cout << "query Time:" << duration3 << " seconds" << std::endl;
    // std::cout << "XO Time:" << duration4 << " seconds" << std::endl;
    // std::cout << "final Time:" << duration5 << " seconds" << std::endl;
}

void multithreadflow()
{
    // struct timeval start0, end0;
    // double duration0 = 0;
    // double duration1 = 0;
    // double duration2 = 0;
    // double duration3 = 0;
    // double duration4 = 0;
    // double duration5 = 0;
    // gettimeofday(&start0, NULL);
    File layout_file(g_config.layout_path);
    Reader read(g_config.pattern_path, g_config.layout_path);
    Layout layout(g_config.layout_path);
    read.readLayout(layout);
    int num_layer = read.num_layer;
    // std::vector<MSQtreeManager> qms;
    // 需要修改MSQtree的定义，不设定热点区域，同时可以直接在一个类里面管理多个层。
    MSQtreeManagers qms(layout);
    // pattern层可能是NOPointFIlter、可能是空层；
    int first_index_maxcount_square_layer = -1;
    int second_index_maxcount_square_layer = -1;
    int first_maxcount_square_layer = 50000;
    int second_maxcount_square_layer = 50000;
    int max_count = 0;
    int many_square_times = 0;
    for (int t = 0; t < num_layer; t++)
    {
        if (layout.square_count[t] > 50000)
        {
            many_square_times++;
            if (layout.square_count[t] > first_maxcount_square_layer)
            {
                first_maxcount_square_layer = layout.square_count[t];
                first_index_maxcount_square_layer = t;
            }
            else if (layout.square_count[t] > second_maxcount_square_layer)
            {
                second_maxcount_square_layer = layout.square_count[t];
                second_index_maxcount_square_layer = t;
            }
        }
    }
    // gettimeofday(&end0, NULL);
    // duration0 = (end0.tv_sec - start0.tv_sec) + (end0.tv_usec - start0.tv_usec) / 1000000.0;
    // std::cout << "readLayout and qms Time: " << duration0 << " seconds" << std::endl;
    while (1)
    {
        int pattern_label = 0;
        // struct timeval start1, end1;
        // gettimeofday(&start1, NULL);
        auto checker = read.readPattern(pattern_label);
        // 对于内部只有矩形的情况，选长方形；
        auto filter = checker.getFilter(); // 得到该pattern的filter
        // 当不存在NOPOINTFilter时，如果出现POLYPATH的特征是正方形，且Layout中出现该方形的数量大于20w时选择跳过该层。
        int NO_point_layer_count = 0;
        for (int t = 0; t < num_layer; t++)
        {
            if (filter[t]->filterType() == "NOPOINTFIler")
                NO_point_layer_count++;
        }
        std::vector<Onelayer_FinalResult> layers_result;
        int continue_times = 0;
        // gettimeofday(&end1, NULL);
        // double durationreadpattern = (end1.tv_sec - start1.tv_sec) + (end1.tv_usec - start1.tv_usec) / 1000000.0;
        // std::cout << "readLayout and qms Time: " << durationreadpattern << " seconds" << std::endl;
        // duration1 += durationreadpattern;
        for (int i = 0; i < num_layer; i++)
        {
            std::pair<std::vector<Marker>, std::vector<MarkerType>> potential_markers; // 一层的potentialmarker
            Onelayer_FinalResult final_result;                                         // 一层完全匹配的marker
            if (filter[i]->filterType() == "NOPOINTFIler")
            {
                // std::cout << "have NOPOINTFIler" << std::endl;
                continue_times++;
                continue;
            }
            else if (NO_point_layer_count == 0 && filter[i]->get_filter_pointcount() == 4 && (i == first_index_maxcount_square_layer || i == second_index_maxcount_square_layer))
            {
                // std::cout << "have many_square_filter" << std::endl;
                continue_times++;
                continue;
            }
            else if (NO_point_layer_count == 1 && filter[i]->get_filter_pointcount() == 4 && i == first_index_maxcount_square_layer)
            {
                // std::cout << "have many_square_filter" << std::endl;
                continue_times++;
                continue;
            }
            int layer_label = i + 1;
            // struct timeval start2, end2;
            // gettimeofday(&start2, NULL);
            filter[i]->getPotentialMarkers_parallel(layout, layer_label, potential_markers);
            // gettimeofday(&end2, NULL);
            // // 计算程序运行时间
            // double durationgetpotential = (end2.tv_sec - start2.tv_sec) + (end2.tv_usec - start2.tv_usec) / 1000000.0;
            // // 输出运行时间
            // std::cout << "GetPotential Time: " << durationgetpotential << " seconds" << std::endl;
            // duration2 += durationgetpotential;
            // std::cout << "potential_markers.size:" << potential_markers.first.size() << std::endl;
            // struct timeval start3, end3;
            // gettimeofday(&start3, NULL);
            PatternIds query_result; // PatternIds = std::vector<std::vector<PolygonId>>;一个元素代表一个marker对应的polys，PolygonId存的是这一层中第几个poly。
            for (auto &search_box : potential_markers.first)
            {
                query_result.push_back(qms.query(search_box, layer_label));
            }
            // gettimeofday(&end3, NULL);
            // double durationquery = (end3.tv_sec - start3.tv_sec) + (end3.tv_usec - start3.tv_usec) / 1000000.0;
            // // 输出运行时间
            // std::cout << "query Time: " << durationquery << " seconds" << std::endl;
            // duration3 += durationquery;
            // struct timeval start4, end4;
            // gettimeofday(&start4, NULL);
            std::mutex mtx;
            tbb::parallel_for(tbb::blocked_range<size_t>(0, query_result.size()),
                              [&](tbb::blocked_range<size_t> r)
                              {
                                  std::vector<Marker> local_markers;
                                  std::vector<MarkerType> local_markertypes;
                                  for (size_t m = r.begin(); m < r.end(); m++)
                                  {
                                      auto &ptid = query_result[m]; // marker相交的poly
                                      if (ptid.empty())
                                      {
                                          assert(false && " marker have no poly");
                                      }
                                      auto &marker = potential_markers.first[m];
                                      auto &type = potential_markers.second[m];
                                      Paths64 Xor;
                                      if (checker.check(layout.getPattern(ptid, layer_label), marker, type, layer_label, Xor))
                                      {
                                          if (Xor.empty())
                                          {
                                              local_markers.push_back(marker);
                                              local_markertypes.push_back(type);
                                          }
                                          else if (Xor.size() == 1)
                                          {
                                              if (Xor[0].size() % 2 == 1)
                                              {
                                                  local_markers.push_back(marker);
                                                  local_markertypes.push_back(type);
                                              }
                                              else if (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))
                                              {
                                                  local_markers.push_back(marker);
                                                  local_markertypes.push_back(type);
                                              }
                                          }
                                          else if (Xor.size() == 2)
                                          {
                                              bool flag1 = false;
                                              bool flag2 = false;
                                              if (Xor[0].size() % 2 == 1 || (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1)))
                                                  flag1 = true;
                                              if (Xor[1].size() % 2 == 1 || (Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1)))
                                                  flag2 = true;
                                              if (flag1 && flag2)
                                              {
                                                  local_markers.push_back(marker);
                                                  local_markertypes.push_back(type);
                                              }
                                          }
                                      }
                                  }
                                  std::lock_guard<std::mutex> lock(mtx);
                                  final_result.markers.insert(final_result.markers.end(), local_markers.begin(), local_markers.end());
                                  final_result.markerTypes.insert(final_result.markerTypes.end(), local_markertypes.begin(), local_markertypes.end());
                              });
            // std::cout <<"final_result.size:"<<final_result.first.size()<<std::endl;
            final_result.layer_label = layer_label;
            layers_result.push_back(final_result);
            // gettimeofday(&end4, NULL);
            // // 计算程序运行时间
            // double durationXor = (end4.tv_sec - start4.tv_sec) + (end4.tv_usec - start4.tv_usec) / 1000000.0;
            // // XO运行时间
            // std::cout << "XO Time: " << durationXor << " seconds" << std::endl;
            // duration4 += durationXor;
        }
        int times = 3 - continue_times;
        std::vector<std::tuple<Marker, MarkerType, std::vector<int>>> potentail_layer_result = newfindFrequentMarkers(layers_result, times, num_layer);
        std::vector<std::pair<Marker, std::vector<Paths64>>> Xor_res;
        // struct timeval start5, end5;
        // gettimeofday(&start5, NULL);
        std::mutex mtx;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, potentail_layer_result.size()),
                          [&](tbb::blocked_range<size_t> r)
                          {
                              std::vector<std::pair<Marker, std::vector<Paths64>>> local_xor;
                              for (size_t i = r.begin(); i < r.end(); i++)
                              {
                                  auto &one_result = potentail_layer_result[i];
                                  auto &final_marker = std::get<0>(one_result);
                                  auto &final_markertype = std::get<1>(one_result);
                                  auto &have_check_labels = std::get<2>(one_result);
                                  std::vector<Paths64> one_marker_res;
                                  int one_marker_xorempty_layer_count = 0;
                                  for (int j = 0; j < num_layer; j++)
                                  {
                                      int layer_label = j + 1;
                                      if (std::find(have_check_labels.begin(), have_check_labels.end(), layer_label) != have_check_labels.end())
                                      {
                                          one_marker_xorempty_layer_count++;
                                          Paths64 empty_Xor;
                                          one_marker_res.push_back(empty_Xor);
                                          continue;
                                      }
                                      PatternId query_one_marker_patternID = qms.query(final_marker, layer_label);
                                      std::vector<Polygon *> query_one_marker_polygons = layout.getPattern(query_one_marker_patternID, layer_label);
                                    //   std::vector<Polygon> query_one_marker_polygons = read.check_readPolygons(layout, query_one_marker_patternID, layer_label);
                                      auto Xor = checker.final_check(query_one_marker_polygons, final_marker, final_markertype, j + 1);
                                      if (times != 3)
                                      {
                                          if (Xor.empty())
                                          {
                                              one_marker_xorempty_layer_count++;
                                          }
                                          if (Xor.size() == 1)
                                          {
                                              if (Xor[0].size() % 2 == 1)
                                              {
                                                  one_marker_xorempty_layer_count++;
                                              }
                                              else if (Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))
                                              {
                                                  one_marker_xorempty_layer_count++;
                                              }
                                          }
                                          else if (Xor.size() == 2)
                                          {
                                              bool flag1 = false;
                                              bool flag2 = false;
                                              if ((Xor[0].size() % 2 == 1) || ((Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))))
                                                  flag1 = true;
                                              if ((Xor[1].size() % 2 == 1) || ((Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1))))
                                                  flag2 = true;
                                              if (flag1 && flag2)
                                              {
                                                  one_marker_xorempty_layer_count++;
                                              }
                                              else if ((!flag1) && flag2)
                                              {
                                                  Xor.erase(Xor.begin() + 1);
                                              }
                                              else if (flag1 && (!flag2))
                                              {
                                                  Xor.erase(Xor.begin());
                                              }
                                          }
                                      }
                                      else
                                      {
                                          if (Xor.empty())
                                              one_marker_xorempty_layer_count++;
                                          else if (Xor.size() == 2)
                                          {
                                              bool flag1 = false;
                                              bool flag2 = false;
                                              if ((Xor[0].size() % 2 == 1) || ((Xor[0].size() == 4 && ((abs(Xor[0][0].x - Xor[0][2].x) == 1) || abs(Xor[0][0].y - Xor[0][2].y) == 1))))
                                                  flag1 = true;
                                              if ((Xor[1].size() % 2 == 1) || ((Xor[1].size() == 4 && ((abs(Xor[1][0].x - Xor[1][2].x) == 1) || abs(Xor[1][0].y - Xor[1][2].y) == 1))))
                                                  flag2 = true;
                                              if (flag1 && flag2)
                                              {
                                                  one_marker_xorempty_layer_count++;
                                              }
                                              else if ((!flag1) && flag2)
                                              {
                                                  Xor.erase(Xor.begin() + 1);
                                              }
                                              else if (flag1 && (!flag2))
                                              {
                                                  Xor.erase(Xor.begin());
                                              }
                                          }
                                      }
                                      one_marker_res.push_back(Xor);
                                  }
                                  if (one_marker_xorempty_layer_count >= 3 && one_marker_xorempty_layer_count < num_layer)
                                      local_xor.push_back(std::pair(final_marker, one_marker_res));
                              }
                              Xor_res.insert(Xor_res.end(), local_xor.begin(), local_xor.end());
                          });
        // gettimeofday(&end5, NULL);
        // // 计算程序运行时间
        // double durationfinal = (end5.tv_sec - start5.tv_sec) + (end5.tv_usec - start5.tv_usec) / 1000000.0;
        // // XO运行时间
        // std::cout << "final Time: " << durationfinal << " seconds" << std::endl;
        // duration5 += durationfinal;
        writeXorResToFile(Xor_res, g_config.output_path, pattern_label);
        if (read.readPatternchar() != 'p')
            break;
    }
    // std::cout << "readLayout_and qms times: " << duration0 << " seconds" << std::endl;
    // std::cout << "readpattern and getfilter times: " << duration1 << " seconds" << std::endl;
    // std::cout << "GetPotential Time: " << duration2 << " seconds" << std::endl;
    // std::cout << "query Time:" << duration3 << " seconds" << std::endl;
    // std::cout << "XO Time:" << duration4 << " seconds" << std::endl;
    // std::cout << "final Time:" << duration5 << " seconds" << std::endl;
}