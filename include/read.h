#ifndef PATTERNMATCH_READER_H
#define PATTERNMATCH_READER_H

#include <vector>
#include <functional>
#include <fstream>
#include <mutex>

#include "mio.hpp"
#include "layout.h"
#include "filter.h"
#include "check.h"

class Reader
{
    using File = mio::mmap_source;

public:
    size_t pattern_cur_off = 0;
    size_t layout_cur_off = 0;
    File layout_file_;
    File pattern_file_;
    int num_layer = 0;
    Reader(const std::string &pattern_path, const std::string &layout_path)
    {
        layout_file_ = File(layout_path);
        pattern_file_ = File(pattern_path);
    }

    Checker readPattern(int &pattern_label); // 读入模板文件，取消静态函数，因为cur_off会改变
    void read_file_chunk(const std::string &filename, size_t start_pos, size_t chunk_size, std::vector<char> &buffer);
    char readPatternchar();
    // static Checker readPattern(const std::string &pattern_path); // 读入模板文件
    // void readLayout_layer(Layout &layout, File &file, Filter &filter, std::pair<std::vector<Marker>, std::vector<MarkerType>> &potential_markers, int &layer_label); // 读入版图文件
    void readLayout(Layout &layout);
    //  static Layout readLayout(const std::string &layout_path, Filter &filter); // 读入版图文件
    std::unordered_map<PolygonId, Polygon> readPolygons(const File &layout_file, std::vector<PolygonId> &pids);
    // static std::unordered_map<PolygonId, Polygon> readPolygons(const File &layout_file, std::vector<PolygonId> &pids);
    std::vector<Polygon> check_readPolygons(Layout &layout, std::vector<PolygonId> &query_one_marker_patternID, int layer_label);
    // static std::vector<BoundingBox> readMarkers(const std::string &file_path);
private:
    inline Vertex readVertex(const File &file, size_t &cur_off);
};

#endif // PATTERNMATCH_READER_H
