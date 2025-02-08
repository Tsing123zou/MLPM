#include "read.h"
#include <iostream>

Checker Reader::readPattern(int &pattern_label)
{
    //  每次read是读一个pattern，分成了许多层。
    const auto &getChar = [&]()
    {
        return pattern_file_[pattern_cur_off++];
    };
    const auto &checkpattern = [&]()
    {
        const char *str = "pattern";
        assert(strncmp(str, (const char *)&pattern_file_[pattern_cur_off], 7) == 0);
        // std::cout << "start_read_one_pattern" << std::endl;
        pattern_cur_off += 7;
        while (1)
        {
            char a = getChar();
            if (a != ':')
                pattern_label = 10 * pattern_label + a - '0';
            else
            {
                while (getChar() != '\n')
                    ;
                break;
            }
        }
    };
    const auto &checklayer = [&]()
    {
        const char *str = "layer";
        assert(strncmp(str, (const char *)&pattern_file_[pattern_cur_off], 5) == 0);
        pattern_cur_off += 5;
        int label_layer = 0;
        while (1)
        {
            char a = getChar();
            if (a != ':')
                label_layer = 10 * label_layer + a - '0';
            else
            {
                // std::cout << getChar() << std::endl;
                // assert(getChar()=='\n');
                while (getChar() != '\n')
                    ;
                // assert(getChar()=='\r');
                // assert(getChar()=='\n');
                break;
            }
        }
        return label_layer;
    };

    const auto &checkmarker = [&]()
    {
        const char *str = "marker:";
        assert(strncmp(str, (const char *)&pattern_file_[pattern_cur_off], 7) == 0);
        pattern_cur_off += 7;
        // assert(getChar()=='\r');
        // assert(getChar()=='\n');
        while (getChar() != '\n')
            ;
    };

    const auto &readPolygon = [&](PatternPoly &ret)
    {
        for (char end = ','; end == ','; end = getChar())
        {
            auto tv = readVertex(pattern_file_, pattern_cur_off);
            ret.push_back(tv);
            assert(end == ',' || end == '\r' || end == '\n');
        }

        if (pattern_file_[pattern_cur_off] == '\n')
            assert(getChar() == '\n');
    };
    const auto &readMarker = [&]()
    {
        Marker marker{};
        for (char end = ','; end == ','; end = getChar())
        {
            auto tv = readVertex(pattern_file_, pattern_cur_off);
            marker.update(tv);
            assert(end == ',' || end == '\r' || end == '\n');
        }
        if (pattern_file_[pattern_cur_off] == '\n')
            assert(getChar() == '\n');
        return marker;
    };
    checkpattern();
    std::vector<std::vector<PatternPoly>> multilayer_pattern_polys;
    int now_layer = 1;
    while (pattern_file_[pattern_cur_off] == 'l')
    {
        std::vector<PatternPoly> pattern_polys;
        int layer_diff = checklayer() - now_layer;
        // std::cout << layer_diff << std::endl;
        if (layer_diff == 0)
            now_layer++;
        else
        {
            now_layer += layer_diff;
            while (layer_diff > 0)
            {
                multilayer_pattern_polys.push_back(pattern_polys);
                layer_diff--;
            }
        }
        // std::cout << file[pattern_cur_off] << std::endl;
        for (char tc = pattern_file_[pattern_cur_off]; tc == '('; tc = pattern_file_[pattern_cur_off])
        {
            // std::cout << "start_readpolygon" << std::endl;
            readPolygon(pattern_polys.emplace_back());
        }
        // std::cout << pattern_polys[0][0].x << std::endl;
        multilayer_pattern_polys.push_back(pattern_polys);
    }
    // std::cout << "have read pattern_layers" << std::endl;
    checkmarker();
    auto marker = readMarker();
    // std::cout << "have read marker" << std::endl;
    // std::cout << marker.rx << ' ' << marker.uy << std::endl;
    return {std::move(multilayer_pattern_polys), marker};
}

char Reader::readPatternchar()
{
    return pattern_file_[pattern_cur_off];
}
Vertex Reader::readVertex(const File &file, size_t &cur_off)
{
    const auto &getChar = [&]()
    { return file[cur_off++]; };
    const auto &readInteger = [&]()
    {
        // read "x," or "x)"
        vert_t ret = 0;
        int flag = 1;
        char tc = getChar();
        if (tc == '-')
        {
            flag = -1;
            tc = getChar();
        }
        do
        {
            ret *= 10;
            ret += tc - '0';
        } while (tc = getChar(), '0' <= tc && tc <= '9');
        return flag * ret;
    };

    // read "(x,x)"
    auto c = getChar();
    assert(c == '(');
    vert_t x = readInteger();
    vert_t y = readInteger();
    return Vertex{x, y};
}

void Reader::readLayout(Layout &layout)
{
    const auto &getChar = [&]()
    { return layout_file_[layout_cur_off++]; };

    const auto &checklayer = [&]()
    {
        const char *str = "layer";
        assert(strncmp(str, (const char *)&layout_file_[layout_cur_off], 5) == 0);
        layout_cur_off += 5;
        int label_layer = 0;
        while (1)
        {
            char a = getChar();
            if (a != ':')
            {
                assert(a >= '0' && a <= '9');
                label_layer = 10 * label_layer + a - '0';
            }
            else
            {
                // assert(getChar() == '\n');
                // break;
                while (getChar() != '\n')
                    ;
                break;
            }
        }
        return label_layer;
    };

    const auto &readPolygon = [&](BoundingBox &layout_layer_box, IndexBoxes &layout_layer_iboxes, int &layout_layer_poly_num, std::vector<LayoutPoly> &layout_layer_polys, int &layout_layer_square_poly_num)
    {
        // 读取一个多边形得到它的box
        IndexBox box(layout_layer_poly_num); // 存的是在文件中的偏移
        LayoutPoly ret;
        for (char end = ','; end == ','; end = getChar())
        {
            auto tv = readVertex(layout_file_, layout_cur_off);
            box.update(tv);
            Point64 point{tv.x, tv.y};
            ret.points.push_back(point);
            // readNullVertex(); // 只更新边框的话，不需要相邻的点
            assert(end == ',' || end == '\n');
        }
        ret.update();
        if (ret.point_count == 4)
            layout_layer_square_poly_num++;
        layout_layer_iboxes.push_back(box); // 存读到的box，这里仅仅读box
        layout_layer_poly_num++;
        layout_layer_box += box;
        layout_layer_polys.push_back(ret);
    };

    while (layout_file_[layout_cur_off] == 'l')
    {
        num_layer++;
        int layer_label = checklayer();
        // std::cout << "read_layer_layer_label: " << layer_label << std::endl;
        BoundingBox layout_layer_box; // 每层总的layout范围
        IndexBoxes layout_layer_iboxes;
        int layout_layer_poly_num = 0;
        int layout_layer_square_poly_num = 0;
        std::vector<LayoutPoly> layout_layer_polys;
        for (char tc = layout_file_[layout_cur_off]; tc == '('; tc = layout_file_[layout_cur_off])
        {
            readPolygon(layout_layer_box, layout_layer_iboxes, layout_layer_poly_num, layout_layer_polys, layout_layer_square_poly_num);
        }
        layout.layout_box_.push_back(layout_layer_box);
        layout.layout_iboxes.push_back(std::move(layout_layer_iboxes));
        layout.layer_poly_num.push_back(layout_layer_poly_num);
        layout.layout_polys.push_back(std::move(layout_layer_polys));
        layout.square_count.push_back(layout_layer_square_poly_num);
    }
    layout.num_layer_layout = num_layer;
}

std::unordered_map<PolygonId, Polygon> Reader::readPolygons(const File &layout_file, std::vector<PolygonId> &pids)
{
    size_t cur_off = 0;

    const auto &setFilePointer = [&](size_t offset)
    { cur_off = offset; };
    const auto &getChar = [&]()
    { return layout_file[cur_off++]; };
    const auto &readPolygon = [&]()
    {
        Polygon ret;
        for (char end = ','; end == ','; end = getChar())
        {
            auto [x, y] = readVertex(layout_file, cur_off);
            ret.emplace_back(x, y);
            assert(end == ',' || end == '\n');
        }
        return ret;
    };

    std::unordered_map<PolygonId, Polygon> ret;
    for (auto id : pids)
    {
        setFilePointer(id);
        ret.emplace(id, readPolygon());
        // 将读取到的多边形数据添加到 ret 的哈希表中，其中键是 id，值是 readPolygon() 返回的 Polygon。
    }
    return ret;
}

std::vector<Polygon> Reader::check_readPolygons(Layout &layout, std::vector<PolygonId> &query_one_marker_patternID, int layer_label)
{
    std::vector<Polygon> query_one_marker_polygons;
    for (auto id : query_one_marker_patternID)
    {
        // setFilePointer(id);
        query_one_marker_polygons.push_back(layout.layout_polys[layer_label - 1][id].points);
    }
    return query_one_marker_polygons;
}
