#include "check.h"
void Checker::init()
{
    const auto &align = [&]()
    {
        Offset off({marker_.lx, marker_.ly});
        // Vertex 结构体没有定义任何构造函数，而 {} 列表初始化可以用来直接初始化成员变量，而 () 则无法在没有匹配构造函数的情况下直接初始化结构体。
        for (auto &pattern_polys_ : multilayer_pattern_polys_)
        {
            for (auto &pattern_poly : pattern_polys_)
            {
                for (auto &v : pattern_poly)
                {
                    v -= off;
                }
                pattern_poly.box = pattern_poly.box - off;
            }
        }
        marker_ = marker_ - off;
    };

    if (marker_.lx != 0 || marker_.ly != 0)
        align();
}

Checker::Checker(std::vector<std::vector<PatternPoly>> &&multilayer_pattern_polys, Marker marker) : multilayer_pattern_polys_(std::move(multilayer_pattern_polys)), marker_(marker)
{
    const auto &poly2path = [&](const PatternPoly &poly)
    {
        Path64 ret;
        for (auto &[x, y] : poly)
        {
            ret.emplace_back(x, y);
        }
        return ret;
    };
    const auto &polys2Paths = [&](const std::vector<PatternPoly> &polys)
    {
        Paths64 ret;
        for (auto &poly : polys)
        {
            ret.push_back(poly2path(poly));
        }
        return ret;
    };
    const auto &modifyPaths = [&](Paths64 paths, MarkerType type)
    {
        auto mw = marker_.rx, mh = marker_.uy;
        const auto &modifyPaths = [&](const std::function<void(Point64 &)> &func)
        {
            for (auto &path : paths)
            {
                for (auto &p : path)
                {
                    func(p);
                }
            }
        };
        switch (type)
        {
        case RAW:
            break;
        case X_AXIAL:
            // std::cout<<"X_AXIAL"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p.y = mh - p.y; });
            for (auto &path : paths)
            {
                std::reverse(path.begin() + 1, path.end());
            }
            break;
        case Y_AXIAL:
            // std::cout<<"Y_AXIAL"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p.x = mw - p.x; });
            for (auto &path : paths)
                std::reverse(path.begin() + 1, path.end());
            break;
        case R180:
            // std::cout<<"R90"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p = {mw - p.x, mh - p.y}; });
            break;
        case R90CW:
            // std::cout<<"R90CW"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p = {p.y, mw - p.x}; });
            break;
        case R90CW_X_AXIAL:
            // std::cout<<"R90CW_X_AXIA"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p = {p.y, p.x}; });
            for (auto &path : paths)
                std::reverse(path.begin() + 1, path.end());
            break;
        case R90CW_Y_AXIAL:
            // std::cout<<"R90CW_Y_AXIA"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p = {mh - p.y, mw - p.x}; });
            for (auto &path : paths)
                std::reverse(path.begin() + 1, path.end());
            break;
        case R90CW_R180:
            // std::cout<<"R90CW_R180"<<std::endl;
            modifyPaths([&](Point64 &p)
                        { p = {mh - p.y, p.x}; });
            break;
        default:
            break;
        }
        return paths;
    };
    // std::cout << "start_create_check" << std::endl;
    init();
    // std::cout << "have_check_init" << std::endl;
    num_layer = multilayer_pattern_polys_.size();
    // std::cout << "num_layer:" << num_layer << std::endl;
    // std::cout << "start_get_mir" << std::endl;
    multilayer_target_patterns_.resize(num_layer); // 没有这个会发生Segmentation fault
    multilayer_pattern_type_.resize(num_layer);
    for (size_t i = 0; i < num_layer; ++i)
    {
        // std::cout<<"start_2_Paths"<<std::endl;
        auto &pattern_polys_ = multilayer_pattern_polys_[i];
        auto raw_paths = polys2Paths(pattern_polys_);
        // std::cout<<"have_2_Paths"<<std::endl;
        for (int t = 0; t < 8; t++)
        {
            multilayer_target_patterns_[i].push_back(modifyPaths(raw_paths, MarkerType(1 << t)));
            // multilayer_pattern_strings_[i].push_back(getPatternString(multilayer_target_patterns_[i].back()));
        }
    }
    // for (int t = 0; t < 8; t++)
    // {
    //     std::cout << "mir:"<< t << std::endl;
    //     for (auto &p : multilayer_target_patterns_[0][t])
    //         for (auto &v : p)
    //         {
    //             std::cout << v.x << ',' << v.y << std::endl;
    //         }
    // }
    // std::cout << "have_create_check" << std::endl;
}

std::vector<ptr<Filter>> Checker::getFilter()
{
    // 更新pattern_type的状态
    // 根据pattern_type的状态来选择filter的类型
    std::vector<ptr<Filter>> Filters;
    for (int i = 0; i < num_layer; i++)
    {
        // PatternPoly *inner_poly;
        int maxcount_inner_point = 0;
        int max_inner_point_index = 0;
        int maxcount_inner_poly = 0;
        int maxcount_inner_poly_index = 0;
        int index = 0;
        multilayer_pattern_type_[i] = INNER_NO_POINT_Layer;
        for (auto &poly : multilayer_pattern_polys_[i]) // 遍历一层的poly
        {
            int inner_point = 0;
            for (auto &point : poly)
            {
                if (marker_.contain(point))
                    inner_point++;
            }
            if (inner_point == poly.size())
            {
                if (inner_point > maxcount_inner_poly)
                {
                    maxcount_inner_poly = inner_point;
                    maxcount_inner_poly_index = index;
                }
                if (multilayer_pattern_type_[i] != INNER_POLY_Layer)
                    multilayer_pattern_type_[i] = INNER_POLY_Layer;
            }
            else if (inner_point >= 1 && inner_point < poly.size())
            {
                if (multilayer_pattern_type_[i] == INNER_NO_POINT_Layer)
                    multilayer_pattern_type_[i] = INNER_PATH_Layer;
            }
            if (inner_point > maxcount_inner_point)
            {
                maxcount_inner_point = inner_point;
                max_inner_point_index = index;
            }
            index++;
        }
        Paths64 target_poly_8;
        if (multilayer_pattern_type_[i] == INNER_POLY_Layer)
        {
            // std::cout << "new one POLYFilter" << std::endl;
            for (int j = 0; j < 8; j++)
            {
                target_poly_8.push_back(multilayer_target_patterns_[i][j][maxcount_inner_poly_index]);
            }
            Filters.push_back(ptr<Filter>(new POLYFilter(target_poly_8, marker_, i + 1)));
            // std::cout << "have new one POLYFilter" << std::endl;
        }
        else if (multilayer_pattern_type_[i] == INNER_PATH_Layer)
        {
            // std::cout << "new one PATHFIlter" << std::endl;
            for (int j = 0; j < 8; j++)
            {
                target_poly_8.push_back(multilayer_target_patterns_[i][j][max_inner_point_index]);
            }
            Filters.push_back(ptr<Filter>(new PATHFilter(target_poly_8, marker_, i + 1)));
            // std::cout << "have new one PATHFIlter" << std::endl;
        }
        else
        {
            Filters.push_back(ptr<Filter>(new NOPOINTFIler()));
            // std::cout << "have new one NOPointFIlter" << std::endl;
        }
    }
    return Filters;
}

bool Checker::check(const std::vector<Polygon *> &polys, Marker &marker, MarkerType type, int &layer_label, Paths64 &Xor_res)
{
    using namespace Clipper2Lib;
    const auto &align = [](Paths64 &subjects, Rect64 &clip)
    {
        // 矩形的左上角平移到了坐标原点，subjects中多边形也进行相应的偏移
        auto ox = clip.left, oy = clip.top;
        clip.left -= ox;
        clip.right -= ox;
        clip.top -= oy;
        clip.bottom -= oy;
        for (auto &path : subjects)
        {
            for (auto &p : path)
            {
                p.x -= ox;
                p.y -= oy;
            }
        }
    };
    const auto &getPaths = [](const std::vector<Polygon *> &polys)
    {
        Paths64 ret;
        for (auto *poly : polys)
            ret.push_back(*poly);
        return ret;
    };
    const auto &box2Rect = [](BoundingBox box)
    {
        auto [lx, ly, rx, uy] = box;
        return Rect64(lx, ly, rx, uy);
    };
    const auto &type2idx = [&](int type)
    {
        int idx = 0;
        while (type >>= 1)
            idx++;
        return idx;
    };

    Paths64 subjects = getPaths(polys);
    auto clip = box2Rect(marker);
    align(subjects, clip);

    Paths64 potential_pattern = RectClip(clip, subjects); // 对subjects进行裁剪，取与clip重叠的部分
    // if(marker.lx==3004633 && marker.ly == 2919498 && marker.rx == 3018433 && marker.uy == 2929698 && layer_label == 6){
    //     for(int i = 0; i<potential_pattern.size();i++){
    //         for(int j =0 ;j<potential_pattern[i].size();j++){
    //             std::cout <<"(" << potential_pattern[i][j].x << ',' << potential_pattern[i][j].y <<"),";
    //         }
    //         std::cout << std::endl;
    //     }
    // }
    // if(marker.lx==2328433 && marker.ly == 2276898 && marker.rx == 2342233 && marker.uy == 2287098 && layer_label == 6){
    //     for(int i = 0; i<potential_pattern.size();i++){
    //         for(int j =0 ;j<potential_pattern[i].size();j++){
    //             std::cout <<"(" << potential_pattern[i][j].x << ',' << potential_pattern[i][j].y <<"),";
    //         }
    //         std::cout << std::endl;
    //     }
    // }
    Paths64 Xor_in;
    for(int i = 0; i < potential_pattern.size(); i++){
        if(potential_pattern[i].size() >= 4){
            Xor_in.push_back(potential_pattern[i]);
        }
    }
    auto idx = type2idx(type);
    if (Xor_in.size() > multilayer_target_patterns_[layer_label - 1][idx].size() + 1)
        return false;
    else
    {
        Xor_res = Xor(multilayer_target_patterns_[layer_label - 1][idx], Xor_in, FillRule::NonZero);
        return true;
    }
}

Paths64 Checker::final_check(const std::vector<Polygon *> &polys, Marker &marker, MarkerType type, int layer_label)
{
    using namespace Clipper2Lib;
    const auto &align = [](Paths64 &subjects, Rect64 &clip)
    {
        // 矩形的左下角平移到了坐标原点，subjects中多边形也进行相应的偏移
        auto ox = clip.left, oy = clip.top;
        clip.left -= ox;
        clip.right -= ox;
        clip.top -= oy;
        clip.bottom -= oy;
        for (auto &path : subjects)
        {
            for (auto &p : path)
            {
                p.x -= ox;
                p.y -= oy;
            }
        }
    };
    const auto &getPaths = [](const std::vector<Polygon *> &polys)
    {
        Paths64 ret;
        for (auto *poly : polys)
            ret.push_back(*poly);
        return ret;
    };
    const auto &box2Rect = [](BoundingBox box)
    {
        auto [lx, ly, rx, uy] = box;
        return Rect64(lx, ly, rx, uy);
    };
    const auto &type2idx = [&](int type)
    {
        int idx = 0;
        while (type >>= 1)
            idx++;
        return idx;
    };

    // Paths64 subjects = polys;
    Paths64 subjects = getPaths(polys);
    auto clip = box2Rect(marker);
    assert(marker.ly == clip.top);
    align(subjects, clip);

    Paths64 potential_pattern = RectClip(clip, subjects); // 对subjects进行裁剪，取与clip重叠的部分
    // for(int i = 0; i < subjects.size(); i++){
    //     for(int j; j < subjects[i].size();j++)
    //     {
    //         std::cout<< subjects[i][j].x << ',' << subjects[i][j].x << std::endl;
    //     }
    // }
    Paths64 Xor_in;
    for(int i = 0; i < potential_pattern.size(); i++){
        if(potential_pattern[i].size() >= 4){
            Xor_in.push_back(potential_pattern[i]);
        }
    }
    auto idx = type2idx(type);
    Paths64 xor_res = Xor(multilayer_target_patterns_[layer_label - 1][idx], Xor_in, FillRule::NonZero);
    for (auto &path : xor_res)
    {
        for (auto &point : path)
        {
            point.x += marker.lx;
            point.y += marker.ly;
        }
    }
    return xor_res;
}