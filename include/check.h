#ifndef PATTERNMATCH_CHECKER_H
#define PATTERNMATCH_CHECKER_H

#include <vector>
#include "clipper.h"
#include "geometry.h"
#include "patternpoly.h"
#include "filter.h"
#include "util.h"

// using Paths64 = Clipper2Lib::Paths64;
// using Path64 = Clipper2Lib::Path64;
// using Point64 = Clipper2Lib::Point64;
// using Strings = std::vector<std::string>;
// using Polygon = Clipper2Lib::Path64; // 按点存储的多边形
// Ponit64包含了两个元素，T x，T y；
// Path64是结构体Point64的向量
// Paths64是向量Path64的向量

class Checker
{
    using Paths64 = Clipper2Lib::Paths64;
    using Path64 = Clipper2Lib::Path64;
    using Point64 = Clipper2Lib::Point64;
    using Strings = std::vector<std::string>;
    using Polygon = Clipper2Lib::Path64; // 按点存储的多边形
    enum Pattern_Layer_Poly_Type
    {
        INNER_POLY,
        INNER_PATH_POLY,
        INNER_POINT_POLY,
        INNER_NO_POINT_POLY
    };
    enum Pattern_Layer_Type
    {
        INNER_POLY_Layer,
        INNER_PATH_Layer,
        INNER_NO_POINT_Layer
    };

public:
    Checker(std::vector<std::vector<PatternPoly>> &&multilayer_pattern_polys, Marker marker); // 右值引用

    void init();

    std::vector<ptr<Filter>> getFilter();
    // Paths64 check(const std::vector<Polygon *> &polys, Marker &marker, MarkerType type, int layer_label);
    bool check(const std::vector<Polygon *> &polys, Marker &marker, MarkerType type, int &layer_label, Paths64 &Xor);
    Paths64 final_check(const std::vector<Polygon *> &polys, Marker &marker, MarkerType type, int layer_label);
    std::vector<std::vector<PatternPoly>> get_multilayer_PatternPolys() { return multilayer_pattern_polys_; }
    std::vector<PatternPoly> get_onelayer_PatternPolys(int i) { return multilayer_pattern_polys_[i]; }
    int get_layer_num()
    {
        return num_layer;
    }
    Marker getMarker()
    {
        return marker_;
    }

private:
    std::vector<Pattern_Layer_Type> multilayer_pattern_type_;        // 层类型
    std::vector<std::vector<PatternPoly>> multilayer_pattern_polys_; // 未镜像，一个元素代表一层
    std::vector<std::vector<Paths64>> multilayer_target_patterns_;   // 镜像，一个元素代表一层，一个元素有八个子元素，对应八个镜像
    Marker marker_;
    int num_layer;
};

#endif // PATTERNMATCH_CHECKER_H
