#ifndef PATTERNMATCH_PATTERNPOLY_H
#define PATTERNMATCH_PATTERNPOLY_H

#include "geometry.h"

#include "util.h"

class PatternPoly : public std::vector<Vertex> // 对象可以存边框和点
{
public:
    void push_back(const Vertex &vertex)
    {
        std::vector<Vertex>::push_back(vertex);
        updateBoundingBox(vertex);
    }

    void swap(PatternPoly &o)
    {
        std::vector<Vertex>::swap(o);
        std::swap(box, o.box);
    }

    BoundingBox box;

private:
    void updateBoundingBox(Vertex vert)
    {
        box.lx = std::min(box.lx, vert.x);
        box.ly = std::min(box.ly, vert.y);
        box.rx = std::max(box.rx, vert.x);
        box.uy = std::max(box.uy, vert.y);
    }
};

#endif // PATTERNMATCH_PATTERNPOLY_H