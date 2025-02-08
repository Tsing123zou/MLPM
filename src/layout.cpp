#include "layout.h"
#include "read.h"

std::vector<Polygon *> Layout::getPattern(std::vector<PolygonId> &pids, int &layer_label)
{
    std::vector<Polygon *> ret;
    for (auto id : pids)
    {
        ret.push_back(&(layout_polys[layer_label - 1][id].points));
    }
    return ret;
}