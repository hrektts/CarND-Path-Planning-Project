#ifndef ROAD_H_
#define ROAD_H_

#include <vector>

using std::vector;

class Road {
 public:
    int num_lane;
    double lane_width;
    vector<double> map_xs;
    vector<double> map_ys;
    vector<double> map_ss;
    vector<double> map_dxs;
    vector<double> map_dys;

    Road(int num_lane,
         double lane_width,
         const vector<double> &map_xs,
         const vector<double> &map_ys,
         const vector<double> &map_ss,
         const vector<double> &map_dxs,
         const vector<double> &map_dys);
    ~Road();
    int ClosestWaypoint(double x, double y);
    int DToLane(double d) const;
    double LaneToD(int lane) const;
    vector<double> GetFrenet(double x, double y, double theta);
    vector<double> GetXY(double s, double d) const;
    int NextWaypoint(double x, double y, double theta);
};

#endif  // ROAD_H_
