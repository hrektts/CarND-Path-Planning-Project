#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <deque>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <set>
#include "road.h"

using std::deque;
using std::map;
using std::ostringstream;
using std::string;
using std::vector;

class Vehicle {
 public:
    int line_kept;

    double check_range_front = 10;

    double check_range_rear = 10;

    double v_ref;

    double v_max;

    double a_max;

    double s_to_check;

    string state;

    size_t history_size;

    deque<vector<double> > states;  // { x, y, s, d, v, psi }

    vector<double> previous_path_x;

    vector<double> previous_path_y;

    std::set<string> possible_states{"CS", "KL", "LCL", "LCR", "PLCL", "PLCR"};

    /**
     * Constructor
     */
    Vehicle(double v_ref, double v_max, double a_max, size_t history_size);

    /**
     * Destructor
     */
    virtual ~Vehicle();

    double X() const {
        return states.back()[0];
    }

    double Y() const {
        return states.back()[1];
    }

    double S() const {
        return states.back()[2];
    }

    double D() const {
        return states.back()[3];
    }

    double V() const {
        return states.back()[4];
    }

    double A() const {
        if (states.size() > 1) {
            return V() - states.end()[-2][4];
        } else {
            return 0.;
        }
    }

    double Psi() const {
        return states.back()[5];
    }

    vector<double> LeaderState(const vector<vector<double> > &others,
                               const Road &road,
                               int lane) const;

    vector<double> FollowerState(const vector<vector<double> > &others,
                                 const Road &road,
                                 int lant) const;

    double PreferredBuffer() const;

    bool RequireAttention(const vector<double> &other_state) const;

    vector<vector<double> > CalcTrajectory(const vector<vector<double> > &others,
                                           const Road &road);

    double CalcCost(const vector<vector<double> > &trajectory,
                    const vector<double> &leader,
                    const vector<double> &follower,
                    const Road &road,
                    int lane);

    vector<vector<double> > Trajectory(double ref_vel,
                                       const Road &road,
                                       int lane);

    void Update(const vector<double> &state,
                const vector<double> &previous_path_x,
                const vector<double> &previous_path_y);
};

#endif  // VEHICLE_H_
