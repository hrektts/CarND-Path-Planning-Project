#include "jmt.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include "Eigen-3.3/Eigen/Dense"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> Jmt(vector<double> start, vector<double> end, double t) {
    /*
      Calculate the Jerk Minimizing Trajectory that connects the initial state
      to the final state in time T.

      INPUTS

      start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

      end   - the desired end state for vehicle. Like "start" this is a
      length three array.

      t     - The duration, in seconds, over which this maneuver should occur.

      OUTPUT
      an array of length 6, each value corresponding to a coefficent in the polynomial
      s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

      EXAMPLE

      > Jmt( [0, 10, 0], [10, 10, 0], 1)
      [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    MatrixXd M_t(3, 3);
    M_t << t3, t4, t5,
            3*t2, 4*t3, 5*t4,
            6*t, 12*t2, 20*t3;

    VectorXd v(3);
    v << end[0] - (start[0] + start[1]*t + 0.5*start[2]*t2),
            end[1] - (start[1] + start[2]*t),
            end[2] - start[2];
    VectorXd a = M_t.inverse() * v;
    return {start[0], start[1], 0.5*start[2], a[0], a[1], a[2]};
}
