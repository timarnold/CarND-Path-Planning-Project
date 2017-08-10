#include "planner.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> Planner::JMT(vector<double> start, vector<double> end, double T) const
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    double si = start[0];
    double sid = start[1];
    double sidd = start[2];
    double sf = end[0];
    double sfd = end[1];
    double sfdd = end[2];

    MatrixXd A(3, 3);
    A <<     powf(T, 3),      powf(T, 4),      powf(T, 5),
         3 * powf(T, 2),  4 * powf(T, 3),  5 * powf(T, 4),
                  6 * T, 12 * powf(T, 2), 20 * powf(T, 3);

    VectorXd b(3);
    b[0] = sf - (si + sid * T + 0.5 * sidd * powf(T, 2));
    b[1] = sfd - (sid + sidd * T);
    b[2] = sfdd - sidd;

    VectorXd x = A.lu().solve(b);
    return { si, sid, 0.5 * sidd, x[0], x[1], x[2] };
}

