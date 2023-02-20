#include "kalman.hpp"

class Object
{
public:
    double x0;
    double y0;
    double z0;
    double l;
    double w;
    double h;
    double phi;
    bool has_orientation;

    double xref;
    double yref;
    double vx;
    double vy;

    double number;
    double color_r;
    double color_g;
    double color_b;

    KalmanFilter4D tracker;
    int tracker_blind_update;

    KalmanFilter2D tracker_x0;
    KalmanFilter2D tracker_y0;
    KalmanFilter2D tracker_z0;
    KalmanFilter2D tracker_l;
    KalmanFilter2D tracker_w;
    KalmanFilter2D tracker_h;
    KalmanFilter2D tracker_phi;
};
