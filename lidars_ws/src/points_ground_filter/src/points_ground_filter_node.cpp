#include "points_ground_filter_core.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_ground_filter");
    
    ros::NodeHandle nh("~");
    
    omp_set_num_threads(4);
    
    PointsGroundFilter pgf(nh);
    
    return 0;
}
