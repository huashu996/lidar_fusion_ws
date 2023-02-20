#include "points_freespace_extractor_core.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "points_freespace_extractor");
    
    ros::NodeHandle nh("~");
    
    omp_set_num_threads(4);
    
    PointsFreespaceExtractor pfe(nh);
    
    return 0;
}
