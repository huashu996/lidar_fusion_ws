std_msgs/Header header
string ns # Namespace to place this obstacle in
int32 id # Unique number for every obstacle in the same namespace

string label # Class name of obstacle

geometry_msgs/Pose pose
geometry_msgs/Vector3 scale

bool v_validity # If v_validity is false, vx & vy & vz are not credible.
float64 vx
float64 vy
float64 vz

bool a_validity # If a_validity is false, ax & ay & az are not credible.
float64 ax
float64 ay
float64 az


