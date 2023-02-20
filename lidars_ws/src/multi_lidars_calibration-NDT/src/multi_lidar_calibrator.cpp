#include "multi_lidar_calibrator.h"

void ROSMultiLidarCalibratorApp::PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr)
{
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
	cloud_msg.header.frame_id = parent_frame_;
	in_publisher.publish(cloud_msg);
}


void ROSMultiLidarCalibratorApp::MatrixToTranform(Eigen::Matrix4f & matrix, tf::Transform & trans){
    tf::Vector3 origin;
    origin.setValue(static_cast<double>(matrix(0,3)),static_cast<double>(matrix(1,3)),static_cast<double>(matrix(2,3)));

    tf::Matrix3x3 tf3d;
    tf3d.setValue(static_cast<double>(matrix(0,0)), static_cast<double>(matrix(0,1)), static_cast<double>(matrix(0,2)),
    static_cast<double>(matrix(1,0)), static_cast<double>(matrix(1,1)), static_cast<double>(matrix(1,2)),
    static_cast<double>(matrix(2,0)), static_cast<double>(matrix(2,1)), static_cast<double>(matrix(2,2)));

    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    trans.setOrigin(origin);
    trans.setRotation(tfqt);
}

void ROSMultiLidarCalibratorApp::PerformNdtOptimize(){

    if (in_parent_cloud_== nullptr || in_child_cloud_== nullptr){
        return;
    }

    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;

    ndt.setTransformationEpsilon(ndt_epsilon_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setResolution(ndt_resolution_);

    ndt.setMaximumIterations(ndt_iterations_);

    //滤波后的点云
    ndt.setInputSource(in_child_filtered_cloud_);
    //配准目标点云
    ndt.setInputTarget(in_parent_cloud_);

    pcl::PointCloud<PointT>::Ptr output_cloud(new pcl::PointCloud<PointT>);

    //如果当前变换为单位矩阵,即无初始估计,则从文件中读取的构建齐次变换矩阵
    if(current_guess_ == Eigen::Matrix4f::Identity())
    {
        //三维空间表示变换矩阵,即旋转与平移
        //初始平移
        Eigen::Translation3f init_translation(transfer_map_[points_child_topic_str][0],
                transfer_map_[points_child_topic_str][1], transfer_map_[points_child_topic_str][2]);
        //初始旋转
        Eigen::AngleAxisf init_rotation_x(transfer_map_[points_child_topic_str][5], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation_y(transfer_map_[points_child_topic_str][4], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation_z(transfer_map_[points_child_topic_str][3], Eigen::Vector3f::UnitZ());

        Eigen::Matrix4f init_guess_ = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

        current_guess_ = init_guess_;
    }
    //采用current_guess_初始估计,来初步对齐点云
    ndt.align(*output_cloud, current_guess_);

    std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << " prob:" << ndt.getTransformationProbability() << std::endl;
    std::cout << "transformation from " << child_frame_ << " to " << parent_frame_ << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    //
    pcl::transformPointCloud (*in_child_cloud_, *output_cloud, ndt.getFinalTransformation());
    
    //获取最终的配准的转化矩阵，即原始点云到目标点云的刚体变换，返回Matrix4数据类型，该数据类型采用了另一个专门用于矩阵计算的开源c++库eigen
    current_guess_ = ndt.getFinalTransformation();
    
    //欧式变换矩阵提取旋转矩阵  0,0是指从矩阵的第0行第0列位置开始,取3×3列,是旋转矩阵
    Eigen::Matrix3f rotation_matrix = current_guess_.block(0,0,3,3);
    //欧式变换矩阵提取平移矩阵
    Eigen::Vector3f translation_vector = current_guess_.block(0,3,3,1);

    std::cout << "This transformation can be replicated using:" << std::endl;
    std::cout << "rosrun tf static_transform_publisher " << translation_vector.transpose()
              << " " << rotation_matrix.eulerAngles(2,1,0).transpose() << " /" << parent_frame_
              << " /" << child_frame_ << " 10" << std::endl;

    std::cout << "Corresponding transformation matrix:" << std::endl
              << std::endl << current_guess_ << std::endl << std::endl;


    PublishCloud(calibrated_cloud_publisher_, output_cloud);

    tf::Transform t_transform;
    MatrixToTranform(current_guess_,t_transform);
    //发布坐标变换关系
    tf_br.sendTransform(tf::StampedTransform(t_transform, ros::Time::now(), parent_frame_, child_frame_));
}

void ROSMultiLidarCalibratorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_parent_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_child_cloud_msg)
{
    pcl::PointCloud<PointT>::Ptr parent_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr child_cloud (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr child_filtered_cloud (new pcl::PointCloud<PointT>);

	pcl::fromROSMsg(*in_parent_cloud_msg, *parent_cloud);
	pcl::fromROSMsg(*in_child_cloud_msg, *child_cloud);

	parent_frame_ = in_parent_cloud_msg->header.frame_id;
	child_frame_ = in_child_cloud_msg->header.frame_id;

	DownsampleCloud(child_cloud, child_filtered_cloud, voxel_size_);
	in_parent_cloud_ = parent_cloud;
	in_child_cloud_ = child_cloud;
	
	//子雷达降采样
    in_child_filtered_cloud_ = child_filtered_cloud;
}

void ROSMultiLidarCalibratorApp::DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr,
                                                 pcl::PointCloud<PointT>::Ptr out_cloud_ptr,
                                                 double in_leaf_size)
{
	pcl::VoxelGrid<PointT> voxelized;
	voxelized.setInputCloud(in_cloud_ptr);
	voxelized.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
	voxelized.filter(*out_cloud_ptr);
}


void ROSMultiLidarCalibratorApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params

	std::string initial_pose_topic_str = "/initialpose";
	std::string calibrated_points_topic_str;
	in_private_handle.param<std::string>("points_calibrated", calibrated_points_topic_str, " ");

	// x, y, z, yaw, pitch, roll
    std::string init_file_path;
    in_private_handle.param<std::string>("init_params_file_path", init_file_path, " ");
    //打开文件,将对应文件存入流中
    std::ifstream ifs(init_file_path);
    //每输出一次,文件中对应的往后移动一次,每次以空格和回车作为本次读取输出截止符
    ifs>>child_topic_num_;

    for (int j = 0; j < child_topic_num_; ++j) {
        std::string child_name;
        ifs>>child_name;
        std::vector<double> tmp_transfer;
        for (int k = 0; k < 6; ++k) {
            // read xyzypr
            double tmp_xyzypr;
            ifs>>tmp_xyzypr;
            tmp_transfer.push_back(tmp_xyzypr);
        }
        //多个雷达时,分别存取
        transfer_map_.insert(std::pair<std::string, std::vector<double>>(child_name, tmp_transfer));
    }
    
    //主坐标系
	in_private_handle.param<std::string>("points_parent_src", points_parent_topic_str, "points_raw");
	ROS_INFO("[%s] points_parent_src: %s",__APP_NAME__, points_parent_topic_str.c_str());
    //子坐标系
	in_private_handle.param<std::string>("points_child_src", points_child_topic_str, "points_raw");
	ROS_INFO("[%s] points_child_src: %s",__APP_NAME__, points_child_topic_str.c_str());
    
    //降采样尺度,降采样加速计算
	in_private_handle.param<double>("voxel_size", voxel_size_, 0.1);
	ROS_INFO("[%s] ndt_epsilon: %.2f",__APP_NAME__, voxel_size_);

	in_private_handle.param<double>("ndt_epsilon", ndt_epsilon_, 0.01);
	ROS_INFO("[%s] voxel_size: %.2f",__APP_NAME__, ndt_epsilon_);

	in_private_handle.param<double>("ndt_step_size", ndt_step_size_, 0.1);
	ROS_INFO("[%s] ndt_step_size: %.2f",__APP_NAME__, ndt_step_size_);

    //⽬标点云的ND体素的尺⼨，单位为⽶
	in_private_handle.param<double>("ndt_resolution", ndt_resolution_, 1.0);
	ROS_INFO("[%s] ndt_resolution: %.2f",__APP_NAME__, ndt_resolution_);

    //使⽤⽜顿法优化的迭代次数，迭代次数越多计算量越⼤
	in_private_handle.param<int>("ndt_iterations", ndt_iterations_, 400);
	ROS_INFO("[%s] ndt_iterations: %d",__APP_NAME__, ndt_iterations_);

	//generate subscribers and synchronizer
	cloud_parent_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_parent_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_parent_topic_str.c_str());

	cloud_child_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_child_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_child_topic_str.c_str());

	calibrated_cloud_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(calibrated_points_topic_str, 1);
	ROS_INFO("[%s] Publishing PointCloud to... %s",__APP_NAME__, calibrated_points_topic_str.c_str());

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_parent_subscriber_,
			                                               *cloud_child_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&ROSMultiLidarCalibratorApp::PointsCallback, this, _1, _2));

}

void ROSMultiLidarCalibratorApp::Run()
{
	ros::NodeHandle private_node_handle("~");

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::Rate loop_rate(10);
	while (ros::ok()){

	    ros::spinOnce();

	    // start NDT process here
        PerformNdtOptimize();

        loop_rate.sleep();
	}

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

ROSMultiLidarCalibratorApp::ROSMultiLidarCalibratorApp()
{
	//initialpose_quaternion_ = tf::Quaternion::getIdentity();
	current_guess_ = Eigen::Matrix4f::Identity();
}
