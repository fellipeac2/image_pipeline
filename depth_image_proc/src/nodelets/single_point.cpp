#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <depth_image_proc/depth_traits.h>

namespace depth_image_proc {

namespace enc = sensor_msgs::image_encodings;
class SinglePointNodelet : public nodelet::Nodelet
{
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::CameraSubscriber sub_depth_;
	ros::Subscriber part_sub_;
	int queue_size_;

	boost::mutex connect_mutex_;
	typedef geometry_msgs::PointStamped Point;
	ros::Publisher pub_point_;

	image_geometry::PinholeCameraModel model_;

	float u_raw_ = -1;
	float v_raw_ = -1;

	virtual void onInit();

	void connectCb();

	void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
		     const sensor_msgs::CameraInfoConstPtr& info_msg);

	void partCb(const std_msgs::Float32MultiArray::ConstPtr& part);
};

void SinglePointNodelet::onInit()
{
	ros::NodeHandle& nh	    = getNodeHandle();
	ros::NodeHandle& private_nh = getPrivateNodeHandle();
	it_.reset(new image_transport::ImageTransport(nh));
	
	private_nh.param("queue_size", queue_size_, 5);
	ros::SubscriberStatusCallback connect_cb = boost::bind(&SinglePointNodelet::connectCb, this);

	boost::lock_guard<boost::mutex> lock(connect_mutex_);
	pub_point_ = nh.advertise<Point>("point", 1, connect_cb, connect_cb);
	part_sub_ = nh.subscribe("/part", 10, &SinglePointNodelet::partCb, this);
}

void SinglePointNodelet::connectCb()
{
	boost::lock_guard<boost::mutex> lock(connect_mutex_);

	if(pub_point_.getNumSubscribers() == 0)
	{
		sub_depth_.shutdown();
	} else if (!sub_depth_)
	{
		image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		sub_depth_ = it_->subscribeCamera("image_rect", queue_size_, &SinglePointNodelet::depthCb, this, hints);
	}
}

void SinglePointNodelet::depthCb(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	Point::Ptr point_msg(new Point);
	
	model_.fromCameraInfo(info_msg);
	if(u_raw_ < 0 || v_raw_ < 0)
		return;
	int u = (int)(u_raw_ * depth_msg->width);
	int v = (int)(v_raw_ * depth_msg->height);
	
	float center_x = model_.cx();
	float center_y = model_.cy();

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = DepthTraits<float>::toMeters( float(1) );
	float constant_x = unit_scaling / model_.fx();
	float constant_y = unit_scaling / model_.fy();
	float bad_point = std::numeric_limits<float>::quiet_NaN();
	typedef uint16_t T;
  	if (depth_msg->encoding == enc::TYPE_32FC1)
	{
		typedef float T;
	}	
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  	int row_step = depth_msg->step / sizeof(T);
	T depth = depth_row[u];
	
	float x, y, z;
	double range_max = 0.0;

	if (!DepthTraits<T>::valid(depth))
	{
		if (range_max != 0.0)
		{
			depth = DepthTraits<T>::fromMeters(range_max);
		}
		else
		{
			x = y = z = bad_point;
		}
	} else 
	{

	// Fill in XYZ
		x = (u - center_x) * depth * constant_x;
		y = (v - center_y) * depth * constant_y;
		z = DepthTraits<T>::toMeters(depth);
	}
	char buffer[50];
	sprintf(buffer, "(X, Y, Z) : (%f, %f, %f)", x, y, z);
	ROS_INFO(buffer);
}

void SinglePointNodelet::partCb(const std_msgs::Float32MultiArray::ConstPtr& part)
{
	u_raw_ = part->data[0];
	v_raw_ = part->data[1];
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::SinglePointNodelet,nodelet::Nodelet);

