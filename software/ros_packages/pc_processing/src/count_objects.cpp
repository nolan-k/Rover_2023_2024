// pc_filter.cpp
//
// Osian Leahy
//
// This node removes the table points from the point cloud data given for HW7, leaving just the bottles.

// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

//Include the ROS2 msg for point cloud:
#include <sensor_msgs/msg/point_cloud2.hpp>
//Also include the geometry message for marker arrays.
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
//int msg for reporting the count:
#include <std_msgs/msg/int32.hpp>

//Include the pointcloud stuff (General)
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//Include the pointcloud stuff (Specific)
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

//Placeholders for callback binding:
using std::placeholders::_1;

// The idiom in C++ is the same as in Python; we create a class that
// inherits from the ROS 2 Node class.
class CountObjects : public rclcpp::Node {
public:
	
	//Simplify the syntax by abstracting this namespace
	using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Marker = visualization_msgs::msg::Marker;
    using Int32 = std_msgs::msg::Int32; 

	//(I Think)This needs to be public because it's used to construct the class
	CountObjects() : Node("count_objects") {
		
		//Subscribe to the rosbag stream
		subscriber_ = this->create_subscription<PointCloud2>("bottles", 5, std::bind(&CountObjects::sub_callback, this, _1));
		
		//Republish to:
		publisher_ = this->create_publisher<MarkerArray>("found_bottles", 10);
        //and
        count_publisher_ = this->create_publisher<Int32>("bottle_count",10);

		RCLCPP_INFO(this->get_logger(), "Starting Count");

	}

private:

	//Class variable prototypes, 2 pass compiler blah blah
	rclcpp::Publisher<MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<Int32>::SharedPtr count_publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	//Where the magic is supposed to happen...
	void sub_callback(const PointCloud2::SharedPtr msg) {
		//Log that we've recieved a msg
		//RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved");

		//First, pull in the point cloud, translate from ROS PC2 to PCL:
		//We get RGB data with this camera, as specified in the PC2 message fields
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); //Create the point cloud object
		pcl::fromROSMsg(*msg, *pcl_cloud); //pass it and the msg by reference for conversion
		
		//Create output objects:
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_object_centers(new pcl::PointCloud<pcl::PointXYZRGB>);
        int object_count = 0;
        
        //Clustering with a Kd tree and euclidian clustering
        //Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(pcl_cloud);
        //Creating the vector to store groups of cluster indices in: 
        std::vector<pcl::PointIndices> cluster_indices;
        //and the extraction class object.
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(525);
        ec.setMaxClusterSize(5000);
        //Running the search and extracting to our indices vector
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_cloud);
        ec.extract(cluster_indices);

        //Find the average point of each cluster, and also count the clusters.
        //For cluster within the vector of clusters
        for(pcl::PointIndices indices : cluster_indices){
            //Pointer shennanigans
            pcl::PointIndices::Ptr idxPtr = pcl::make_shared<pcl::PointIndices>(indices);
            
            //Extract the indices to a point cloud:
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(pcl_cloud);
            extract.setIndices(idxPtr);
            extract.setNegative(false);
            extract.filter(*object_cloud);
            
            pcl::PointXYZRGB average_point;
            //Iterate over the points w/in the cluster and get an average point:
            for(auto& point : *object_cloud){
                average_point.x += point.x;
                average_point.y += point.y;
                average_point.z += point.z;
                average_point.r += point.r;
                average_point.g += point.g;
                average_point.b += point.b;
            }
            //Normalize the avg
            float num_pts = (object_cloud->height * object_cloud->width);
            average_point.x = average_point.x/num_pts;
            average_point.y = average_point.y/num_pts;
            average_point.z = average_point.z/num_pts;
            average_point.r = average_point.r/num_pts;
            average_point.g = average_point.g/num_pts;
            average_point.b = average_point.b/num_pts;

            //Add it to pcl_object_centers:
            pcl_object_centers->push_back(average_point);

            //Add to the object count
            object_count++;
        }

        //Now we need to publish the object count and visual markers.
        //Create the count message to be published
        Int32 count_out = Int32();
        count_out.data = object_count;

        //Create a marker array to be filled in:
        std::vector<Marker> bottle_markers;

        
        //Iterate over all our points to create markers of them
        int marker_count = 0; //Probably shoudlve just done this in the same for loop above lmao
        for (pcl::PointXYZRGB center : *pcl_object_centers){
            Marker marker = Marker();
            
            //Fill in the excessive amount of marker data
            marker.header.frame_id = "/quori/head_camera_optical";
            marker.header.stamp = rclcpp::Clock().now();

            marker.ns = "/";
            marker.id = marker_count;

            marker.type = visualization_msgs::msg::Marker::SPHERE; //Just gonna do spheres because I don't wanna rotate the cylinders LOL 

            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = center.x;
            marker.pose.position.y = center.y;
            marker.pose.position.z = center.z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.08;
            marker.scale.y = 0.08;
            marker.scale.z = 0.08;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;   // Don't forget to set the alpha!

            marker.lifetime = rclcpp::Duration::from_nanoseconds(250000000);

            //Add it to the list that gets published
            bottle_markers.push_back(marker);

            marker_count++;

        }

        //Move the array into the message
        MarkerArray bottle_markers_msg;
        bottle_markers_msg.markers = bottle_markers;

		//Publish and log the results:
		this->publisher_->publish(bottle_markers_msg);
        this->count_publisher_->publish(count_out);
        std::string log_msg = std::to_string(object_count) + " Points found";
		RCLCPP_INFO(this->get_logger(), log_msg.c_str());

	}
};


// This is the entry point for the executable. 
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node instance and store a shared pointer to it.
	auto node = std::make_shared<CountObjects>();

	// Give control to ROS via the shared pointer.
	rclcpp::spin(node);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	// Main always returns 0 on success.
	return 0;
}