
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

geometry_msgs::Vector3 received_vector0;
geometry_msgs::Vector3 received_vector1;

void magnetic_vector0_callback(const geometry_msgs::Vector3::ConstPtr& msg){

	received_vector0 = *msg;
}
void magnetic_vector1_callback(const geometry_msgs::Vector3::ConstPtr& msg){
	received_vector1 = *msg;
}
geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}
geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}
geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}
double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}

geometry_msgs::Vector3 GetMagnetometerReadings(tf::StampedTransform transform, double current_amplitude, double frequency)
{
	double x0 = (double) transform.getOrigin().x();
	double y0 = (double) transform.getOrigin().y();
	double z0 = (double) transform.getOrigin().z();
	tf::Matrix3x3 basis=transform.getBasis();
	geometry_msgs::Vector3 point;
	geometry_msgs::Vector3 v_line;
	point.x = x0;
	point.y = y0;
	point.z = z0;

	v_line.x = basis.getColumn(0).getX();
	v_line.y = basis.getColumn(0).getY();
	v_line.z = basis.getColumn(0).getZ();


	double t=-DotProduct(point,v_line);

	geometry_msgs::Vector3 closest;
	closest.x = x0 + t * v_line.x;
	closest.y = y0 + t * v_line.y;
	closest.z = z0 + t * v_line.z;
//	ROS_INFO_STREAM("t "<<t);

//	ROS_INFO_STREAM("v_line"<<v_line.x<<" "<<v_line.y<<" "<<v_line.z);
//	ROS_INFO_STREAM("point"<<point.x<<" "<<point.y<<" "<<point.z);
//	ROS_INFO_STREAM("closest"<<closest.x<<" "<<closest.y<<" "<<closest.z);

	double d=sqrt((closest.x) * (closest.x) + (closest.y) * (closest.y) + (closest.z) * (closest.z));

	geometry_msgs::Vector3 vector=CrossProduct(closest, v_line);
	ros::Time time = ros::Time::now();
	double stamp = time.sec-(double)time.nsec/1000000000;
	double b=0;
	if (d>0.01)
	{
		b = 2 * current_amplitude * 0.0001 / d * sin(2 *3.14159 * frequency * stamp);
	}
	else
	{
		b= 2 * current_amplitude * 0.0001 / 0.01 * sin(2 *3.14159 * frequency * stamp);
	}
//	ROS_INFO_STREAM("stamp "<<stamp<<" distance "<<d<<" mag strengh "<<2 * current_amplitude * 0.0001 / d);
//	ROS_INFO_STREAM("point"<<point.x<<" "<<point.y<<" "<<point.z);
	vector=normalize_vector(vector);
	vector.x = vector.x * b;
	vector.y = vector.y * b;
	vector.z = vector.z * b;

	return vector;
}
int main (int argc, char** argv){
    ros::init(argc, argv, "power_line_simulation");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string magnetic_vector0, magnetic_vector1, frame0, frame1, base_frame , magnetometer_topic;

    nh_ns.param("magnetometer_frame", frame0, (std::string) "/magnetometer0");
    nh_ns.param("magnetometer_topic", magnetometer_topic, (std::string) "/imu_magnetic0");

    //    nh_ns.param("simulated_power_line_frame", frame1, (std::string) "/power_line_real");
    std::vector<std::string> power_lines_frames_list;
    std::vector<double> power_lines_currents_list;

    nh_ns.getParam ("simulated_power_line_frames", power_lines_frames_list);
    nh_ns.getParam ("simulated_power_line_currents", power_lines_currents_list);

    double frequency= 50;

    geometry_msgs::Vector3 magnetometer_vector;
    geometry_msgs::Vector3 power_line_point;
    ros::Rate loop_rate(188);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform1;
    int counter=0;
	ros::Publisher magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>(magnetometer_topic, 1000);

    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;
        ros::Time t = ros::Time(0);
        try{
        	magnetometer_vector.x=0;
        	magnetometer_vector.y=0;
        	magnetometer_vector.z=0;
        	for (int i=0;i<power_lines_frames_list.size();i++)
        	{
        		listener.lookupTransform(frame0, power_lines_frames_list[i], t, transform);

        		geometry_msgs::Vector3 rez=GetMagnetometerReadings(transform, power_lines_currents_list[i], frequency);
            	magnetometer_vector.x += rez.x*1000;
            	magnetometer_vector.y += rez.y*1000;
            	magnetometer_vector.z += rez.z*1000;
        	}
            sensor_msgs::MagneticField data;
        	data.magnetic_field = magnetometer_vector;
            data.header.stamp = ros::Time::now();
            magnetic_field_pub.publish(data);
/*            transform1.setOrigin(tf::Vector3(power_line_point.x, power_line_point.y, power_line_point.z));
            tf::Quaternion q;
            if (power_line_vector.x==power_line_vector.x && power_line_point.x==power_line_point.x)
            {
//            	q.setsetRotation(tf::Vector3(power_line_vector.x, power_line_vector.y, power_line_vector.z),0.1);
            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=CrossProduct(power_line_vector,pomocni);
            	double w=1+DotProduct(power_line_vector,pomocni);
//            	transform1.setRotation(tf::Quaternion(power_line_vector.x, power_line_vector.y, power_line_vector.z,0));
            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));

            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "/magnetometer0", "/power_line"));
            }*/
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();


    }
}


