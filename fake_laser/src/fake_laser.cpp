#include <csignal>
#include <cstdio>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>


#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0
#endif

using namespace std;

// Convert from map index to world coords
#define MAP_WXGX(map_width, map_height, map_scale, i) (i * map_scale)
#define MAP_WYGY(map_width, map_height, map_scale, j) (j * map_scale)

// Convert from world coords to map coords
#define MAP_GXWX(map_width, map_height, map_scale, x) (floor(x / map_scale + 0.5))
#define MAP_GYWY(map_width, map_height, map_scale, y) (floor(y / map_scale + 0.5))

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map_width, map_height, i, j) ((i >= 0) && (i < map_width) && (j >= 0) && (j < map_height))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map_width, i, j) ((i) + (j)*map_width)

//#define OBSTACLES

nav_msgs::GetMap::Request static_map_req;
nav_msgs::GetMap::Response static_map_resp;

std::vector<std::pair<double, double> > points;

double map_calc_range(double ox, double oy, double oa, double max_range)
{
	double map_scale;
	int map_width;
	int map_height;

	map_scale = static_map_resp.map.info.resolution;
	map_width = static_map_resp.map.info.width;
	map_height = static_map_resp.map.info.height;

	// Bresenham raytracing
	int x0, x1, y0, y1;
	int x, y;
	int xstep, ystep;
	char steep;
	int tmp;
	int deltax, deltay, error, deltaerr;

	x0 = MAP_GXWX(map_width, map_height, map_scale, ox);
	y0 = MAP_GYWY(map_width, map_height, map_scale, oy);

	x1 = MAP_GXWX(map_width, map_height, map_scale, (ox + max_range * cos(oa)));
	y1 = MAP_GYWY(map_width, map_height, map_scale, (oy + max_range * sin(oa)));

	if (abs(y1 - y0) > abs(x1 - x0))
	{
		steep = 1;

		tmp = x0;
		x0 = y0;
		y0 = tmp;

		tmp = x1;
		x1 = y1;
		y1 = tmp;
	}
	else
	{
		steep = 0;
	}

	deltax = abs(x1 - x0);
	deltay = abs(y1 - y0);
	error = 0;
	deltaerr = deltay;

	x = x0;
	y = y0;

	xstep = (x0 < x1) ? 1 : -1;
	ystep = (y0 < y1) ? 1 : -1;

	if (steep)
	{
		if (!MAP_VALID(map_width, map_height, y, x) || static_map_resp.map.data[MAP_INDEX(map_width, y, x)] != 0)
		{
			return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
		}
		// edit
		for(int loop = 0; loop < points.size(); loop++)
		{
			double tx, ty;
			tx = points[loop].first;
			ty = points[loop].second;
			if (( x * map_scale < ty + 1 && x * map_scale > ty - 0.1 && y * map_scale > tx - 0.1 && y * map_scale < tx + 0.1))
			{
				return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
			}
		}
	}
	else
	{
		if (!MAP_VALID(map_width, map_height, x, y) || static_map_resp.map.data[MAP_INDEX(map_width, x, y)] != 0)
		{
			return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
		}
		// edit
		for(int loop = 0; loop < points.size(); loop++)
		{
			double tx, ty;
			tx = points[loop].first;
			ty = points[loop].second;
			if (( y * map_scale < ty + 0.1 && y * map_scale > ty - 0.1 && x * map_scale > tx - 0.1 && x * map_scale < tx + 0.1))
			{
				return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
			}
		}
	}

	while (x != (x1 + xstep * 1))
	{
		x += xstep;
		error += deltaerr;
		if (2 * error >= deltax)
		{
			y += ystep;
			error -= deltax;
		}

		if (steep)
		{
			if (!MAP_VALID(map_width, map_height, y, x) || static_map_resp.map.data[MAP_INDEX(map_width, y, x)] != 0)
			{
				return sqrt((double)((x - x0) * (x - x0) + (y - y0) * (y - y0))) * map_scale;
			}
			// edit
			for(int loop = 0; loop < points.size(); loop++)
			{
				double tx, ty;
				tx = points[loop].first;
				ty = points[loop].second;
				if (( x * map_scale < ty + 0.1 && x * map_scale > ty - 0.1 && y * map_scale > tx - 0.1 && y * map_scale < tx + 0.1))
				{
					return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
				}
			}
		}
		else
		{
			if (!MAP_VALID(map_width, map_height, x, y) || static_map_resp.map.data[MAP_INDEX(map_width, x, y)] != 0)
			{
				return sqrt((double)((x - x0) * (x - x0) + (y - y0) * (y - y0))) * map_scale;
			}
			// edit
			for(int loop = 0; loop < points.size(); loop++)
			{
				double tx, ty;
				tx = points[loop].first;
				ty = points[loop].second;
				if (( y * map_scale < ty + 0.1 && y * map_scale > ty - 0.1 && x * map_scale > tx - 0.1 && x * map_scale < tx + 0.1))
				{
					return sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * map_scale;
				}
			}
		}
	}
	//ROS_INFO("Ouch.. Ouch.. #5 :: xy(%d, %d) max_range(%.3lf)", x, y, static_map_resp.map.data[MAP_INDEX(map_width,x,y)]);
	return max_range;
}

int main(int argc, char **argv)
{
	// published data
	sensor_msgs::LaserScan scan_msg;

	// parameters
	std::string scan_name;
	std::string frame_id;
	std::string global_frame_id;
	std::string static_map;

	ros::init(argc, argv, "fake_laser");
	ros::NodeHandle nh;
	ros::NodeHandle n("~");

	n.param<std::string>("scan_name", scan_name, "scan");
	n.param<std::string>("frame_id", frame_id, "base_laser");
	n.param<std::string>("global_frame_id", global_frame_id, "map");
	n.param<std::string>("static_map", static_map, "static_map");

	ROS_INFO("Requesting the map...");
	while (!ros::service::call(static_map, static_map_req, static_map_resp))
	{
		ROS_WARN("Request for map failed; trying again...");
		ros::Duration d(0.5);
		d.sleep();
	}
#ifdef OBSTACLES
	// obstacles
	std::pair<double, double> point;
	point.first = 33.9;
	point.second = 20.8;
	points.push_back(point);
	point.first = 32.6;
	point.second = 20.1;
	points.push_back(point);
	point.first = 31.5;
	point.second = 19.4;
	points.push_back(point);


	point.first = 35.9;
	point.second = 19.06;
	points.push_back(point);
	point.first = 34.6;
	point.second = 18.16;
	points.push_back(point);
	point.first = 32.98;
	point.second = 17.07;
	points.push_back(point);

	point.first = 36.73;
	point.second = 16.28;
	points.push_back(point);
	point.first = 35.47;
	point.second = 15.35;
	points.push_back(point);	
	point.first = 34.35;
	point.second = 14.68;
	points.push_back(point);
#endif
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_name, 1024);

	int num_values = 541; //or 1081
	scan_msg.header.frame_id = frame_id;

	scan_msg.range_min = 0.01;
	scan_msg.range_max = 20.0;

	scan_msg.scan_time = 1000.0 / 5000.0; //실제 레이저에서 어떻게 나오는지 확인

	scan_msg.angle_increment = 0.5 * DEG2RAD;
	scan_msg.angle_min = -45.0 * DEG2RAD;
	scan_msg.angle_max = 225.0 * DEG2RAD;

	scan_msg.time_increment = scan_msg.scan_time / num_values;

	scan_msg.ranges.resize(num_values);
	scan_msg.intensities.resize(num_values);

	ros::Rate r((num_values == 541) ? 100.0 : 100.0);

	tf::TransformListener m_listener_laser_to_map;

	while (ros::ok())
	{
		ros::Time start = ros::Time::now();

		scan_msg.header.stamp = start;
		++scan_msg.header.seq;

		try
		{
			tf::Stamped<tf::Pose> laser_pose, global_laser_pose;
			laser_pose.frame_id_ = frame_id;
			laser_pose.setIdentity();

			m_listener_laser_to_map.waitForTransform("map", frame_id, ros::Time::now(), ros::Duration(0.020));
			m_listener_laser_to_map.transformPose("map", laser_pose, global_laser_pose);

			//ROS_INFO("Goal(%s), (%.2lf, %.2lf, %.1lf)", global_laser_pose.frame_id_.c_str(),
			//		global_laser_pose.getOrigin().getX(), global_laser_pose.getOrigin().getY(),
			//		angles::to_degrees(tf::getYaw(global_laser_pose.getRotation())));

			double laser_ref_x = global_laser_pose.getOrigin().getX();
			double laser_ref_y = global_laser_pose.getOrigin().getY();

			double laser_ref_angle = tf::getYaw(global_laser_pose.getRotation());
			double laser_beam_angle = 0.0;

			ros::Time start_time = ros::Time::now();
			for (int i = 0; i < num_values; i++)
			{
				laser_beam_angle = laser_ref_angle + scan_msg.angle_min + ((double)i) * scan_msg.angle_increment;

				scan_msg.ranges[i] = map_calc_range(laser_ref_x, laser_ref_y, laser_beam_angle, scan_msg.range_max);
				scan_msg.intensities[i] = 512;
			}
			//ROS_INFO("RANGE (%.1lf, %.1lf) :: 11th = %d", laser_ref_x, laser_ref_y, (int) (scan_msg.ranges[10] * 1000.0));

			laser_pose.stamp_ = ros::Time::now();
			scan_pub.publish(scan_msg);

			ros::Time end_time = ros::Time::now();
			//sensor_msgs::LaserScanConstPtr *msg;
			//scan_sub(sensor_msgs::LaserScanConsPtr & msg)
			// sensor_msgs::LaserScan scan;
			// //scan = *msg;
			// std::vector<float>::iterator iter;
			// //1
			// double angle;
			// for(iter = scan.ranges.begin(), angle = scan.angle_min;
			// 	iter != scan.ranges.end(); iter++, angle+=scan.angle_increment)
			// {
			// 	float range = (*iter);
			// 	double x,y;
			// 	x = range * cos(angle);
			// 	y = range * sin(angle);
			// }
			// //2
			// for(int i = 0; i < (int)scan.ranges.size(); i++)
			// {
			// 	float range = scan.ranges[i];
			// }

			//ROS_INFO("Range Calc Time = %.3lf", (end_time - start_time).toSec());
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
