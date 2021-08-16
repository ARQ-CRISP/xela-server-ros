/*
 * Copyright: (C) 2019 CRISP, Advanced Robotics at Queen Mary,
 *                Queen Mary University of London, London, UK
 * Author: Rodrigo Neves Zenha <r.neveszenha@qmul.ac.uk>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 *
 */
/**
 * \file grid_map_plotter.cpp
 *
 * \author Rodrigo Neves Zenha
 * \copyright  Released under the terms of the GNU GPL v3.0.
 */

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "xela_server/xServerMsg.h"


#include <cmath>

// Hardcoded values retrieved from trial & error
#define XNODEMAXREAD 20000
#define YNODEMAXREAD 20000
#define ZNODEMAXREAD 50000

using namespace grid_map;
using namespace std;

ros::Publisher publisher;
int sensor_to_visualize;

float uskin_pad[24][3];
float uskin_pad_min_reads[24][3] = {{65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000},
                                    {65000, 65000, 65000}};
int min_reads = 0;
bool normalized_flag = false;

void retrieveSensorToVisualize(ros::NodeHandle nh)
{
  ROS_INFO("  >> retrieveSensorToVisualize()");

  if (!nh.hasParam("/grid_map_ploter_node/sensor_viz"))
  {
    ROS_ERROR("    It was not possible to retrieve Sensor to visualize!! Shutting down...");

    ros::shutdown();
  }
  else
  {
    nh.getParam("//grid_map_ploter_node/sensor_viz", sensor_to_visualize);

    ROS_INFO("      Found Sensor: %d. Starting visualization.", sensor_to_visualize);
  }

  ROS_INFO("  << retrieveSensorToVisualize()");

  return;
}

void normalize(int x, int y, int z, int index, float values_normalized[24][3])
{
  // if (index == 0){
  //   cout << "values before normalize: x: "<< x << "  y: " << y << "  z: " << z << endl;
  //   cout << "uskin min reads: "<< uskin_pad_min_reads[0][0] << "  y: " << uskin_pad_min_reads[0][1] << "  z: " << uskin_pad_min_reads[0][2] << endl;
  // }

  values_normalized[index][0] = (int)((((float)x - uskin_pad_min_reads[index][0]) / (XNODEMAXREAD - uskin_pad_min_reads[index][0])) * 100);
  values_normalized[index][1] = (int)((((float)y - uskin_pad_min_reads[index][1]) / (YNODEMAXREAD - uskin_pad_min_reads[index][1])) * 100);
  values_normalized[index][2] = (int)((((float)z - uskin_pad_min_reads[index][2]) / (ZNODEMAXREAD - uskin_pad_min_reads[index][2])) * 100);
  // if (z_value < 0)
  //   z_value = 0; // Force Z to be above 0. Z would only get negative values if a node is being "pulled", which should not happen
  // Force normalized valies to be within boundaries
  values_normalized[index][2] < 0 ? values_normalized[index][2] = 0 : (values_normalized[index][2] > 100 ? values_normalized[index][2] = 100 : values_normalized[index][2]);
  values_normalized[index][0] < -100 ? values_normalized[index][0] = -100 : (values_normalized[index][0] > 100 ? values_normalized[index][0] = 100 : values_normalized[index][0]);
  values_normalized[index][1] < -100 ? values_normalized[index][1] = -100 : (values_normalized[index][1] > 100 ? values_normalized[index][1] = 100 : values_normalized[index][1]);

  // if (index == 0)
  // cout << "values before: x: "<< values_normalized[0][0] << "  y: " << values_normalized[0][1] << "  z: " << values_normalized[0][2] << endl;

  return;
}

void ploterCallback(const xela_server::xServerMsg &msg)
{

  // We only care for messages corresponding to selected sensor
  if (msg.sensor != sensor_to_visualize)
    return;
  // Create grid map.
  GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  map.setFrameId("map");
  map.setGeometry(Length(120, 180), 30);
  // map.setGeometry(Length(90, 180), 30);

  // Add data to grid map.
  ros::Time time = ros::Time::now();

  // cout << min_reads<< endl;

  if (min_reads < 10 || !normalized_flag)
  {
    normalized_flag = true;
    for (int i = 0; i < 24; i++)
    {
        cout <<"Got values for taxel " << i << " are x: "<<  msg.points[i].point.x << "  y: " <<  msg.points[i].point.y << "  z: " <<  msg.points[i].point.z << endl;
        if (msg.points[i].point.x < uskin_pad_min_reads[i][0] && msg.points[i].point.x != 0)
          uskin_pad_min_reads[i][0] = msg.points[i].point.x;
        if (msg.points[i].point.y < uskin_pad_min_reads[i][1] && msg.points[i].point.y != 0)
          uskin_pad_min_reads[i][1] = msg.points[i].point.y;
        if (msg.points[i].point.z < uskin_pad_min_reads[i][2] && msg.points[i].point.z != 0)
          uskin_pad_min_reads[i][2] = msg.points[i].point.z;

        if(uskin_pad_min_reads[i][0] == 65000 || uskin_pad_min_reads[i][1] == 65000 || uskin_pad_min_reads[i][2] == 65000)
          normalized_flag = false;

        cout <<"Min values for taxel " << i << " are x: "<< uskin_pad_min_reads[i][0] << "  y: " << uskin_pad_min_reads[i][1] << "  z: " << uskin_pad_min_reads[i][2] << endl;

    }
        cout << endl;
    min_reads++;
    cout <<"Min values readings: " << min_reads << endl;

    return;
  }


  for (int i = 0; i < 24; i++)
  {
    if (i == 0)
      cout <<"Got values for taxel " << i << " are x: "<<  msg.points[i].point.x << "  y: " <<  msg.points[i].point.y << "  z: " <<  msg.points[i].point.z << endl;
    normalize(msg.points[i].point.x, msg.points[i].point.y, msg.points[i].point.z, i, uskin_pad);
  }




  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    Position position;
    map.getPosition(*it, position);
    Index index;
    map.getIndex(position, index);
    // cout << "Position: (" << index(0) << ","<< index(1) << ") -> uskin_pad index: " << (index(1))+ (index(0)) * 6 << endl;

    float vector_lenght = sqrt(pow(uskin_pad[(index(1))+ (index(0)) * 6][0], 2) + pow(uskin_pad[(index(1)) + (index(0)) * 6][1], 2) + pow(uskin_pad[(index(1)) + (index(0)) * 6][2], 2));

    map.at("elevation", *it) = -1 * uskin_pad[(index(1)) + (index(0)) * 6][2];
    map.at("normal_x", *it) = -1 * uskin_pad[(index(1)) + (index(0)) * 6][1] * 10 / vector_lenght;
    map.at("normal_y", *it) = -1 * uskin_pad[(index(1)) + (index(0)) * 6][0] * 10 / vector_lenght;
    map.at("normal_z", *it) = uskin_pad[(index(1)) + (index(0)) * 6][2] * 10 / vector_lenght;

    // ROS_INFO("Printing node %i at position x:%i, y:%i, with value %f", ((index(1))+ (index(0)) * 6), index(0), index(1), uskin_pad[(index(1)) + (index(0)) * 6][2]);
  }

// Publish grid map.
map.setTimestamp(time.toNSec());
grid_map_msgs::GridMap message;
GridMapRosConverter::toMessage(map, message);
publisher.publish(message);

}

int main(int argc, char **argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_ploter_node");
  ros::NodeHandle nh("~");

  retrieveSensorToVisualize(nh);

  publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  ros::Subscriber sub = nh.subscribe("/xServTopic", 1000, ploterCallback);

  ros::spin();
  // Wait for next cycle.

  return 0;
}
