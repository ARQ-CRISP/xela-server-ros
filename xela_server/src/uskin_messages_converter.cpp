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
#include "xela_server/xSensorData.h"
#include "xela_server/xServerMsg.h"

ros::Publisher publisher_pad1;
ros::Publisher publisher_pad2;

int uskin_pad1[24][3];
int uskin_pad2[24][3];

int min_reads_taxels_pad1 = 0;
int min_reads_taxels_pad2 = 0;

int message_sequence = 0;



void convert_message_callback(const xela_server::xServerMsg &msg)
{

  // geometry_msgs::PointStamped uskin_node_reading_msg;

  // if (msg.sensor == 1)
  // {
  //   uskin_pad1[msg.taxel-1][0] = msg.xyz.x;
  //   uskin_pad1[msg.taxel-1][1] = msg.xyz.y;
  //   uskin_pad1[msg.taxel-1][2] = msg.xyz.z;
    
  //   min_reads_taxels_pad1 = msg.taxel;

  // }else if (msg.sensor == 2)
  // {
  //   uskin_pad2[msg.taxel-1][0] = msg.xyz.x;
  //   uskin_pad2[msg.taxel-1][1] = msg.xyz.y;
  //   uskin_pad2[msg.taxel-1][2] = msg.xyz.z;

  //   min_reads_taxels_pad2 = msg.taxel;
  // }

  // if (min_reads_taxels_pad1 == min_reads_taxels_pad2 && min_reads_taxels_pad1 == 24)
  // {
  //   xela_server::xServeFullSensorMsg uskin1_frame_reading_msg, uskin2_frame_reading_msg;

  //   for (int i = 0; i < 24; i++)
  //   {
  //     geometry_msgs::PointStamped uskin1_node_reading_msg, uskin2_node_reading_msg;
  
  //     uskin1_node_reading_msg.header.frame_id = i+1;
  //     uskin2_node_reading_msg.header.frame_id = i+1;
      
  //     uskin1_node_reading_msg.point.x = uskin_pad1[i][0];
  //     uskin1_node_reading_msg.point.y = uskin_pad1[i][1];
  //     uskin1_node_reading_msg.point.z = uskin_pad1[i][2];

  //     uskin2_node_reading_msg.point.x = uskin_pad2[i][0];
  //     uskin2_node_reading_msg.point.y = uskin_pad2[i][1];
  //     uskin2_node_reading_msg.point.z = uskin_pad2[i][2];

  //     uskin1_frame_reading_msg.frame.push_back(uskin1_node_reading_msg);
  //     uskin2_frame_reading_msg.frame.push_back(uskin2_node_reading_msg);
  //   }

  //   uskin1_frame_reading_msg.header.seq = message_sequence;
  //   uskin2_frame_reading_msg.header.seq = message_sequence;

  //   ros::Time time = ros::Time::now(); // Ensure same timestampe on readings from both sensors
  //   uskin1_frame_reading_msg.header.stamp = time;
  //   uskin2_frame_reading_msg.header.stamp = time;

  //   message_sequence++;

  //   publisher_pad1.publish(uskin1_frame_reading_msg);
  //   publisher_pad2.publish(uskin2_frame_reading_msg);

  //   min_reads_taxels_pad1 = 0;
  //   min_reads_taxels_pad2 = 0;

  // }

  return;
}
  
  

int main(int argc, char **argv)
{
  // Initialize node and publisher.
  // ros::init(argc, argv, "uskin_messages_converter");
  // ros::NodeHandle nh("~");

  // publisher_pad1 = nh.advertise<xela_server::xServeFullSensorMsg>("/uskin1_xyz_values", 1000);
  // publisher_pad2 = nh.advertise<xela_server::xServeFullSensorMsg>("/uskin2_xyz_values", 1000);
  
  // ros::Subscriber sub = nh.subscribe("/xServTopic", 1000, convert_message_callback);

  // ros::spin();
  // // Wait for next cycle.

  return 0;
}
