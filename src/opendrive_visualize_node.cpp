#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opendrive/OpenDrive.hpp"
#include "commonroad/CommonRoad.hpp"

#include <cmath>
#include <vector>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "visualize_lanes");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("map_marker_array", 10);
  // currently publishes every 10 seconds
  ros::Rate r(0.1);
  ros::NodeHandle nh;

  // get parameters from params.yaml
  std::string pathOD;
  nh.getParam("/pathToOpenDrive", pathOD);

  // create opendrive data structure and load info from OpenDrive File
  opendrive::OpenDriveData odr;
  bool bSuccess = opendrive::Load(pathOD, odr);

  // check if the file was loaded successfully
  if(!bSuccess)
  {
    ROS_FATAL("Unable to load Map file!");
    exit(0);
  }

  // marker arry to store all individual line strips
  visualization_msgs::MarkerArray line_strips;

  while (ros::ok())
  {
    int i = 0;
    // iterate trough all roads in the OpenDrive Road file
    for(const opendrive::RoadInformation &road : odr.roads)
    {
      
      // get all width entries in the lane section description
      std::vector<opendrive::LaneWidth> rWidths;
      for(const opendrive::LaneSection &lane_section: road.lanes.lane_sections)
      {
        for(const opendrive::LaneInfo &right: lane_section.right)
        {
          for(const opendrive::LaneWidth &width: right.lane_width)
          {
            // store the width for later use
            rWidths.push_back(width);
          }
        }
      }

      // create marker for the originally defined road geometry
      visualization_msgs::Marker lineStripMain;
      lineStripMain.header.frame_id = "map";
      lineStripMain.header.stamp = ros::Time::now();
      lineStripMain.ns = "map_visualize";
      lineStripMain.action = visualization_msgs::Marker::ADD;
      lineStripMain.pose.orientation.w = 1.0;
      lineStripMain.id = i;
      i ++;
      lineStripMain.type = visualization_msgs::Marker::LINE_STRIP;

      // STRIP markers use only the x component of scale, for the line width
      lineStripMain.scale.x = 0.12;
      // STRIP is white
      lineStripMain.color.a = 1.0;
      lineStripMain.color.r = 1.0;
      lineStripMain.color.g = 1.0;
      lineStripMain.color.b = 1.0;

      // iterate trough all geometries defining the road
      for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road.geometry_attributes)
      {
        // check if the geometry is a line
        // other types are currently not supported
        if(geometry->type == opendrive::GeometryType::LINE)
        {
          // create a new point to store the start position of the geometry
          geometry_msgs::Point p;
          p.z = 0;
          // line defined by s reference frame
          p.x = geometry->start_position_x;
          p.y = geometry->start_position_y;
          // add the point to the visualization_msgs::Marker::LINE_STRIP
          lineStripMain.points.push_back(p);

          // the loop arrives a the last element in the geometry
          if(road.geometry_attributes.back() == geometry)
          {
            // calculate the last point of the lineStrip based on line heading and line length
            p.x = geometry->start_position_x + cos(geometry->heading) * geometry->length;
            p.y = geometry->start_position_y + sin(geometry->heading) * geometry->length;
            // add the point to the visualization_msgs::Marker::LINE_STRIP
            lineStripMain.points.push_back(p);
          }
        }
      }
      // add the newly created Line Strip to the Marker Array
      line_strips.markers.push_back(lineStripMain);

      // create marker for the Marker defined by the lane with and original geometry
      visualization_msgs::Marker lineStripWidth;
      lineStripWidth.header.frame_id = "map";
      lineStripWidth.header.stamp = ros::Time::now();
      lineStripWidth.ns = "map_visualize";
      lineStripWidth.action = visualization_msgs::Marker::ADD;
      lineStripWidth.pose.orientation.w = 1.0;
      lineStripWidth.id = i;
      i ++;
      lineStripWidth.type = visualization_msgs::Marker::LINE_STRIP;

      // STRIP markers use only the x component of scale, for the line width
      lineStripWidth.scale.x = 0.12;
      // STRIP is white
      lineStripWidth.color.a = 1.0;
      lineStripWidth.color.r = 1.0;
      lineStripWidth.color.g = 1.0;
      lineStripWidth.color.b = 1.0;


      // iterate trough all geometries defining the road
      for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road.geometry_attributes)
      {
        // check if the geometry is a line
        // other types are currently not supported
        if(geometry->type == opendrive::GeometryType::LINE)
        {

          //line defined by lane rWidth reference frame
          for(const opendrive::LaneWidth width: rWidths)
          {
            // calculate the difference between width and geometry s position
            double dS = width.soffset - geometry->start_position;
            // in case the difference is smaller 0.1 meter
            if(abs(dS) < 0.1)
            {
              // create a new point to store the start position of the geometry 
              // + the width vector defined by the heading + pi/2 and the lane width
              geometry_msgs::Point p;
              p.z = 0;  
              p.x = geometry->start_position_x + cos(geometry->heading + M_PI/2) * width.a;
              p.y = geometry->start_position_y + sin(geometry->heading + M_PI/2) * width.a;
              // add to LANE_STRIP
              lineStripWidth.points.push_back(p);
              
              // the loop arrives a the last element in the geometry
              if(road.geometry_attributes.back() == geometry)
              {
                // calculate the last point of the lineStrip based on 
                // line heading and line length + the width vector defined by the heading + pi/2 and the lane width
                p.x = geometry->start_position_x + cos(geometry->heading) * geometry->length + cos(geometry->heading + M_PI/2) * width.a;
                p.y = geometry->start_position_y + sin(geometry->heading) * geometry->length + sin(geometry->heading + M_PI/2) * width.a;
                // add to LANE_STRIP
                lineStripWidth.points.push_back(p);
              }
              
            }
          }
        }
      }

      // add the newly created Line Strip to the Marker Array
      line_strips.markers.push_back(lineStripWidth);
      
    }

    // publis the Marker Array
    marker_pub.publish(line_strips);
    // clear the Marker Array for next loop
    line_strips.markers.clear();
    r.sleep();
  }
}