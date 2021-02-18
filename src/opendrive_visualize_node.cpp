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
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("lane_marker_array", 10);
  ros::Rate r(0.1);
  ros::NodeHandle nh;

  // get parameters from params.yaml
  std::string pathOD;
  nh.getParam("/pathToOpenDrive", pathOD);

  // create opendrive data structure and load info from OpenDrive File
  opendrive::OpenDriveData odr;
  bool bSuccess = opendrive::Load(pathOD, odr);

  if(!bSuccess)
  {
    ROS_FATAL("Unable to load Map file!");
    exit(0);
  }

  visualization_msgs::MarkerArray line_strips;

  while (ros::ok())
  {
    int i = 0;
    for(const opendrive::RoadInformation &road : odr.roads)
    {
      std::vector<opendrive::LaneWidth> rWidths;
      for(const opendrive::LaneSection &lane_section: road.lanes.lane_sections)
      {
        for(const opendrive::LaneInfo &right: lane_section.right)
        {
          for(const opendrive::LaneWidth &width: right.lane_width)
          {
            rWidths.push_back(width);
          }
        }
      }

      visualization_msgs::Marker lineStripMain;
      lineStripMain.header.frame_id = "map";
      lineStripMain.header.stamp = ros::Time::now();
      lineStripMain.ns = "opendrive_visualize";
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

      for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road.geometry_attributes)
      {
        if(geometry->type == opendrive::GeometryType::LINE)
        {
          geometry_msgs::Point p;
          p.z = 0;
          //line defined by s reference frame
          p.x = geometry->start_position_x;
          p.y = geometry->start_position_y;
          lineStripMain.points.push_back(p);

          if(road.geometry_attributes.back() == geometry)
          {
            p.x = geometry->start_position_x + cos(geometry->heading) * geometry->length;
            p.y = geometry->start_position_y + sin(geometry->heading) * geometry->length;
            lineStripMain.points.push_back(p);
          }
        }
      }

      line_strips.markers.push_back(lineStripMain);


      visualization_msgs::Marker lineStripWidth;
      lineStripWidth.header.frame_id = "map";
      lineStripWidth.header.stamp = ros::Time::now();
      lineStripWidth.ns = "opendrive_visualize";
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

      for(const std::unique_ptr<opendrive::GeometryAttributes> &geometry: road.geometry_attributes)
      {
        if(geometry->type == opendrive::GeometryType::LINE)
        {

        //line defined by lane rWidth reference frame
          for(const opendrive::LaneWidth width: rWidths){
            double dW = width.soffset - geometry->start_position;
            if(abs(dW) < 0.1)
            {
              geometry_msgs::Point p;
              p.z = 0;  
              p.x = geometry->start_position_x + cos(geometry->heading + M_PI/2) * width.a;
              p.y = geometry->start_position_y + sin(geometry->heading + M_PI/2) * width.a;
              lineStripWidth.points.push_back(p);

              if(road.geometry_attributes.back() == geometry)
              {
                p.x = geometry->start_position_x + cos(geometry->heading) * geometry->length + cos(geometry->heading + M_PI/2) * width.a;
                p.y = geometry->start_position_y + sin(geometry->heading) * geometry->length + sin(geometry->heading + M_PI/2) * width.a;
                lineStripWidth.points.push_back(p);
              }
              
            }
          }
        }
      }
      line_strips.markers.push_back(lineStripWidth);
      
    }

    marker_pub.publish(line_strips);
    line_strips.markers.clear();
    r.sleep();
  }
}