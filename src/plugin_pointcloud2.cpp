
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/livox_points_plugin.h"
#include "livox_laser_simulation/plugin_pointcloud2.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(PluginPointCloud2)

void PluginPointCloud2::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
    std::vector<std::vector<double>> datas;
    std::string file_name = sdf->Get<std::string>("csv_file_name");
    ROS_INFO_STREAM("load csv file name:" << file_name);
    if (!CsvReader::ReadCsvFile(file_name, datas)) {
        ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
        return;
    }
    sdfPtr = sdf;
    auto rayElem = sdfPtr->GetElement("ray");
    auto scanElem = rayElem->GetElement("scan");
    auto rangeElem = rayElem->GetElement("range");

    int argc = 0;
    char **argv = nullptr;
    auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
    ROS_INFO_STREAM("ros topic name:" << curr_scan_topic);
    ros::init(argc, argv, curr_scan_topic);
    rosNode.reset(new ros::NodeHandle);
    rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

    raySensor = _parent;
    auto sensor_pose = raySensor->Pose();
    SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());

    node = transport::NodePtr(new transport::Node());
    node->Init(raySensor->WorldName());
    scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
    aviaInfos.clear();
    convertDataToRotateInfo(datas, aviaInfos);
    ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
    maxPointSize = aviaInfos.size();

    RayPlugin::Load(_parent, sdfPtr);
    laserMsg.mutable_scan()->set_frame(_parent->ParentName());
    // parentEntity = world->GetEntity(_parent->ParentName());
    parentEntity = this->world->EntityByName(_parent->ParentName());
    auto physics = world->Physics();
    laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
    laserCollision->SetName("ray_sensor_collision");
    laserCollision->SetRelativePose(_parent->Pose());
    laserCollision->SetInitialRelativePose(_parent->Pose());
    rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
    laserCollision->SetShape(rayShape);
    samplesStep = sdfPtr->Get<int>("samples");
    downSample = sdfPtr->Get<int>("downsample");
    if (downSample < 1) {
        downSample = 1;
    }
    ROS_INFO_STREAM("sample:" << samplesStep);
    ROS_INFO_STREAM("downsample:" << downSample);
    rayShape->RayShapes().reserve(samplesStep / downSample);
    rayShape->Load(sdfPtr);
    rayShape->Init();
    minDist = rangeElem->Get<double>("min");
    maxDist = rangeElem->Get<double>("max");
    auto offset = laserCollision->RelativePose();
    ignition::math::Vector3d start_point, end_point;
    for (int j = 0; j < samplesStep; j += downSample) {
        int index = j % maxPointSize;
        auto &rotate_info = aviaInfos[index];
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
        auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
        start_point = minDist * axis + offset.Pos();
        end_point = maxDist * axis + offset.Pos();
        rayShape->AddRay(start_point, end_point);
    }
}

void PluginPointCloud2::OnNewLaserScans() {
    if (rayShape) {
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);
        rayShape->Update();

        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        sensor_msgs::PointCloud2 scan_point;
        scan_point.header.stamp = ros::Time::now();
        scan_point.header.frame_id = raySensor->Name();
        // auto &scan_points = scan_point.points;

        scan_point.height = 1;
        scan_point.width = points_pair.size();
        scan_point.is_dense = true;
        scan_point.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(scan_point);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points_pair.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(scan_point, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(scan_point, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(scan_point, "z");

        for (auto &pair : points_pair) {
            //int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            //int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            //if (verticle_index < 0 || horizon_index < 0) {
            //   continue;
            //}
            //if (verticle_index < verticalRayCount && horizon_index < rayCount) {
            //   auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= RangeMax()) {
                    range = 0;
                } else if (range <= RangeMin()) {
                    range = 0;
                }
                //scan->set_ranges(index, range);
                //scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();//转换成世界坐标系

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                // scan_points.emplace_back();
                // scan_points.back().x = point.X();
                // scan_points.back().y = point.Y();
                // scan_points.back().z = point.Z();
                *iter_x = point.X();
                *iter_y = point.Y();
                *iter_z = point.Z();
                ++iter_x;
                ++iter_y;
                ++iter_z;
            //} else {

            //    //                ROS_INFO_STREAM("count is wrong:" << verticle_index << "," << verticalRayCount << ","
            //    //                << horizon_index
            //    //                          << "," << rayCount << "," << pair.second.zenith << "," <<
            //    //                          pair.second.azimuth);
            //}
        }
        if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);
        rosPointPub.publish(scan_point);
        ros::spinOnce();
    }
}



}