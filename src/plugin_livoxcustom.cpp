
#include <ros/ros.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include "livox_ros_driver/CustomMsg.h"
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/livox_points_plugin.h"
#include "livox_laser_simulation/plugin_livoxcustom.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(PluginLivoxCustom)

void PluginLivoxCustom::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
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
    rosPointPub = rosNode->advertise<livox_ros_driver::CustomMsg>(curr_scan_topic, 5);

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

void PluginLivoxCustom::OnNewLaserScans() {
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

        livox_ros_driver::CustomMsg livox_msg;
        livox_msg.header.stamp = ros::Time::now();
        livox_msg.header.frame_id = raySensor->Name();
        // auto &livox_msgs = livox_msg.points;

        livox_msg.timebase = ros::Time::now().toNSec();
        livox_msg.point_num = points_pair.size();
        livox_msg.lidar_id = 1;
        // livox_msg.is_dense = true;
        // livox_msg.is_bigendian = false;
        livox_msg.points.resize(points_pair.size());

        auto iter= livox_msg.points.begin();
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

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();//转换成世界坐标系

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                (*iter).x=point.X();
                (*iter).y=point.Y();
                (*iter).z=point.Z();
                ++iter;
            //} else {

            //    //                ROS_INFO_STREAM("count is wrong:" << verticle_index << "," << verticalRayCount << ","
            //    //                << horizon_index
            //    //                          << "," << rayCount << "," << pair.second.zenith << "," <<
            //    //                          pair.second.azimuth);
            //}
        }
        if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);
        rosPointPub.publish(livox_msg);
        ros::spinOnce();
    }
}



}