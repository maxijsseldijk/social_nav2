//DONE!!

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>

#include <memory>
#include <string>


#include "service_factories/core_ros_gz_interfaces.hpp"  //DONE!!!
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"  //DONE
#include "ros_gz_interfaces/srv/spawn_entity.hpp"  //DONE
#include "ros_gz_interfaces/srv/delete_entity.hpp"  //DONE
#include "core_ros_gz_service_bridge/convert/ros_gz_interfaces.hpp"  //DONE!!

#include "service_factory.hpp"  //DONE!!

namespace core_ros_gz_service_bridge
{

std::shared_ptr<ServiceFactoryInterface>
get_service_factory__ros_gz_interfaces(
  const std::string & ros_type_name,
  const std::string & gz_req_type_name,
  const std::string & gz_rep_type_name)
{

  if (
    ros_type_name == "ros_gz_interfaces/srv/DeleteEntity" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.Entity") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    std::cout<<"Deleting";
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::DeleteEntity,
        gz::msgs::Entity,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.Entity", "gz.msgs.Boolean");
  }

  if (
    ros_type_name == "ros_gz_interfaces/srv/SpawnEntity" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.EntityFactory") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
      std::cout<<"Spawning";
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::SpawnEntity,
        gz::msgs::EntityFactory,
        gz::msgs::Boolean>
    >(ros_type_name, "gz.msgs.EntityFactory", "gz.msgs.Boolean");
  }

  if (
    ros_type_name == "ros_gz_interfaces/srv/SetEntityPose" &&
    (gz_req_type_name.empty() || gz_req_type_name == "gz.msgs.Pose") &&
    (gz_rep_type_name.empty() || gz_rep_type_name == "gz.msgs.Boolean"))
  {
    return std::make_shared<
      ServiceFactory<
        ros_gz_interfaces::srv::SetEntityPose,
        gz::msgs::Pose,
        gz::msgs::Boolean>
    >(ros_type_name, "ignition.msgs.Pose", "ignition.msgs.Boolean");
  }
  return nullptr;
}




template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::DeleteEntity::Request & ros_req,
  gz::msgs::Entity & gz_req)
{
  convert_ros_to_gz(ros_req.entity, gz_req);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::SpawnEntity::Request & ros_req,
  gz::msgs::EntityFactory & gz_req)
{
  
  convert_ros_to_gz(ros_req.entity_factory, gz_req);
}

template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::srv::SetEntityPose::Request & ros_req,
  gz::msgs::Pose & gz_req)
{

  convert_ros_to_gz(ros_req.entity, gz_req);
  convert_ros_to_gz(ros_req.pose, gz_req);
}




template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::DeleteEntity::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::SpawnEntity::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Boolean & gz_rep,
  ros_gz_interfaces::srv::SetEntityPose::Response & ros_res)
{
  ros_res.success = gz_rep.data();
}





template<>
bool
send_response_on_error(ros_gz_interfaces::srv::DeleteEntity::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::SpawnEntity::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}

template<>
bool
send_response_on_error(ros_gz_interfaces::srv::SetEntityPose::Response & ros_res)
{
  // TODO(now): Is it worth it to have a different field to encode Gazebo request errors?
  //  Currently we're reusing the success field, which seems fine for this case.
  ros_res.success = false;
  return true;
}
}  // namespace core_ros_gz_service_bridge
