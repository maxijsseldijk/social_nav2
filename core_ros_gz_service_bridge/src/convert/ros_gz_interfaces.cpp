//DONE!!



#include "core_ros_gz_service_bridge/convert/ros_gz_interfaces.hpp"  //DONE!!

namespace core_ros_gz_service_bridge
{



template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Entity & gz_msg)
{
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_name(ros_msg.name);
  switch (ros_msg.type) {
    case ros_gz_interfaces::msg::Entity::NONE:
      gz_msg.set_type(gz::msgs::Entity::NONE);
      break;
    case ros_gz_interfaces::msg::Entity::LIGHT:
      gz_msg.set_type(gz::msgs::Entity::LIGHT);
      break;
    case ros_gz_interfaces::msg::Entity::MODEL:
      gz_msg.set_type(gz::msgs::Entity::MODEL);
      break;
    case ros_gz_interfaces::msg::Entity::LINK:
      gz_msg.set_type(gz::msgs::Entity::LINK);
      break;
    case ros_gz_interfaces::msg::Entity::VISUAL:
      gz_msg.set_type(gz::msgs::Entity::VISUAL);
      break;
    case ros_gz_interfaces::msg::Entity::COLLISION:
      gz_msg.set_type(gz::msgs::Entity::COLLISION);
      break;
    case ros_gz_interfaces::msg::Entity::SENSOR:
      gz_msg.set_type(gz::msgs::Entity::SENSOR);
      break;
    case ros_gz_interfaces::msg::Entity::JOINT:
      gz_msg.set_type(gz::msgs::Entity::JOINT);
      break;
    default:
      std::cerr << "Unsupported entity type [" << ros_msg.type << "]\n";
  }
}
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Entity & ros_msg,
  gz::msgs::Pose & gz_msg)
{
  gz_msg.set_id(ros_msg.id);
  gz_msg.set_name(ros_msg.name);
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Entity & gz_msg,
  ros_gz_interfaces::msg::Entity & ros_msg)
{
  ros_msg.id = gz_msg.id();
  ros_msg.name = gz_msg.name();
  if (gz_msg.type() == gz::msgs::Entity::Type::Entity_Type_NONE) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::NONE;
  } else if (gz_msg.type() == gz::msgs::Entity::LIGHT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LIGHT;
  } else if (gz_msg.type() == gz::msgs::Entity::MODEL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::MODEL;
  } else if (gz_msg.type() == gz::msgs::Entity::LINK) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::LINK;
  } else if (gz_msg.type() == gz::msgs::Entity::VISUAL) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::VISUAL;
  } else if (gz_msg.type() == gz::msgs::Entity::COLLISION) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::COLLISION;
  } else if (gz_msg.type() == gz::msgs::Entity::SENSOR) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::SENSOR;
  } else if (gz_msg.type() == gz::msgs::Entity::JOINT) {
    ros_msg.type = ros_gz_interfaces::msg::Entity::JOINT;
  } else {
    std::cerr << "Unsupported Entity [" <<
      gz_msg.type() << "]" << std::endl;
  }
}


template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::EntityFactory & ros_msg,
  gz::msgs::EntityFactory & gz_msg) {

  gz_msg.set_name(ros_msg.name);
  gz_msg.set_allow_renaming(ros_msg.allow_renaming);


  if(ros_msg.sdf.length()>0) {
    gz_msg.set_sdf(ros_msg.sdf);
  } else if(ros_msg.sdf_filename.length()>0) {
    gz_msg.set_sdf_filename(ros_msg.sdf_filename);
  } else if(ros_msg.clone_name.length()>0) {
    gz_msg.set_clone_name(ros_msg.clone_name);
  } else {
    std::cerr << "Must provide one of: sdf, sdf_filname, or clone_name" << std::endl;
  }


  convert_ros_to_gz(ros_msg.pose, *gz_msg.mutable_pose());

  gz_msg.set_relative_to(ros_msg.relative_to);

}

template<>
void
convert_gz_to_ros(
  const gz::msgs::EntityFactory & gz_msg,
  ros_gz_interfaces::msg::EntityFactory & ros_msg)
{
  ros_msg.name = gz_msg.name();
  ros_msg.allow_renaming = gz_msg.allow_renaming();

  ros_msg.sdf = gz_msg.sdf();
  ros_msg.sdf_filename = gz_msg.sdf_filename();
  ros_msg.clone_name = gz_msg.clone_name();

  convert_gz_to_ros(gz_msg.pose(), ros_msg.pose);

  ros_msg.relative_to = gz_msg.relative_to();
}


#if HAVE_DATAFRAME
template<>
void
convert_ros_to_gz(
  const ros_gz_interfaces::msg::Dataframe & ros_msg,
  gz::msgs::Dataframe & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  auto * rssiPtr = gz_msg.mutable_header()->add_data();
  rssiPtr->set_key("rssi");
  rssiPtr->add_value(std::to_string(ros_msg.rssi));

  gz_msg.set_src_address(ros_msg.src_address);
  gz_msg.set_dst_address(ros_msg.dst_address);

  gz_msg.set_data(&(ros_msg.data[0]), ros_msg.data.size());
}

template<>
void
convert_gz_to_ros(
  const gz::msgs::Dataframe & gz_msg,
  ros_gz_interfaces::msg::Dataframe & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);

  ros_msg.src_address = gz_msg.src_address();
  ros_msg.dst_address = gz_msg.dst_address();

  const auto & header = gz_msg.header();
  for (auto i = 0; i < header.data_size(); ++i) {
    if (header.data(i).key() == "rssi" && header.data(i).value_size() > 0) {
      try {
        ros_msg.rssi = std::stod(header.data(i).value(0));
      } catch (const std::invalid_argument &) {
        std::cerr << "RSSI value is invalid (" <<
          header.data(i).value(0) << ")" << std::endl;
      } catch (const std::out_of_range &) {
        std::cerr << "RSSI value is out of range (" <<
          header.data(i).value(0) << ")" << std::endl;
      }
    }
  }

  ros_msg.data.resize(gz_msg.data().size());
  std::copy(
    gz_msg.data().begin(),
    gz_msg.data().begin() + gz_msg.data().size(),
    ros_msg.data.begin());
}
#endif  // HAVE_DATAFRAME




}  // namespace ros_gz_bridge
