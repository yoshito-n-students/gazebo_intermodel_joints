#ifndef GAZEBO_INTERMODEL_JOINT
#define GAZEBO_INTERMODEL_JOINT

#include <iostream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <boost/pointer_cast.hpp>

namespace gazebo {

class IntermodelJoint : public WorldPlugin {
public:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
    // get the joint name and its parent model from the [name] attribute
    std::string joint_name;
    physics::ModelPtr joint_model;
    {
      GZ_ASSERT(sdf->HasAttribute("name"), "No [name] attribute");
      const std::string full_name(sdf->GetAttribute("name")->GetAsString());
      std::string model_name;
      SplitName(full_name, model_name, joint_name);
      GZ_ASSERT(!model_name.empty(), "No model name in [name] attribute");
      joint_model = world->ModelByName(model_name);
      GZ_ASSERT(joint_model, "[name] attribute contains an unknown model name");
    }

    // get the joint type from the [type] element. "fixed" is the only supported type for now.
    GZ_ASSERT(sdf->HasElement("type"), "No [type] element");
    const std::string joint_type(sdf->Get< std::string >("type"));
    GZ_ASSERT(joint_type == "fixed", "Value of [type] element is not supported");

    // find the parent link of joint with the [parent] element
    physics::LinkPtr parent;
    {
      GZ_ASSERT(sdf->HasElement("parent"), "No [parent] element");
      std::string name(sdf->Get< std::string >("parent"));
      parent = boost::dynamic_pointer_cast< physics::Link >(world->EntityByName(name));
      GZ_ASSERT(parent, "Cannot find a link with the value of [parent] element");
    }

    // find the child link of joint with the [child] element
    physics::LinkPtr child;
    {
      GZ_ASSERT(sdf->HasElement("child"), "No [child] element");
      std::string name(sdf->Get< std::string >("child"));
      child = boost::dynamic_pointer_cast< physics::Link >(world->EntityByName(name));
      GZ_ASSERT(child, "Cannot find a link with the value of [child] element");
    }

    // create the joint
    const physics::JointPtr joint(joint_model->CreateJoint(joint_name, joint_type, parent, child));
    GZ_ASSERT(joint, "Cannot create a joint with the given parameters");
    joint->Init();

    // done!!
    std::cout << "[intermodel_joint]: A " << joint_type << "-type joint named "
              << joint->GetScopedName() << " between the parent link " << parent->GetScopedName()
              << " and the child link " << child->GetScopedName()
              << " has been successfully created" << std::endl;
  }

private:
  static void SplitName(const std::string &name, std::string &model, std::string &entity) {
    const std::size_t end_model(name.rfind("::"));
    if (end_model != std::string::npos) {
      // model name found
      model = name.substr(/* begin */ 0, /* len */ end_model - 0);
      entity = name.substr(/* begin */ end_model + 2);
    } else {
      // no model name found
      model = "";
      entity = name;
    }
  }
};

} // namespace gazebo

#endif