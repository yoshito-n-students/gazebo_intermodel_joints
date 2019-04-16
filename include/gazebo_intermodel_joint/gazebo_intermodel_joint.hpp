#ifndef GAZEBO_INTERMODEL_JOINT
#define GAZEBO_INTERMODEL_JOINT

#include <iostream>
#include <sstream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class IntermodelJoint : public WorldPlugin {
public:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
    // get the joint name from an attribute
    GZ_ASSERT(sdf->HasAttribute("name"), "No [name] attribute");
    std::string joint_name(sdf->GetAttribute("name")->GetAsString());

    // resolve the parent model of the joint
    const physics::ModelPtr joint_model(ResolveModel(world, joint_name));
    GZ_ASSERT(joint_model, "[name] attribute contains an unknown model name");

    // get the joint type from an element. "fixed" is the only supported type for now.
    GZ_ASSERT(sdf->HasElement("type"), "No [type] element");
    const std::string joint_type(sdf->Get< std::string >("type"));
    GZ_ASSERT(joint_type == "fixed", "Value of [type] element is not supported");

    // find the parent link of joint
    physics::LinkPtr parent;
    {
      GZ_ASSERT(sdf->HasElement("parent"), "No [parent] element");
      std::string name(sdf->Get< std::string >("parent"));
      const physics::ModelPtr model(ResolveModel(world, name));
      GZ_ASSERT(model, "Value of [parent] element contains an unknown model name");
      parent = model->GetLink(name);
      GZ_ASSERT(parent, "Cannot find a link with the value of [parent] element");
    }

    // find the child link of joint
    physics::LinkPtr child;
    {
      GZ_ASSERT(sdf->HasElement("child"), "No [child] element");
      std::string name(sdf->Get< std::string >("child"));
      const physics::ModelPtr model(ResolveModel(world, name));
      GZ_ASSERT(model, "Value of [child] element contains an unknown model name");
      child = model->GetLink(name);
      GZ_ASSERT(child, "Cannot find a link with the value of [child] element");
    }

    // create the joint
    const physics::JointPtr joint(joint_model->CreateJoint(joint_name, joint_type, parent, child));
    GZ_ASSERT(joint, "Cannot create a joint with the given type");
    joint->Init();

    // done!!
    std::cout << "[intermodel_joint]: A " << joint_type << "-type joint named "
              << joint->GetScopedName() << " between the parent link " << parent->GetScopedName()
              << " and the child link " << child->GetScopedName()
              << " has been successfully created" << std::endl;
  }

private:
  // given:
  //   - world the entity belongs to
  //   - entity name containing model names
  // returns:
  //   - the parent model
  //   - entity name without any model names
  static physics::ModelPtr ResolveModel(const physics::WorldPtr &world, std::string &name) {
    // pick the root model name from the given name
    const std::size_t end_root_name(name.find("::"));
    if (end_root_name == std::string::npos) {
      // no root model name found
      return physics::ModelPtr();
    }
    const std::string root_name(name.substr(/* begin */ 0, /* len */ end_root_name - 0));

    // find the root model
    const physics::ModelPtr root_model(world->ModelByName(root_name));
    if (!root_model) {
      // no root model found
      return physics::ModelPtr();
    }

    // remove the root model name from the given name
    name.erase(/* begin */ 0, /* len */ (end_root_name + 2) - 0);
    return ResolveModel(root_model, name);
  }

  static physics::ModelPtr ResolveModel(const physics::ModelPtr &model, std::string &name) {
    // pick the nested model name from the given name
    const std::size_t end_nested_name(name.find("::"));
    if (end_nested_name == std::string::npos) {
      // no nested model name found, which means this model is the parent
      return model;
    }
    const std::string nested_name(name.substr(/* begin */ 0, /* len */ end_nested_name - 0));

    // find the nested model
    const physics::ModelPtr nested_model(model->NestedModel(nested_name));
    if (!nested_model) {
      // no nested model found
      return physics::ModelPtr();
    }

    // remove the nested model name from the given name
    name.erase(/* begin */ 0, /* len */ (end_nested_name + 2) - 0);
    return ResolveModel(nested_model, name);
  }
};

} // namespace gazebo

#endif