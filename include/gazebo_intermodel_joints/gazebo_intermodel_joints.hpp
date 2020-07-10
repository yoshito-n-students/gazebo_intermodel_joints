#ifndef GAZEBO_INTERMODEL_JOINTS
#define GAZEBO_INTERMODEL_JOINTS

#include <iostream>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <ros/package.h>

#include <boost/pointer_cast.hpp>

namespace gazebo {

class IntermodelJoints : public WorldPlugin {
public:
  virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
    const sdf::ElementPtr formatted_sdf(ToPluginSDF(_sdf));
    plugin_name_ = formatted_sdf->GetAttribute("name")->GetAsString();

    std::cout << "[" << plugin_name_ << "]:"
              << " Start loading plugin" << std::endl;

    // load from [joint] (required, +)
    for (sdf::ElementPtr joint_sdf = formatted_sdf->GetElement("joint"); joint_sdf;
         joint_sdf = joint_sdf->GetNextElement("joint")) {
      LoadJoint(_world, joint_sdf);
    }

    std::cout << "[" << plugin_name_ << "]:"
              << " Loaded plugin" << std::endl;
  }

private:
  // get a sdf element which has been initialized by the plugin format file.
  // the initialied sdf may look empty but have a format information.
  static sdf::ElementPtr LoadPluginFormat() {
    const sdf::ElementPtr sdf(new sdf::Element());
    GZ_ASSERT(sdf::initFile(ros::package::getPath("gazebo_intermodel_joints") +
                                "/sdf/intermodel_joints_plugin.sdf",
                            sdf),
              "Cannot initialize sdf by intermodel_joints_plugin.sdf");
    return sdf;
  }

  // merge the plugin format sdf and the given sdf.
  // assert if the given sdf does not match the format
  // (ex. no required element, value type mismatch, ...).
  static sdf::ElementPtr ToPluginSDF(const sdf::ElementPtr &_src) {
    static const sdf::ElementPtr fmt(LoadPluginFormat());
    const sdf::ElementPtr dst(fmt->Clone());
    GZ_ASSERT(
        sdf::readString("<sdf version='" SDF_VERSION "'>" + _src->ToString("") + "</sdf>", dst),
        "The given sdf does not match IntermodelJoints plugin format");
    return dst;
  }

  // wrap a breaking change on World::GetModel()
  static physics::ModelPtr ModelByName(const physics::WorldPtr &world, const std::string &name) {
#if GAZEBO_MAJOR_VERSION >= 8
    return world->ModelByName(name);
#else
    return world->GetModel(name);
#endif
  }

  // wrap a breaking change on World::GetEntity()
  static physics::EntityPtr EntityByName(const physics::WorldPtr &world, const std::string &name) {
#if GAZEBO_MAJOR_VERSION >= 8
    return world->EntityByName(name);
#else
    return world->GetEntity(name);
#endif
  }

  void LoadJoint(const physics::WorldPtr &_world, const sdf::ElementPtr &_sdf) const {
    // clone the given sdf to make local changes to it
    const sdf::ElementPtr sdf(_sdf->Clone());

    // resolve model & joint names on the basis of [name] attribute
    physics::ModelPtr model;
    std::string joint_name;
    {
      const std::string scoped_name(sdf->GetAttribute("name")->GetAsString());
      for (std::size_t model_name_end = scoped_name.rfind("::");
           !model && model_name_end != std::string::npos;
           model_name_end = scoped_name.rfind("::", model_name_end)) {
        model = ModelByName(_world, scoped_name.substr(0, model_name_end));
        joint_name = scoped_name.substr(model_name_end + 2);
      }
    }
    GZ_ASSERT(model, "Cannot find a valid model name in the [name] attribute");

    // modify the joint name in sdf
    sdf->GetAttribute("name")->Set(joint_name);

    // find the parent link of joint with the [parent] element
    const physics::LinkPtr parent(boost::dynamic_pointer_cast< physics::Link >(
        EntityByName(_world, sdf->Get< std::string >("parent"))));
    GZ_ASSERT(parent, "Cannot find a link with the value of [parent] element");

    // find the child link of joint with the [child] element
    const physics::LinkPtr child(boost::dynamic_pointer_cast< physics::Link >(
        EntityByName(_world, sdf->Get< std::string >("child"))));
    GZ_ASSERT(child, "Cannot find a link with the value of [child] element");

    // create the joint
    const physics::JointPtr joint(
        model->CreateJoint(joint_name, sdf->GetAttribute("type")->GetAsString(), parent, child));
    GZ_ASSERT(joint, "Cannot create a joint with the given parameters");

    // load params from sdf.
    // to find the parent link in Joint::Load(),
    // the parent model has to be temporary set that of the parent link.
    joint->SetModel(parent->GetModel());
    joint->Load(sdf);
    joint->SetModel(model);

    // activate the joint
    joint->Init();

    std::cout << "[" << plugin_name_ << "]:"
              << " Created a joint \"" << joint_name << "\" under the model \""
              << model->GetScopedName() << "\"" << std::endl;
  }

private:
  std::string plugin_name_;
};

} // namespace gazebo

#endif