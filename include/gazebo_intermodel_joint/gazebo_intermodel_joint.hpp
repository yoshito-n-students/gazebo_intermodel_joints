#ifndef GAZEBO_GAZEBO_INTERMODEL_JOINT
#define GAZEBO_GAZEBO_INTERMODEL_JOINT

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class IntermodelJoint : public WorldPlugin {
public:
  virtual void Load(physics::WorldPtr world, sdf::ElementPtr sdf) {}
};

} // namespace gazebo

#endif