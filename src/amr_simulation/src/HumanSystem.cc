// SPDX-License-Identifier: Apache-2.0
// Physical waypoint servo and displacement-driven visual gait, Gazebo Sim 8.
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>

namespace amr_simulation
{
class HumanSystem : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
  gz::sim::Link body;
  std::map<std::string, double> p;
  std::vector<gz::math::Vector3d> route;
  struct Visual { gz::sim::Entity entity; std::string name; gz::math::Pose3d rest; };
  std::vector<Visual> visuals;
  size_t goal{0};
  bool loop{true}, initialized{false}, finished{false};
  double phase{0}, gaitSpeed{0}, animationTime{0};
  gz::math::Vector3d previous;

  void Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    body = gz::sim::Link(gz::sim::Model(_entity).LinkByName(_ecm, "body"));
    body.EnableVelocityChecks(_ecm);
    auto config = _sdf->Clone();
    for (const auto *key : {"speed", "acceleration", "braking", "response_time",
        "max_force", "max_torque", "yaw_rate", "yaw_gain", "yaw_response_time",
        "yaw_friction", "heading_gate", "range_tolerance", "arrival_speed",
        "mass", "friction", "gravity", "yaw_inertia", "stride_length", "leg_swing",
        "arm_swing", "bob_height", "animation_rate", "gait_stop_speed",
        "gait_response_time", "leg_height", "shoulder_height"})
      p[key] = config->Get<double>(key);
    loop = config->Get<bool>("loop");
    auto points = config->GetElement("waypoints");
    for (auto point = points->GetElement("waypoint"); point;
         point = point->GetNextElement("waypoint"))
    {
      auto xy = point->Get<gz::math::Vector2d>();
      route.emplace_back(xy.X(), xy.Y(), 0);
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !body.Valid(_ecm) || route.empty()) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0) { initialized = false; goal = 0; finished = false; phase = 0; return; }
    const auto pose = gz::sim::worldPose(body.Entity(), _ecm);
    auto pos = pose.Pos(); pos.Z() = 0;
    if (!initialized)
    {
      previous = pos;
      visuals.clear();
      _ecm.Each<gz::sim::components::Visual, gz::sim::components::ParentEntity,
                gz::sim::components::Name, gz::sim::components::Pose>(
          [&](const gz::sim::Entity &e, const auto *, const auto *parent,
              const auto *name, const auto *rest) {
            if (parent->Data() == body.Entity())
              visuals.push_back({e, name->Data(), rest->Data()});
            return true;
          });
      initialized = true;
    }
    auto velocity = body.WorldLinearVelocity(_ecm).value_or(gz::math::Vector3d::Zero);
    velocity.Z() = 0;
    const double actualSpeed = (pos-previous).Length()/dt;
    previous = pos;
    auto direction = route[goal]-pos;
    double range = direction.Length();
    if (!finished && range < p["range_tolerance"] && velocity.Length() < p["arrival_speed"])
    {
      if (goal+1 < route.size()) ++goal;
      else if (loop) goal = 0;
      else finished = true;
      direction = route[goal]-pos; range = direction.Length();
    }
    const double error = std::remainder(std::atan2(direction.Y(), direction.X())-
                                       pose.Rot().Yaw(), 2*M_PI);
    const double heading = std::max(0.0, 1.0-std::abs(error)/p["heading_gate"]);
    // Stop inside the acceptance disk before turning. Contact forces remain
    // authoritative: no pose commands or linear/angular velocity overrides.
    const double remaining = std::max(0.0, range-p["range_tolerance"]*.5);
    const double target = finished ? 0 : std::min({p["speed"],
        std::sqrt(2*p["braking"]*remaining), remaining/p["response_time"]})*heading;
    auto desired = range > 0 ? direction/range*target : gz::math::Vector3d::Zero;
    auto acceleration = (desired-velocity)/p["response_time"];
    const double limit = target < velocity.Length() ? p["braking"] : p["acceleration"];
    if (acceleration.Length() > limit) acceleration *= limit/acceleration.Length();
    auto force = acceleration*p["mass"];
    // Coulomb feedforward compensates the configured floor friction only
    // while a translation is requested; near stops feedback provides braking.
    if (target > p["gait_stop_speed"])
      force += desired/target*(p["friction"]*p["mass"]*p["gravity"]);
    if (force.Length() > p["max_force"]) force *= p["max_force"]/force.Length();
    const double omega = body.WorldAngularVelocity(_ecm).value_or(gz::math::Vector3d::Zero).Z();
    const double desiredOmega = finished ? 0 : std::clamp(error*p["yaw_gain"], -p["yaw_rate"], p["yaw_rate"]);
    const double torque = std::clamp(p["yaw_inertia"]*(desiredOmega-omega)/p["yaw_response_time"]+
        p["yaw_friction"]*std::tanh(desiredOmega/p["gait_stop_speed"]), -p["max_torque"], p["max_torque"]);
    body.AddWorldWrench(_ecm, force, {0, 0, torque});

    gaitSpeed += (actualSpeed-gaitSpeed)*std::min(1.0, dt/p["gait_response_time"]);
    const bool walking = gaitSpeed > p["gait_stop_speed"];
    if (walking) phase = std::fmod(phase+2*M_PI*actualSpeed*dt/p["stride_length"], 2*M_PI);
    animationTime += dt;
    if (animationTime < 1/p["animation_rate"]) return;
    animationTime = 0;
    const double amplitude = walking ? std::min(1.0, gaitSpeed/p["speed"]) : 0;
    for (const auto &visual : visuals)
    {
      auto value = visual.rest;
      const bool leg = visual.name.find("leg_") == 0 || visual.name.find("shoe_") == 0;
      const bool arm = visual.name.find("arm_") == 0;
      if (leg || arm)
      {
        const double side = visual.name.find("_-1_") != std::string::npos ? -1 : 1;
        const double angle = side*std::sin(phase)*amplitude*(leg ? p["leg_swing"] : -p["arm_swing"]);
        const gz::math::Vector3d pivot(0, value.Pos().Y(), leg ? p["leg_height"] : p["shoulder_height"]);
        const gz::math::Quaterniond rotation(0, angle, 0);
        value.Pos() = pivot+rotation.RotateVector(value.Pos()-pivot);
        value.Rot() = rotation*value.Rot();
      }
      else value.Pos().Z() += p["bob_height"]*amplitude*(1-std::cos(2*phase));
      _ecm.SetComponentData<gz::sim::components::Pose>(visual.entity, value);
      // SetComponentData alone updates sensor rendering, but does NOT mark
      // this component for serialized GUI state updates in Gazebo Sim 8.
      _ecm.SetChanged(visual.entity, gz::sim::components::Pose::typeId,
                      gz::sim::ComponentState::PeriodicChange);
    }
  }
};
}
GZ_ADD_PLUGIN(amr_simulation::HumanSystem, gz::sim::System,
              amr_simulation::HumanSystem::ISystemConfigure,
              amr_simulation::HumanSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(amr_simulation::HumanSystem, "amr_simulation::HumanSystem")
