#include "rover_sim_gazebo/terrain_preset_plugin.hpp"

#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/msgs/msgs.hh>

namespace rover_sim_gazebo
{

static double ReadDouble(sdf::ElementPtr sdf, const std::string &name, double def)
{
  if (!sdf || !sdf->HasElement(name)) return def;
  return sdf->Get<double>(name);
}

static std::string ReadString(sdf::ElementPtr sdf, const std::string &name, const std::string &def)
{
  if (!sdf || !sdf->HasElement(name)) return def;
  return sdf->Get<std::string>(name);
}

void TerrainPresetPlugin::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  world_ = world;

  const std::string target_model = ReadString(sdf, "target_model", "terrain");
  const std::string target_link  = ReadString(sdf, "target_link", "ground");
  const std::string preset       = ReadString(sdf, "preset", "regolith_nominal");

  // Defaults (if preset system not yet wired to YAML)
  double mu = 0.70;
  double mu2 = 0.70;
  double restitution = 0.05;

  // Minimal preset selection (hard-coded until YAML loader is introduced)
  if (preset == "regolith_loose") {
    mu = 0.55; mu2 = 0.55; restitution = 0.03;
  } else if (preset == "bedrock_hard") {
    mu = 0.95; mu2 = 0.95; restitution = 0.10;
  }

  auto model = world_->ModelByName(target_model);
  if (!model) {
    gzerr << "[TerrainPresetPlugin] target model not found: " << target_model << "\n";
    return;
  }

  ApplyFriction(model, target_link, mu, mu2, restitution);
  gzmsg << "[TerrainPresetPlugin] Applied preset '" << preset
        << "' to model=" << target_model << " link=" << target_link
        << " mu=" << mu << " mu2=" << mu2 << " restitution=" << restitution << "\n";
}

void TerrainPresetPlugin::ApplyFriction(gazebo::physics::ModelPtr model,
                                       const std::string &link_name,
                                       double mu,
                                       double mu2,
                                       double restitution)
{
  auto link = model->GetLink(link_name);
  if (!link) {
    gzerr << "[TerrainPresetPlugin] link not found: " << link_name << "\n";
    return;
  }

  auto cols = link->GetCollisions();
  if (cols.empty()) {
    gzerr << "[TerrainPresetPlugin] no collisions on link: " << link_name << "\n";
    return;
  }

  for (auto &c : cols) {
    if (!c) continue;

    auto surf = c->GetSurface();
    if (!surf) continue;

    // ODE friction parameters
    auto fric = surf->FrictionPyramid();
    if (fric) {
      fric->SetMuPrimary(mu);
      fric->SetMuSecondary(mu2);
    }

    // Gazebo 11 exposes restitution through its Surface message API.
    // Round-trip the existing surface so other contact parameters are preserved.
    gazebo::msgs::Surface surface_msg;
    surf->FillMsg(surface_msg);
    surface_msg.set_restitution_coefficient(restitution);
    surf->ProcessMsg(surface_msg);
  }
}

GZ_REGISTER_WORLD_PLUGIN(TerrainPresetPlugin)

}  // namespace rover_sim_gazebo
