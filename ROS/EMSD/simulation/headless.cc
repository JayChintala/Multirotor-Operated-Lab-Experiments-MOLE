// Run the same adapter and Gazebo DART physics backend without a GUI or transport.
// Useful for deterministic tests and machines where sockets are unavailable.
#include "MotorPhysics.cc"
#if EMSD_GZ_HARMONIC
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/Link.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/math/eigen3/Conversions.hh>
namespace emsd { namespace gzphysics = gz::physics; }
#else
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/Link.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/math/eigen3/Conversions.hh>
namespace emsd { namespace gzphysics = ignition::physics; }
#endif
#include <sdf/Root.hh>
#include <sdf/Physics.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>
#include <filesystem>

int main(int argc, char **argv)
{
  try
  {
    if (argc != 3)
      throw std::runtime_error("Usage: emsd_headless world.sdf duration_seconds");
    emsd::gzcommon::Console::log.Init(
        std::filesystem::path(argv[1]).parent_path().string(), "dart.log");
    namespace phys = emsd::gzphysics;
    namespace sim = emsd::gzsim;
    namespace comp = emsd::gzsim::components;
    using Features = phys::FeatureList<phys::sdf::ConstructSdfWorld,
        phys::ForwardStep, phys::GetModelFromWorld, phys::GetLinkFromModel,
        phys::LinkFrameSemantics, phys::AddLinkExternalForceTorque>;
    emsd::gzplugin::Loader loader;
    const auto names = loader.LoadLib(EMSD_DART_ENGINE);
    phys::Engine3dPtr<Features> engine;
    for (const auto &name : names)
    {
      engine = phys::RequestEngine3d<Features>::From(loader.Instantiate(name));
      if (engine) break;
    }
    if (!engine) throw std::runtime_error("Cannot load Gazebo DART engine");
    sdf::Root root;
    const auto errors = root.Load(argv[1]);
    if (!errors.empty())
    {
      for (const auto &error : errors) std::cerr << error << '\n';
      throw std::runtime_error("Invalid SDF");
    }
    const auto worldSdf = root.WorldByIndex(0);
    const auto world = engine->ConstructWorld(*worldSdf);
    if (!world) throw std::runtime_error("Cannot construct physics world");
    const auto physicalModel = world->GetModel("emsd_drone_frame");
    const auto body = physicalModel->GetLink("base_link");
    const auto modelSdf = worldSdf->ModelByName("emsd_drone_frame");
    const double dt = worldSdf->PhysicsByIndex(0)->MaxStepSize();
    const double duration = std::stod(argv[2]);
    if (!std::isfinite(duration) || duration <= 0)
      throw std::runtime_error("Headless duration must be finite and positive");
    sim::EntityComponentManager ecm;
    const auto model = ecm.CreateEntity();
    ecm.CreateComponent(model, comp::Model());
    ecm.CreateComponent(model, comp::Name("emsd_drone_frame"));
    ecm.CreateComponent(model, comp::Pose());
    const auto entity = ecm.CreateEntity();
    ecm.CreateComponent(entity, comp::Link());
    ecm.CreateComponent(entity, comp::Name("base_link"));
    ecm.CreateComponent(entity, comp::ParentEntity(model));
    ecm.CreateComponent(entity, comp::Pose());
    ecm.SetParentEntity(entity, model);
    sim::EventManager events;
    emsd::MotorPhysics adapter;
    adapter.Configure(model, modelSdf->Plugins().at(0).Element(), ecm, events);
    phys::ForwardStep::Input input;
    phys::ForwardStep::Output output;
    phys::ForwardStep::State state;
    sim::UpdateInfo info;
    info.paused = false;
    info.dt = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(dt));
    input.Get<std::chrono::steady_clock::duration>() = info.dt;
    const std::size_t count = static_cast<std::size_t>(std::ceil(duration/dt));
    for (std::size_t i = 0; i < count; ++i)
    {
      const auto data = body->FrameDataRelativeToWorld();
      ecm.Component<comp::Pose>(entity)->Data() = emsd::gzmath::eigen3::convert(data.pose);
      ecm.Component<comp::WorldLinearVelocity>(entity)->Data() = emsd::gzmath::eigen3::convert(data.linearVelocity);
      ecm.Component<comp::WorldAngularVelocity>(entity)->Data() = emsd::gzmath::eigen3::convert(data.angularVelocity);
      info.simTime = info.dt * (i+1);
      adapter.PreUpdate(info, ecm);
      const auto wrench = ecm.Component<comp::ExternalWorldWrenchCmd>(entity);
      if (!wrench) throw std::runtime_error("Missing physical wrench");
      const auto &w = wrench->Data();
      body->AddExternalForce(Eigen::Vector3d(w.force().x(), w.force().y(), w.force().z()));
      body->AddExternalTorque(Eigen::Vector3d(w.torque().x(), w.torque().y(), w.torque().z()));
      world->Step(output, state, input);
      ecm.RemoveComponent<comp::ExternalWorldWrenchCmd>(entity);
      if (i % static_cast<std::size_t>(1/dt) == 0)
        std::cout << "sim t=" << i*dt << " s, position=" << data.pose.translation().transpose() << std::endl;
    }
    std::cout << "Completed " << count << " DART physics steps (" << duration << " simulated seconds).\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "EMSD physics error: " << error.what() << std::endl;
    return 1;
  }
}
