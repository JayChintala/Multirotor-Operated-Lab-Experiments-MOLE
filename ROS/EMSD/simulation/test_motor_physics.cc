// Exercise the actual C++/Python adapter using a Gazebo ECM, without transport.
#include "MotorPhysics.cc"
#include <sdf/parser.hh>

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  namespace sim = emsd::gzsim;
  namespace comp = emsd::gzsim::components;
  sim::EntityComponentManager ecm;
  const auto model = ecm.CreateEntity();
  ecm.CreateComponent(model, comp::Model());
  ecm.CreateComponent(model, comp::Name("test_drone"));
  ecm.CreateComponent(model, comp::Pose());
  const auto body = ecm.CreateEntity();
  ecm.CreateComponent(body, comp::Link());
  ecm.CreateComponent(body, comp::Name("base_link"));
  ecm.CreateComponent(body, comp::ParentEntity(model));
  ecm.SetParentEntity(body, model);
  const emsd::gzmath::Pose3d pose(0, 0, .15, .1, 0, 0);
  ecm.CreateComponent(body, comp::Pose(pose));
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::init(sdf);
  const std::string xml = std::string("<sdf version='1.8'><model name='test'><plugin ") +
    "filename='test' name='emsd::MotorPhysics'><python_path>" + argv[1] +
    "</python_path><config>" + argv[2] + "</config></plugin></model></sdf>";
  if (!sdf::readString(xml, sdf)) return 3;
  sim::EventManager events;
  emsd::MotorPhysics plugin;
  plugin.Configure(model, sdf->Root()->GetElement("model")->GetElement("plugin"), ecm, events);
  sim::UpdateInfo info;
  info.paused = false;
  info.dt = std::chrono::milliseconds(2);
  info.simTime = info.dt;
  plugin.PreUpdate(info, ecm);
  auto wrench = ecm.Component<comp::ExternalWorldWrenchCmd>(body);
  if (!wrench) throw std::runtime_error("Adapter did not create a physical wrench");
  const auto &w = wrench->Data();
  if (!(w.force().z() > 0 && w.force().y() < 0 && w.torque().x() < 0))
    throw std::runtime_error("Wrong body-to-world force or corrective roll torque");
  std::cout << "PASS: Python feedback/allocation -> C++ world force and corrective torque.\n";
  return 0;
}
