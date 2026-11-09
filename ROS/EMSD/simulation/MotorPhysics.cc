// Gazebo force adapter (Harmonic or Fortress; see gz_compat.hh). Never sets vehicle pose or velocity.
#include <Python.h>
#include <dlfcn.h>
#include "gz_compat.hh"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace emsd
{
class MotorPhysics : public gzsim::System,
                     public gzsim::ISystemConfigure,
                     public gzsim::ISystemPreUpdate
{
  public: void Configure(const gzsim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gzsim::EntityComponentManager &_ecm,
      gzsim::EventManager &) override
  {
    link = gzsim::Link(
        gzsim::Model(_entity).LinkByName(_ecm, "base_link"));
    if (!link.Valid(_ecm))
      throw std::runtime_error("EMSD: base_link missing");
    link.EnableVelocityChecks(_ecm);
    // Gazebo loads system plugins with local symbols. Native Python modules
    // such as NumPy resolve CPython symbols from the global loader scope.
    static void *pythonHandle = [] {
      Dl_info pythonLibrary{};
      if (!dladdr(reinterpret_cast<void *>(&Py_Initialize), &pythonLibrary) ||
          !pythonLibrary.dli_fname)
        return static_cast<void *>(nullptr);
      return dlopen(pythonLibrary.dli_fname, RTLD_NOW | RTLD_GLOBAL);
    }();
    if (!pythonHandle)
      throw std::runtime_error("EMSD: could not expose embedded Python symbols");
    if (!Py_IsInitialized())
    {
      Py_Initialize();
      // Release the initial interpreter lock; Gazebo may run updates on
      // another thread. Each call below explicitly acquires the GIL.
      PyEval_SaveThread();
    }
    const auto gil = PyGILState_Ensure();
    auto path = PyUnicode_FromString(_sdf->Get<std::string>("python_path").c_str());
    PyList_Insert(PySys_GetObject("path"), 0, path);
    Py_DECREF(path);
    const auto moduleName = _sdf->HasElement("controller_module") ?
        _sdf->Get<std::string>("controller_module") : std::string("physics_control");
    auto module = PyImport_ImportModule(moduleName.c_str());
    auto cls = module ? PyObject_GetAttrString(module, "PhysicsController") : nullptr;
    const auto config = _sdf->Get<std::string>("config");
    runtime = cls ? PyObject_CallFunction(cls, "s", config.c_str()) : nullptr;
    Py_XDECREF(cls);
    Py_XDECREF(module);
    if (!runtime)
    {
      PyErr_Print();
      PyGILState_Release(gil);
      throw std::runtime_error("EMSD: Python controller initialization failed");
    }
    PyGILState_Release(gil);
  }

  public: ~MotorPhysics() override
  {
    if (runtime && Py_IsInitialized())
    {
      const auto gil = PyGILState_Ensure();
      Py_DECREF(runtime);
      PyGILState_Release(gil);
    }
    // Do not finalize a process-wide interpreter owned by other plugins.
  }

  public: void PreUpdate(const gzsim::UpdateInfo &_info,
      gzsim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !runtime)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0)
      throw std::runtime_error("EMSD: restart the runner after resetting time");
    const auto pose = gzsim::worldPose(link.Entity(), _ecm);
    const auto vel = link.WorldLinearVelocity(_ecm).value_or(gzmath::Vector3d::Zero);
    const auto omega = pose.Rot().RotateVectorReverse(
        link.WorldAngularVelocity(_ecm).value_or(gzmath::Vector3d::Zero));
    // State is the start of this integration step, rather than its end time.
    const double t = std::chrono::duration<double>(_info.simTime).count() - dt;
    const auto gil = PyGILState_Ensure();
    auto state = Py_BuildValue("(d(ddd)(dddd)(ddd)(ddd)d)", t,
        pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
        pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W(),
        vel.X(), vel.Y(), vel.Z(), omega.X(), omega.Y(), omega.Z(), dt);
    auto method = PyObject_GetAttrString(runtime, "update");
    auto result = method ? PyObject_CallObject(method, state) : nullptr;
    Py_XDECREF(method);
    Py_DECREF(state);
    if (!result)
    {
      PyErr_Print();
      PyGILState_Release(gil);
      throw std::runtime_error("EMSD: controller failed; stopping physics");
    }
    auto seq = PySequence_Fast(result, "Expected body force and torque (6 numbers)");
    Py_DECREF(result);
    double values[6]{};
    bool valid = seq && PySequence_Fast_GET_SIZE(seq) == 6;
    if (valid)
      for (int i = 0; i < 6; ++i)
      {
        values[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
        valid = valid && std::isfinite(values[i]);
      }
    valid = valid && !PyErr_Occurred();
    if (PyErr_Occurred()) PyErr_Print();
    Py_XDECREF(seq);
    PyGILState_Release(gil);
    if (!valid)
      throw std::runtime_error("EMSD: invalid controller wrench");
    // Python sums each r_i x F_i and signed propeller reaction torque.
    // The generated base_link origin is the CoM, so the wrench acts at CoM.
    link.AddWorldWrench(_ecm,
        pose.Rot().RotateVector(gzmath::Vector3d(values[0], values[1], values[2])),
        pose.Rot().RotateVector(gzmath::Vector3d(values[3], values[4], values[5])));
  }

  private: gzsim::Link link;
  private: PyObject *runtime{nullptr};
};
}

EMSD_ADD_PLUGIN(emsd::MotorPhysics, emsd::gzsim::System,
    emsd::gzsim::ISystemConfigure, emsd::gzsim::ISystemPreUpdate)
EMSD_ADD_PLUGIN_ALIAS(emsd::MotorPhysics, "emsd::MotorPhysics")
