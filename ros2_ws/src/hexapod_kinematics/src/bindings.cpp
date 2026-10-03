// The door between Python and the leg library.
//
// No triangle lives here. The 20 ms loop speaks Python lists of
// numbers. solve_pose and gate speak C++ structs. This file carries
// the numbers across and carries the decision back.
//
// A foot the solver refuses comes back as a PoseResult with ok false.
// A pose the gate refuses comes back as a SafetyDecision with allow
// false. Neither call raises for those answers. An exception here
// would fall into the step loop's handler and drop the generator.
// The wrong count of feet is not one of those answers: that raises
// ValueError, because the caller did not hand over a pose.

#include "hexapod_kinematics/safety.hpp"

#include <pybind11/pybind11.h>

#include <array>
#include <string>

namespace py = pybind11;
namespace hk = hexapod_kinematics;

namespace {

// A Python number, including an int written where a millimetre belongs.
// A list or a string is not a number. The message names the slot.
double as_double(py::handle value, const char* what) {
  try {
    return value.cast<double>();
  } catch (const py::cast_error&) {
    throw py::value_error(what);
  }
}

bool is_triple(py::handle value) {
  if (py::isinstance<py::str>(value) || !py::isinstance<py::sequence>(value)) {
    return false;
  }
  return value.cast<py::sequence>().size() == 3;
}

// Six feet, each (x, y, z) millimetres, leg 1 first. This is the shape
// set_pose_base already passes to the vendor call.
std::array<hk::Vec3, hk::kLegCount> feet_from_python(py::sequence feet) {
  if (py::isinstance<py::str>(feet) || feet.size() != static_cast<std::size_t>(hk::kLegCount)) {
    throw py::value_error("solve_pose wants 6 feet, each an x, y, z in millimetres");
  }
  std::array<hk::Vec3, hk::kLegCount> out{};
  for (int i = 0; i < hk::kLegCount; ++i) {
    py::handle foot = feet[i];
    if (!is_triple(foot)) {
      throw py::value_error("each foot is x, y, z in millimetres");
    }
    py::sequence xyz = foot.cast<py::sequence>();
    out[static_cast<std::size_t>(i)].x_mm = as_double(xyz[0], "foot x is a number of millimetres");
    out[static_cast<std::size_t>(i)].y_mm = as_double(xyz[1], "foot y is a number of millimetres");
    out[static_cast<std::size_t>(i)].z_mm = as_double(xyz[2], "foot z is a number of millimetres");
  }
  return out;
}

void store_joint(std::array<hk::JointAngles, hk::kLegCount>& pose, int zero_based, double radians) {
  hk::JointAngles& leg = pose[static_cast<std::size_t>(zero_based / 3)];
  switch (zero_based % 3) {
    case 0:
      leg.coxa_rad = radians;
      break;
    case 1:
      leg.femur_rad = radians;
      break;
    default:
      leg.tibia_rad = radians;
      break;
  }
}

// Eighteen radians in kinematic order, or six (coxa, femur, tibia) triples.
// Joint 1 is the front-left coxa. None is not handled here.
std::array<hk::JointAngles, hk::kLegCount> angles_from_python(py::handle value) {
  if (py::isinstance<py::str>(value) || !py::isinstance<py::sequence>(value)) {
    throw py::value_error("angles are 18 radians, or 6 triples of coxa, femur, tibia");
  }
  py::sequence seq = value.cast<py::sequence>();
  std::array<hk::JointAngles, hk::kLegCount> out{};
  if (seq.size() == static_cast<std::size_t>(hk::kJointCount) && !is_triple(seq[0])) {
    for (int i = 0; i < hk::kJointCount; ++i) {
      store_joint(out, i, as_double(seq[i], "a joint angle is a number of radians"));
    }
    return out;
  }
  if (seq.size() == static_cast<std::size_t>(hk::kLegCount)) {
    for (int leg = 0; leg < hk::kLegCount; ++leg) {
      if (!is_triple(seq[leg])) {
        throw py::value_error("each leg is coxa, femur, tibia in radians");
      }
      py::sequence triple = seq[leg].cast<py::sequence>();
      store_joint(out, leg * 3 + 0, as_double(triple[0], "coxa is a number of radians"));
      store_joint(out, leg * 3 + 1, as_double(triple[1], "femur is a number of radians"));
      store_joint(out, leg * 3 + 2, as_double(triple[2], "tibia is a number of radians"));
    }
    return out;
  }
  throw py::value_error("angles are 18 radians, or 6 triples of coxa, femur, tibia");
}

py::tuple angles_to_python(const std::array<hk::JointAngles, hk::kLegCount>& angles) {
  py::tuple legs(hk::kLegCount);
  for (int i = 0; i < hk::kLegCount; ++i) {
    const hk::JointAngles& leg = angles[static_cast<std::size_t>(i)];
    legs[i] = py::make_tuple(leg.coxa_rad, leg.femur_rad, leg.tibia_rad);
  }
  return legs;
}

}  // namespace

PYBIND11_MODULE(hexapod_kinematics, m) {
  // The name Python imports. One tick calls solve_pose, then gate.
  m.doc() =
      "Leg inverse kinematics for hunter gait 15. "
      "Feet are millimetres, body frame, +X head, +Y left, +Z up. "
      "Angles are radians in the set_leg_position sense. "
      "A refusal is a returned result, not an exception.";

  py::class_<hk::PoseResult>(m, "PoseResult")
      .def_readonly("ok", &hk::PoseResult::ok)
      .def_property_readonly("reason",
                             [](const hk::PoseResult& result) {
                               return std::string(hk::ik_reason_name(result.reason));
                             })
      .def_property_readonly("leg",
                             [](const hk::PoseResult& result) {
                               return static_cast<int>(result.leg);
                             })
      .def_property_readonly("angles",
                             [](const hk::PoseResult& result) {
                               return angles_to_python(result.angles);
                             })
      .def("__repr__", [](const hk::PoseResult& result) {
        return "<PoseResult ok=" + std::string(result.ok ? "True" : "False") +
               " reason='" + hk::ik_reason_name(result.reason) +
               "' leg=" + std::to_string(static_cast<int>(result.leg)) + ">";
      });

  py::class_<hk::SafetyDecision>(m, "SafetyDecision")
      .def_readonly("allow", &hk::SafetyDecision::allow)
      .def_property_readonly("reason",
                             [](const hk::SafetyDecision& decision) {
                               return std::string(hk::ik_reason_name(decision.reason));
                             })
      .def_readonly("joint_id", &hk::SafetyDecision::joint_id)
      .def_readonly("q_now_rad", &hk::SafetyDecision::q_now_rad)
      .def_readonly("q_previous_rad", &hk::SafetyDecision::q_previous_rad);

  // The stand-in step, radians. solve_pose uses this. The gate uses
  // this. The tick does not pass a second copy.
  m.def("max_step_rad", &hk::max_step_rad);

  // Six feet in. One pose decision out. The step limit is max_step_rad().
  m.def(
      "solve_pose",
      [](py::sequence feet) {
        return hk::solve_pose(feet_from_python(feet), hk::max_step_rad());
      },
      py::arg("feet"));

  // Build an ok command from angles the caller already has, so the gate
  // can be offered a pose that did not come from solve_pose. This does
  // not solve a foot. The tick uses solve_pose.
  m.def(
      "command_from_angles",
      [](py::handle angles) {
        hk::PoseResult command;
        command.ok = true;
        command.reason = hk::IkReason::None;
        command.angles = angles_from_python(angles);
        return command;
      },
      py::arg("angles"));

  // previous is the last pose this gate allowed the caller to send.
  // None means there is no such pose. Eighteen radians, or six triples.
  m.def(
      "gate",
      [](const hk::PoseResult& command, py::object previous) {
        if (previous.is_none()) {
          return hk::gate(command, nullptr);
        }
        const std::array<hk::JointAngles, hk::kLegCount> sent = angles_from_python(previous);
        return hk::gate(command, &sent);
      },
      py::arg("command"), py::arg("previous") = py::none());
}
