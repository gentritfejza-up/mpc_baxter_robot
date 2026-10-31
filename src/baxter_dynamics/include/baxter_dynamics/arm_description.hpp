/**
 * @file arm_description.hpp
 * @brief Description of a serial manipulator for the Lagrange-Euler model: the
 *        classic Denavit-Hartenberg table, joint types and limits, and the
 *        inertial parameters of every link, either built in code or loaded
 *        from the ROS parameter server.
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 *
 * The description is plain data without any ROS dependency so that models can
 * be unit-tested on synthetic arms (see `test/planar_arm_test.cpp`).
 * `loadArmDescription()` is the only function that touches the parameter
 * server; its YAML layout is documented in `config/dynamics_params.yaml`.
 */
#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <vector>

namespace ros {
class NodeHandle;
}

namespace baxter {

/// Kind of a one-degree-of-freedom joint in the DH chain.
enum class JointType {
  kRevolute,   ///< Joint variable adds to theta (rotation about z_{i-1}).
  kPrismatic,  ///< Joint variable adds to d (translation along z_{i-1}).
};

/**
 * @brief Classic Denavit-Hartenberg row of one joint.
 *
 * The transform from link frame i-1 to link frame i is
 * `Rz(theta) Tz(d) Tx(a) Rx(alpha)` with `theta = theta_offset + q_i` for a
 * revolute joint and `d = d_offset + q_i` for a prismatic joint.
 */
struct DhParameters {
  double d_offset{0.0};      ///< Constant offset along the previous z axis [m].
  double theta_offset{0.0};  ///< Constant rotation about the previous z axis [rad].
  double a{0.0};             ///< Length along the rotated x axis [m].
  double alpha{0.0};         ///< Twist about the rotated x axis [rad].
};

/**
 * @brief Inertial parameters of one link, expressed in that link's DH frame.
 *
 * The inertia tensor is taken about the centre of mass with the URDF sign
 * convention (I_xy = -int x y dm).
 */
struct LinkInertialParameters {
  double mass{0.0};                                       ///< [kg]
  Eigen::Vector3d centre_of_mass{Eigen::Vector3d::Zero()}; ///< In the link frame [m].
  double Ixx{0.0}, Iyy{0.0}, Izz{0.0};  ///< Moments of inertia about the COM [kg m^2].
  double Ixy{0.0}, Iyz{0.0}, Izx{0.0};  ///< Products of inertia, URDF sign [kg m^2].
};

/// Everything one joint of the chain carries.
struct JointDescription {
  std::string name;                              ///< Full joint name, e.g. `left_s0`.
  JointType type{JointType::kRevolute};
  DhParameters dh;
  double lower_limit{-1.0e9};                    ///< Position limit [rad] or [m].
  double upper_limit{1.0e9};
  double effort_limit{1.0e9};                    ///< Symmetric torque/force limit [Nm] or [N].
  double viscous_damping{0.0};                   ///< b in tau = b qd [Nm s/rad].
  LinkInertialParameters link;                   ///< The link moved by this joint.
};

/**
 * @brief Serial chain of joints plus the gravity vector of its base frame.
 *
 * `validate()` throws `std::invalid_argument` on an empty chain, duplicate
 * joint names, a non-finite value, a negative mass, an inverted position
 * limit or a non-positive effort limit, so a validated description is always
 * usable by the model.
 */
struct ArmDescription {
  std::string name;                         ///< Arm name, e.g. `left`, used in logs.
  std::vector<JointDescription> joints;     ///< Base to tip.
  /// Gravitational acceleration in the DH base frame (frame 0) [m/s^2].
  Eigen::Vector3d gravity{0.0, 0.0, -9.81};

  std::size_t jointCount() const { return joints.size(); }
  std::vector<std::string> jointNames() const;
  /// @throw std::invalid_argument when the description is inconsistent.
  void validate() const;

  /**
   * @brief Lump a rigid payload into the last link (a held object, or the
   *        simulator's gripper fingers that the URDF-based model lumps too).
   * @param payload Mass, centre of mass and inertia tensor about that centre,
   *        expressed in the last link's DH frame.
   * @throw std::invalid_argument on an empty chain or a negative payload mass.
   */
  void addTipPayload(const LinkInertialParameters& payload);
};

/**
 * @brief Rigid-body composition of two bodies given in the same frame.
 * @return Mass, common centre of mass and inertia tensor about it (parallel-axis theorem).
 */
LinkInertialParameters combineBodies(const LinkInertialParameters& first,
                                     const LinkInertialParameters& second);

/**
 * @brief Read an arm description from the parameter server.
 *
 * Layout under @p parameter_namespace (see `config/dynamics_params.yaml`):
 *  - `joints/<arm>`: ordered joint names;
 *  - `gravity`: optional 3-vector, default (0, 0, -9.81);
 *  - per-joint tables keyed by the joint name with the `<arm>_` prefix
 *    stripped (or by the full name): `dh_parameters/<key>` = [d, theta, a,
 *    alpha], `angle_limits/<key>` = [lower, upper], `effort_limits/<key>`,
 *    optional `joint_types/<key>` ("revolute" | "prismatic") and
 *    `viscous_damping/<key>`;
 *  - per-link lists in joint order under `links/`: `mass`, `com_x`, `com_y`,
 *    `com_z`, `Ixx`, `Iyy`, `Izz`, `Ixy`, `Iyz`, `Izx`;
 *  - optional `tip_payload/{enabled,mass,com,inertia}` (com = [x, y, z] in the
 *    last link's frame, inertia = [Ixx, Iyy, Izz, Ixy, Iyz, Izx] about the
 *    payload's centre of mass), lumped into the last link when `enabled`.
 *
 * @param nh Node handle used for the lookup.
 * @param parameter_namespace Namespace of the tables, e.g. `robot/arm_model`.
 * @param arm Arm name, the key of the joint list and the prefix of its joints.
 * @return The validated description.
 * @throw std::runtime_error when a required parameter is missing or malformed.
 * @throw std::invalid_argument when the loaded description is inconsistent.
 */
ArmDescription loadArmDescription(ros::NodeHandle& nh, const std::string& parameter_namespace,
                                  const std::string& arm);

}  // namespace baxter
