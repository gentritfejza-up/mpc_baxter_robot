#pragma once

/**
 * @file geometry_rows.hpp
 * @brief Per-cycle linearisation of the collision-guard distances into parametric
 *        linear rows for the MPC QP, the post-solve guard that evaluates the true
 *        distances along the predicted trajectory, and the reference projection.
 *
 * Row for a distance constraint d(q) >= m, linearised at the point qbar with the
 * gradient g = n^T J (n: distance normal, J: 3x7 sphere-centre Jacobian):
 *
 *     g^T X_k + s >= m - d(qbar) + g^T qbar
 *
 * The coefficient vector g goes into the QP parameters, the right-hand side into
 * the per-cycle lower bound; an inactive slot has g = 0 and lower bound -inf, and
 * s >= 0 is the quadratic slack of the sphere at that step. The QP applies the row
 * to X_k + tau V_k (robot/geometry/velocity_lookahead_sec): the point tau seconds
 * ahead must clear the margin, which limits the approach speed near a keep-out and
 * removes the re-linearisation chatter.
 *
 * Slot layout per prediction step and sphere: [rows_floor][rows_body][rows_other_arm].
 *
 * @author Gentrit Fejza, University of Prishtina (Master's thesis, 2026)
 */

#include "mpc_controller/arm_collision_model.hpp"

#include <array>
#include <limits>
#include <string>
#include <vector>

namespace mpc_geometry {

/** @brief Index of a constraint group in the per-group distance arrays. */
enum GroupIndex { kFloorGroup = 0, kBodyGroup = 1, kArmGroup = 2, kGroupCount = 3 };

/** @brief Distances per constraint group [m]; +inf when the group has no obstacle. */
using GroupDistances = std::array<double, kGroupCount>;

/** @brief GroupDistances with every entry at +infinity. */
inline GroupDistances infiniteGroupDistances() {
  GroupDistances distances;
  distances.fill(std::numeric_limits<double>::infinity());
  return distances;
}

/**
 * @brief The other arm as this controller expects it over the horizon.
 *
 * Index 0 is the measured state now, index k (1..N) the state assumed at prediction
 * step k (constant-velocity extrapolation, capped in time, by the node). Empty: unknown.
 */
using OtherArmTrajectory = std::vector<JointVector>;

/** @brief Collision-guard parameters (the robot/geometry parameter namespace). */
struct GeometryParams {
  double z_min{-0.80};  ///< Floor height in the base frame [m].
  /// @name Row margins [m]
  /// Each must exceed guard_margin + guard_worsening_tolerance: the guard rejects
  /// plans below guard_margin, so a row that only asks for a smaller clearance would
  /// have every plan near that obstacle rejected (see validate()).
  /// @{
  double floor_margin{0.03};
  double body_margin{0.04};
  double arm_margin{0.10};
  /// @}
  double activation_distance{0.15};  ///< Rows farther than this at qbar stay inactive [m].
  /// Row margins grow with the step's time into the horizon [m/s]: the linearisation
  /// error is largest late in the horizon, and the guard checks the true distance
  /// there, so late steps keep extra clearance (0: constant margins).
  double margin_growth_per_sec{0.02};
  /// A row never asks for more than this much distance recovery over the horizon [m].
  /// Large requested corrections mean large slacks, which stall OSQP; capping keeps
  /// the slacks near zero and lets the arm climb out of a violation progressively.
  double max_recovery_per_horizon{0.05};
  int rows_floor{1};      ///< Floor slots per sphere and step (0 or 1).
  int rows_body{1};       ///< Body slots per sphere and step (nearest obstacles).
  int rows_other_arm{1};  ///< Other-arm slots per sphere and step (nearest spheres).
  int step_stride{1};     ///< 1: rows on every step; k: every k-th step plus the last.
  double guard_margin{0.015};               ///< The guard rejects predicted distances below this [m].
  double guard_worsening_tolerance{0.005};  ///< Allowed worsening relative to the current distance [m].

  /**
   * @brief Reads the robot/geometry parameters and validates.
   * @param node_handle Node handle used for the parameter lookups.
   */
  static GeometryParams fromParams(ros::NodeHandle& node_handle);
  /** @brief Throws std::invalid_argument on an inconsistent parameter set. */
  void validate() const;
};

/** @brief Dimensions of the geometry block of the QP and the row indexing. */
struct GeometryLayout {
  int prediction_horizon{0};  ///< N, prediction steps.
  int sphere_count{0};        ///< Spheres per arm.
  int rows_floor{0};          ///< Floor slots per sphere and step.
  int rows_body{0};           ///< Body slots per sphere and step.
  int rows_other_arm{0};      ///< Other-arm slots per sphere and step.
  std::vector<double> step_times;  ///< Time of step k from now [s] at index k-1; empty: all 0.

  /**
   * @brief Time of a prediction step from now.
   * @param step Prediction step, 1..N.
   * @return Time [s]; 0 when no step times are set.
   */
  double timeAt(int step) const {
    return step_times.empty() ? 0.0 : step_times[static_cast<std::size_t>(step - 1)];
  }
  /** @brief Rows per sphere and step. */
  int slotsPerSphere() const { return rows_floor + rows_body + rows_other_arm; }
  /** @brief Rows per prediction step. */
  int slotsPerStep() const { return sphere_count * slotsPerSphere(); }
  /** @brief Total geometry rows of the QP. */
  int rowCount() const { return prediction_horizon * slotsPerStep(); }
  /** @brief Length of the row-major coefficient vector (rowCount() * 7). */
  int coefficientCount() const { return rowCount() * kArmJoints; }
  /**
   * @brief Row index of one slot.
   * @param step Prediction step, 1..N.
   * @param sphere Sphere index, 0..sphere_count-1.
   * @param slot Slot within the sphere, 0..slotsPerSphere()-1.
   */
  int rowIndex(int step, int sphere, int slot) const {
    return (step - 1) * slotsPerStep() + sphere * slotsPerSphere() + slot;
  }
};

/** @brief The linearised rows of one control cycle, in the layout of GeometryLayout. */
struct GeometryRowBatch {
  std::vector<double> coefficients;  ///< rowCount() * 7 row gradients, row-major [m/rad].
  std::vector<double> lower_bounds;  ///< rowCount() right-hand sides [m]; -inf when inactive.
  int active_rows{0};                ///< Rows written this cycle.
  int degenerate_normals{0};         ///< Active rows whose distance normal was a fallback.
  GroupDistances min_linearised_distance{infiniteGroupDistances()};  ///< Minimum distance at the linearisation points per group [m].

  /**
   * @brief Sizes the vectors for `layout` and marks every row inactive.
   * @param layout Geometry block layout.
   */
  void reset(const GeometryLayout& layout);
};

/** @brief Result of the post-solve guard along a predicted trajectory. */
struct GuardReport {
  GroupDistances min_distance{infiniteGroupDistances()};      ///< Along the predicted trajectory [m].
  GroupDistances min_distance_now{infiniteGroupDistances()};  ///< At the measured state [m].
  int worst_step{-1};          ///< Prediction step of the worst violation, 1..N.
  int worst_sphere{-1};        ///< Sphere index of the worst violation.
  int worst_group{-1};         ///< GroupIndex of the worst violation.
  std::string worst_obstacle;  ///< Name of the obstacle of the worst violation.
  double worst_distance{std::numeric_limits<double>::infinity()};  ///< Distance of the worst violation [m].
  bool violated{false};        ///< True when the plan must be rejected.
};

/**
 * @brief Linearises the collision distances into QP rows, evaluates the guard and
 *        projects references out of keep-outs. Not thread-safe (scratch members).
 */
class GeometryRowLinearizer {
 public:
  /**
   * @param model Sphere collision model of both arms.
   * @param side Arm this linearizer serves, "left" or "right".
   * @param params Collision-guard parameters.
   * @param layout Geometry block layout; must match `model` and `params`.
   */
  GeometryRowLinearizer(const ArmCollisionModel& model, const std::string& side,
                        const GeometryParams& params, const GeometryLayout& layout);

  /** @brief Geometry block layout this linearizer writes. */
  const GeometryLayout& layout() const { return layout_; }
  /**
   * @brief Enables or disables the floor rows and the floor guard group.
   *
   * The floor can be held off e.g. until the arm has reached its home position (the
   * simulated rest pose hangs below any belt).
   */
  void setFloorEnabled(bool enabled) { floor_enabled_ = enabled; }
  /** @brief True while the floor rows and guard group are active. */
  bool floorEnabled() const { return floor_enabled_; }
  /** @brief Arm this linearizer serves. */
  const std::string& side() const { return side_; }
  /** @brief The opposite arm. */
  const std::string& otherSide() const { return other_side_; }

  /**
   * @brief Writes the rows of one cycle.
   * @param linearisation_points One joint vector per prediction step (1..N) [rad].
   * @param other_arm N+1 entries (see OtherArmTrajectory) or empty: every other-arm slot inactive.
   * @param[out] batch Reset and filled.
   */
  void linearize(const std::vector<JointVector>& linearisation_points,
                 const OtherArmTrajectory& other_arm, GeometryRowBatch& batch) const;
  /**
   * @brief Writes the rows of one cycle with a static other arm.
   * @param linearisation_points One joint vector per prediction step (1..N) [rad].
   * @param other_arm_positions Joint positions of the other arm [rad], or nullptr: unknown.
   * @param[out] batch Reset and filled.
   */
  void linearize(const std::vector<JointVector>& linearisation_points,
                 const JointVector* other_arm_positions, GeometryRowBatch& batch) const;

  /**
   * @brief True (nonlinear) distances now and along a predicted trajectory.
   *
   * A violation is a predicted distance below guard_margin that is also below the
   * arm's current distance to the same obstacle (minimum over the spheres) minus
   * guard_worsening_tolerance: an arm inside an obstacle may move out, never in.
   * @param measured_positions Measured joint positions [rad].
   * @param predicted One joint vector per predicted step [rad].
   * @param other_arm N+1 entries (see OtherArmTrajectory) or empty.
   * @return Minimum distances and the worst violation, if any.
   */
  GuardReport evaluateTrajectory(const JointVector& measured_positions,
                                 const std::vector<JointVector>& predicted,
                                 const OtherArmTrajectory& other_arm) const;
  /** @brief evaluateTrajectory() with a static other arm (nullptr: unknown). */
  GuardReport evaluateTrajectory(const JointVector& measured_positions,
                                 const std::vector<JointVector>& predicted,
                                 const JointVector* other_arm_positions) const;

  /**
   * @brief Moves a reference out of every keep-out it violates.
   *
   * Gauss-Newton steps along the row gradients, each fixing the worst sphere/obstacle
   * pair (floor when enabled, body, the other arm as measured now), until every pair
   * clears its margin by `extra_clearance` or `max_iterations` is reached. The MPC then
   * tracks a reachable set point at the margin instead of pushing into it, which keeps
   * the QP easy and the arm at rest there.
   * @param reference_positions Reference joint positions [rad].
   * @param other_arm Other-arm trajectory; only index 0 (now) is used. Empty: unknown.
   * @param[out] max_shift Largest |dq| over the joints [rad]; may be nullptr.
   * @param max_iterations Gauss-Newton step limit.
   * @param extra_clearance Clearance requested beyond the margin [m].
   * @param lower_bounds Optional joint lower limits (7) [rad] applied after every step, so the
   *        projection never trades a keep-out for a joint limit.
   * @param upper_bounds Optional joint upper limits (7) [rad].
   * @return The projected reference [rad].
   */
  JointVector projectReference(const JointVector& reference_positions,
                               const OtherArmTrajectory& other_arm, double* max_shift = nullptr,
                               int max_iterations = 8, double extra_clearance = 0.005,
                               const std::vector<double>* lower_bounds = nullptr,
                               const std::vector<double>* upper_bounds = nullptr) const;

 private:
  /** @brief One sphere/obstacle distance considered for a row slot. */
  struct Candidate {
    double distance;         ///< Surface distance [m].
    int obstacle_index;      ///< Body obstacle index or other-arm sphere index.
    Eigen::Vector3d normal;  ///< d(distance)/d(centre).
    bool degenerate;         ///< Normal is a fallback.
  };

  bool stepActive(int step) const;
  double bodyMargin(const SphereSpec& sphere) const;
  /// Margin of a row at `step`: grows with the step time (margin_growth_per_sec).
  double stepMargin(double margin, int step) const {
    return margin + params_.margin_growth_per_sec * layout_.timeAt(step);
  }
  /// Fills other_arm_centers_[0..N] (empty trajectory: none).
  void computeOtherArmCenters(const OtherArmTrajectory& other_arm) const;
  OtherArmTrajectory staticOtherArm(const JointVector* other_arm_positions) const;
  /// Distances of one sphere to every body obstacle it checks.
  void bodyCandidates(const SphereSpec& spec, const Eigen::Vector3d& center,
                      std::vector<Candidate>& candidates) const;
  /// Distances of one sphere to every other-arm sphere at `step`.
  void armCandidates(const SphereSpec& spec, const Eigen::Vector3d& center, int step,
                     std::vector<Candidate>& candidates) const;
  /// Keeps the `slot_count` nearest candidates, ordered by obstacle index (stable slot assignment).
  static void keepNearest(std::vector<Candidate>& candidates, int slot_count);
  void writeRow(const SphereState& state, const JointVector& linearisation_point,
                const Candidate& candidate, double margin, int row, GeometryRowBatch& batch) const;

  const ArmCollisionModel& model_;
  std::string side_;
  std::string other_side_;
  GeometryParams params_;
  GeometryLayout layout_;
  Obstacle floor_;
  bool floor_enabled_{true};
  mutable std::vector<SphereState> sphere_states_;        ///< Scratch: this arm, with Jacobians.
  mutable std::vector<Eigen::Vector3d> sphere_centers_;   ///< Scratch: this arm, centres only.
  mutable std::vector<std::vector<Eigen::Vector3d>> other_arm_centers_;  ///< Scratch: [step][sphere].
  mutable bool has_other_arm_centers_{false};
  mutable std::vector<Candidate> candidates_;
  mutable std::vector<double> current_distance_per_obstacle_;  ///< Scratch for evaluateTrajectory().
};

}  // namespace mpc_geometry
