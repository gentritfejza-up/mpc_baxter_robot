#include "baxter_kdl_dynamics/kdl_inverse_dynamics.hpp"

#include <baxter_interface_cpp/limb.hpp>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kdl_inverse_dynamics_node");
  if (argc < 2 || (std::string(argv[1]) != "left" && std::string(argv[1]) != "right")) {
    ROS_ERROR("Usage: run_kdl_inverse_dynamics <left|right>");
    return EXIT_FAILURE;
  }

  try {
    ros::NodeHandle nh;
    const std::string limb_name(argv[1]);
    Limb limb(limb_name);
    baxter_kdl_dynamics::KdlInverseDynamics dynamics(nh, limb_name);

    int spinner_threads = 2;
    double loop_rate_hz = 800.0;
    double diagnostics_rate_hz = 50.0;
    double acceleration_timeout_sec = 0.2;
    nh.param("robot/control/spinner_threads", spinner_threads, spinner_threads);
    nh.param("robot/control/kdl_dynamics_rate_hz", loop_rate_hz, loop_rate_hz);
    nh.param("robot/control/kdl_diagnostics_rate_hz", diagnostics_rate_hz,
             diagnostics_rate_hz);
    nh.param("robot/control/acceleration_ref_timeout_sec", acceleration_timeout_sec,
             acceleration_timeout_sec);
    if (spinner_threads <= 0 || !std::isfinite(loop_rate_hz) || loop_rate_hz <= 0.0 ||
        !std::isfinite(diagnostics_rate_hz) || diagnostics_rate_hz <= 0.0 ||
        diagnostics_rate_hz > loop_rate_hz ||
        !std::isfinite(acceleration_timeout_sec) || acceleration_timeout_sec <= 0.0) {
      throw std::runtime_error("Invalid KDL dynamics control timing parameters");
    }
    limb.setCommandTimeout(acceleration_timeout_sec);
    ros::Publisher diagnostics_publisher = nh.advertise<std_msgs::Float64MultiArray>(
        "/robot/limb/" + limb_name + "/kdl_dynamics_diagnostics", 2);

    ros::AsyncSpinner spinner(spinner_threads);
    spinner.start();
    ros::Rate rate(loop_rate_hz);
    bool torque_control_active = false;
    uint64_t sequence = 0;
    const double diagnostics_period_sec = 1.0 / diagnostics_rate_hz;
    auto previous_loop_time = std::chrono::steady_clock::now();
    auto previous_diagnostics_time = previous_loop_time;
    while (ros::ok()) {
      const auto loop_start = std::chrono::steady_clock::now();
      const double loop_dt_ms =
          std::chrono::duration<double, std::milli>(loop_start - previous_loop_time).count();
      previous_loop_time = loop_start;
      const auto positions = limb.jointAngles();
      const auto velocities = limb.jointVelocities();
      const auto accelerations = limb.jointAccelerationsRef();
      if (positions.size() != 7 || velocities.size() != 7 || accelerations.size() != 7) {
        ROS_WARN_THROTTLE(1.0, "Waiting for complete joint state and acceleration reference");
      } else if (limb.accelerationRefAgeSec() > acceleration_timeout_sec) {
        ROS_WARN_THROTTLE(1.0, "Acceleration references are stale; exiting torque control");
        if (torque_control_active) {
          limb.exitControlMode(acceleration_timeout_sec);
          torque_control_active = false;
        }
      } else {
        try {
          const auto result = dynamics.compute(positions, velocities, accelerations,
                                               limb.jointAnglesRef(), limb.jointVelocitiesRef());
          limb.setJointTorques(result.torques);
          torque_control_active = true;
          ++sequence;
          const auto compute_end = std::chrono::steady_clock::now();
          if (std::chrono::duration<double>(compute_end - previous_diagnostics_time).count() >=
              diagnostics_period_sec) {
            previous_diagnostics_time = compute_end;
            std_msgs::Float64MultiArray message;
            std_msgs::MultiArrayDimension dimension;
            dimension.label = "v1:";
            for (const auto& name : dynamics.jointNames()) dimension.label += name + ",";
            constexpr std::size_t kMetadataCount = 5;
            constexpr std::size_t kFieldCount = 14;
            dimension.size = kMetadataCount + kFieldCount * dynamics.jointNames().size();
            dimension.stride = dimension.size;
            message.layout.dim.push_back(dimension);
            message.data.reserve(dimension.size);
            message.data.push_back(static_cast<double>(sequence));
            message.data.push_back(loop_dt_ms);
            message.data.push_back(
                std::chrono::duration<double, std::milli>(compute_end - loop_start).count());
            message.data.push_back(limb.accelerationRefAgeSec());
            message.data.push_back(torque_control_active ? 1.0 : 0.0);
            const std::vector<const std::vector<double>*> fields{
                &result.position,             &result.velocity,
                &result.reference_position,   &result.reference_velocity,
                &result.mpc_acceleration,      &result.pd_acceleration,
                &result.commanded_acceleration, &result.inertia_torque,
                &result.coriolis_torque,       &result.gravity_torque,
                &result.damping_torque,        &result.raw_torque,
                &result.commanded_torque,      &result.torque_limit};
            for (const auto* field : fields) {
              message.data.insert(message.data.end(), field->begin(), field->end());
            }
            diagnostics_publisher.publish(message);
          }
        } catch (const std::exception& exception) {
          ROS_ERROR_THROTTLE(1.0, "KDL torque calculation failed: %s", exception.what());
          if (torque_control_active) {
            limb.exitControlMode(acceleration_timeout_sec);
            torque_control_active = false;
          }
        }
      }
      rate.sleep();
    }
    if (torque_control_active) limb.exitControlMode(acceleration_timeout_sec);
  } catch (const std::exception& exception) {
    ROS_FATAL("KDL inverse dynamics startup failed: %s", exception.what());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
