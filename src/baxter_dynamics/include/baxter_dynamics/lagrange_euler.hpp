#pragma once

#include <memory>
#include <unordered_map>

#include "forward_kinematics.hpp"

namespace baxter {

inline constexpr int kConstantFour = 4;

class LagrangeEuler {
public:
  LagrangeEuler(const std::string &limb, ros::NodeHandle &nh,
                std::shared_ptr<ForwardKinematics> fksolver);
  LagrangeEuler(const std::string &limb, ros::NodeHandle &nh);

  std::unordered_map<std::string, double> Torque_calc(
      const std::unordered_map<std::string, double> &joint_angles,
      const std::unordered_map<std::string, double> &joint_velocities,
      const std::unordered_map<std::string, double> &joint_accelerations);

  std::unordered_map<std::string, double>
  EigenVectorToMap(const Eigen::VectorXd &values,
                   const std::vector<std::string> &joint_names);

private:
  static constexpr int number_of_links_ = 7;

  Eigen::VectorXd Ixx_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd Iyy_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd Izz_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd Ixy_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd Iyz_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd Izx_{Eigen::VectorXd(number_of_links_)};
  Eigen::VectorXd link_mass_{Eigen::VectorXd(number_of_links_)};
  Eigen::MatrixXd centre_of_mass_{
      Eigen::MatrixXd(number_of_links_, kConstantFour)};
  Eigen::Matrix4d Q_j_;
  std::string limb_;
  std::vector<std::string> joint_names_;

  double torque_limit_shoulder_elbow_max_{50.0};
  double torque_limit_shoulder_elbow_min_{-50.0};
  double torque_limit_wrist_max_{20.0};
  double torque_limit_wrist_min_{-20.0};

  std::shared_ptr<ForwardKinematics> fksolver_{nullptr};

  void setJointNames(ros::NodeHandle &nh);
  void setInertia(ros::NodeHandle &nh);
  void setTorqueLimits(ros::NodeHandle &nh);

  std::vector<Eigen::Matrix4d> transformation_matrix(
      const std::unordered_map<std::string, double> &joint_angles);
  std::vector<Eigen::Matrix4d> inertia_mat();
  std::vector<Eigen::Matrix4d>
  U_ij(const std::vector<Eigen::Matrix4d> &transformation_mat);
  std::vector<Eigen::Matrix4d>
  U_ijk(const std::vector<Eigen::Matrix4d> &transformation_mat);
  Eigen::MatrixXd M_mat(const std::vector<Eigen::Matrix4d> &J_i,
                        const std::vector<Eigen::Matrix4d> &Uij);
    Eigen::VectorXd
    C_vec(const std::vector<Eigen::Matrix4d> &J_i,
      const std::vector<Eigen::Matrix4d> &Uij,
      const std::vector<Eigen::Matrix4d> &Uijk,
      const std::unordered_map<std::string, double> &joint_velocities);
  Eigen::VectorXd G_mat(const std::vector<Eigen::Matrix4d> &Uij);
};

} // namespace baxter