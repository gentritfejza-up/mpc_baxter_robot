#include "baxter_dynamics/lagrange_euler.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace baxter {

LagrangeEuler::LagrangeEuler(const std::string &limb, ros::NodeHandle &nh,
                             std::shared_ptr<ForwardKinematics> fksolver)
    : fksolver_(fksolver), limb_(limb) {
  setJointNames(nh);
  setInertia(nh);
  setTorqueLimits(nh);
}

LagrangeEuler::LagrangeEuler(const std::string &limb, ros::NodeHandle &nh)
    : LagrangeEuler(limb, nh,
                    std::make_shared<ForwardKinematics>(limb, nh)) {}

std::unordered_map<std::string, double>
LagrangeEuler::EigenVectorToMap(const Eigen::VectorXd &values,
                                const std::vector<std::string> &joint_names) {
  if (values.size() != static_cast<int>(joint_names.size())) {
    throw std::invalid_argument(
        "Size mismatch between values and joint names.");
  }

  std::unordered_map<std::string, double> result;
  result.reserve(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); ++i) {
    const std::string &joint_name = joint_names[i];
    double torque_value = values(i);

    if (joint_name == limb_ + "_s0" || joint_name == limb_ + "_s1" ||
        joint_name == limb_ + "_e0" || joint_name == limb_ + "_e1") {
      torque_value = std::max(torque_limit_shoulder_elbow_min_,
                              std::min(torque_limit_shoulder_elbow_max_,
                                       torque_value));
    } else if (joint_name == limb_ + "_w0" || joint_name == limb_ + "_w1" ||
               joint_name == limb_ + "_w2") {
      torque_value = std::max(torque_limit_wrist_min_,
                              std::min(torque_limit_wrist_max_, torque_value));
    }

    result.emplace(joint_name, torque_value);
  }

  return result;
}

void LagrangeEuler::setJointNames(ros::NodeHandle &nh) {
  nh.getParam(std::string("robot/limb/" + limb_), joint_names_);
}

void LagrangeEuler::setTorqueLimits(ros::NodeHandle &nh) {
  nh.param("robot/torque_limits/shoulder_elbow_max",
           torque_limit_shoulder_elbow_max_, 50.0);
  nh.param("robot/torque_limits/shoulder_elbow_min",
           torque_limit_shoulder_elbow_min_, -50.0);
  nh.param("robot/torque_limits/wrist_max", torque_limit_wrist_max_, 20.0);
  nh.param("robot/torque_limits/wrist_min", torque_limit_wrist_min_, -20.0);
}

void LagrangeEuler::setInertia(ros::NodeHandle &nh) {
  std::vector<double> Ixx_vec, Iyy_vec, Izz_vec, Ixy_vec, Iyz_vec, Izx_vec,
      link_mass;

  if (nh.getParam("robot/limb/inertia/Ixx", Ixx_vec) &&
      nh.getParam("robot/limb/inertia/Iyy", Iyy_vec) &&
      nh.getParam("robot/limb/inertia/Izz", Izz_vec) &&
      nh.getParam("robot/limb/inertia/Ixy", Ixy_vec) &&
      nh.getParam("robot/limb/inertia/Iyz", Iyz_vec) &&
      nh.getParam("robot/limb/inertia/Izx", Izx_vec) &&
      nh.getParam("robot/limb/link_masses", link_mass) &&
      static_cast<int>(Ixx_vec.size()) == number_of_links_ &&
      static_cast<int>(Iyy_vec.size()) == number_of_links_ &&
      static_cast<int>(Izz_vec.size()) == number_of_links_ &&
      static_cast<int>(Ixy_vec.size()) == number_of_links_ &&
      static_cast<int>(Iyz_vec.size()) == number_of_links_ &&
      static_cast<int>(Izx_vec.size()) == number_of_links_ &&
      static_cast<int>(link_mass.size()) == number_of_links_) {
    Ixx_ = Eigen::VectorXd::Map(Ixx_vec.data(), Ixx_vec.size());
    Iyy_ = Eigen::VectorXd::Map(Iyy_vec.data(), Iyy_vec.size());
    Izz_ = Eigen::VectorXd::Map(Izz_vec.data(), Izz_vec.size());
    Ixy_ = Eigen::VectorXd::Map(Ixy_vec.data(), Ixy_vec.size());
    Iyz_ = Eigen::VectorXd::Map(Iyz_vec.data(), Iyz_vec.size());
    Izx_ = Eigen::VectorXd::Map(Izx_vec.data(), Izx_vec.size());
    link_mass_ = Eigen::VectorXd::Map(link_mass.data(), link_mass.size());
  } else {
    ROS_ERROR("Failed to load inertia parameters.");
    throw std::runtime_error("Invalid inertia or link mass parameters.");
  }

  std::vector<double> com_x, com_y, com_z, com_w;

  if (nh.getParam("robot/limb/com_x", com_x) &&
      nh.getParam("robot/limb/com_y", com_y) &&
      nh.getParam("robot/limb/com_z", com_z) &&
      nh.getParam("robot/limb/com_w", com_w) &&
      static_cast<int>(com_x.size()) == number_of_links_ &&
      static_cast<int>(com_y.size()) == number_of_links_ &&
      static_cast<int>(com_z.size()) == number_of_links_ &&
      static_cast<int>(com_w.size()) == number_of_links_) {
    for (int i = 0; i < number_of_links_; i++) {
      centre_of_mass_.row(i) << com_x[i], com_y[i], com_z[i], com_w[i];
    }
  } else {
    ROS_ERROR("Failed to load the center of mass data.");
    throw std::runtime_error("Invalid center of mass parameters.");
  }

  std::vector<double> qj_data;
  if (nh.getParam("robot/limb/Q_j", qj_data) && qj_data.size() == 16) {
    Q_j_ =
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(qj_data.data());
  } else {
    ROS_ERROR("Failed to load Q_j matrix.");
    throw std::runtime_error("Invalid Q_j matrix parameter.");
  }
}

std::vector<Eigen::Matrix4d> LagrangeEuler::transformation_matrix(
    const std::unordered_map<std::string, double> &joint_angles) {
  return fksolver_->solveIntermediateFK(joint_angles);
}

std::vector<Eigen::Matrix4d> LagrangeEuler::inertia_mat() {
  std::vector<Eigen::Matrix4d> J_i_mat;
  for (int i = 0; i < number_of_links_; i++) {
    Eigen::Matrix4d J_i;
    J_i << (-Ixx_(i) + Iyy_(i) + Izz_(i)) / 2, Ixy_(i), Izx_(i),
        link_mass_(i) * centre_of_mass_(i, 0), Ixy_(i),
        (Ixx_(i) - Iyy_(i) + Izz_(i)) / 2, Iyz_(i),
        link_mass_(i) * centre_of_mass_(i, 1), Izx_(i), Iyz_(i),
        (Ixx_(i) + Iyy_(i) - Izz_(i)) / 2,
        link_mass_(i) * centre_of_mass_(i, 2),
        link_mass_(i) * centre_of_mass_(i, 0),
        link_mass_(i) * centre_of_mass_(i, 1),
        link_mass_(i) * centre_of_mass_(i, 2), link_mass_(i);

    J_i_mat.push_back(J_i);
  }
  return J_i_mat;
}

std::vector<Eigen::Matrix4d>
LagrangeEuler::U_ij(const std::vector<Eigen::Matrix4d> &transformation_mat) {
  std::vector<Eigen::Matrix4d> Uij_vector;
  Eigen::Matrix4d Uij;
  for (int i = 1; i <= number_of_links_; i++) {
    for (int j = 1; j <= number_of_links_; j++) {
      if (j <= i) {
        Uij = transformation_mat[j - 1] * Q_j_ *
              transformation_mat[j - 1].inverse() * transformation_mat[i];
      } else {
        Uij.setZero();
      }
      Uij_vector.push_back(Uij);
    }
  }
  return Uij_vector;
}

std::vector<Eigen::Matrix4d>
LagrangeEuler::U_ijk(const std::vector<Eigen::Matrix4d> &transformation_mat) {
  std::vector<Eigen::Matrix4d> Uijk_vector;
  Eigen::Matrix4d Uijk;
  for (int i = 1; i <= number_of_links_; i++) {
    for (int j = 1; j <= number_of_links_; j++) {
      for (int k = 1; k <= number_of_links_; k++) {
        if ((i >= k) && (k >= j)) {
          Uijk = transformation_mat[j - 1] * Q_j_ *
                 transformation_mat[j - 1].inverse() *
                 transformation_mat[k - 1] * Q_j_ *
                 transformation_mat[k - 1].inverse() * transformation_mat[i];
        } else if ((i >= j) && (j >= k)) {
          Uijk = transformation_mat[k - 1] * Q_j_ *
                 transformation_mat[k - 1].inverse() *
                 transformation_mat[j - 1] * Q_j_ *
                 transformation_mat[j - 1].inverse() * transformation_mat[i];
        } else {
          Uijk.setZero();
        }

        Uijk_vector.push_back(Uijk);
      }
    }
  }
  return Uijk_vector;
}

Eigen::MatrixXd
LagrangeEuler::M_mat(const std::vector<Eigen::Matrix4d> &J_i,
                     const std::vector<Eigen::Matrix4d> &Uij) {
  Eigen::MatrixXd M_matrix{
      Eigen::MatrixXd(number_of_links_, number_of_links_)};
  double temp = 0;

  for (int i = 0; i < number_of_links_; i++) {
    for (int k = 0; k < number_of_links_; k++) {
      temp = 0;
      for (int j = std::max(i, k); j < number_of_links_; j++) {
        temp += (Uij[j * number_of_links_ + k] * J_i[j] *
                 (Uij[j * number_of_links_ + i].transpose()))
                    .trace();
      }
      M_matrix(i, k) = temp;
    }
  }
  return M_matrix;
}

Eigen::VectorXd LagrangeEuler::C_vec(
    const std::vector<Eigen::Matrix4d> &J_i,
    const std::vector<Eigen::Matrix4d> &Uij,
    const std::vector<Eigen::Matrix4d> &Uijk,
    const std::unordered_map<std::string, double> &joint_velocities) {
  std::vector<double> velocities;
  for (const auto &joint : joint_names_) {
    velocities.push_back(joint_velocities.at(joint));
  }

  Eigen::VectorXd H_i{Eigen::VectorXd(number_of_links_)};
  H_i.setZero();
  double h_ikm = 0;
  for (int i = 0; i < number_of_links_; i++) {
    for (int k = 0; k < number_of_links_; k++) {
      for (int m = 0; m < number_of_links_; m++) {
        h_ikm = 0;
        for (int j = std::max({i, k, m}); j < number_of_links_; j++) {
          h_ikm +=
              (Uijk[j * (number_of_links_ * number_of_links_) +
                    k * number_of_links_ + m] *
               J_i[j] * Uij[j * number_of_links_ + i].transpose())
                  .trace();
        }
        h_ikm = h_ikm * velocities[k] * velocities[m];
        H_i(i) += h_ikm;
      }
    }
  }
  return H_i;
}

Eigen::VectorXd
LagrangeEuler::G_mat(const std::vector<Eigen::Matrix4d> &Uij) {
  Eigen::VectorXd g{Eigen::VectorXd(kConstantFour)};
  Eigen::VectorXd centre{Eigen::VectorXd(kConstantFour)};
  Eigen::VectorXd G_matrix{Eigen::VectorXd(number_of_links_)};
  g << 0, 0, -9.81, 0;
  double temp = 0;

  for (int i = 0; i < number_of_links_; ++i) {
    temp = 0;
    for (int j = i; j < number_of_links_; j++) {
      centre = centre_of_mass_.row(j);
      temp += link_mass_[j] * g.transpose() *
              Uij[j * number_of_links_ + i] * centre;
    }
    G_matrix(i) = -temp;
  }
  return G_matrix;
}

std::unordered_map<std::string, double> LagrangeEuler::Torque_calc(
    const std::unordered_map<std::string, double> &joint_angles,
    const std::unordered_map<std::string, double> &joint_velocities,
    const std::unordered_map<std::string, double> &joint_accelerations) {
  Eigen::VectorXd accelerations{Eigen::VectorXd(joint_names_.size())};
  std::vector<Eigen::Matrix4d> transformation_matrices;
  std::vector<Eigen::Matrix4d> inertia_matrices;
  std::vector<Eigen::Matrix4d> uij_matrices;
  std::vector<Eigen::Matrix4d> uijk_matrices;
  Eigen::MatrixXd m_mat;
  Eigen::VectorXd c_vec, g_vec;

  for (size_t i = 0; i < joint_names_.size(); i++) {
    const auto joint_name = joint_names_[i];
    if (joint_accelerations.find(joint_name) != joint_accelerations.end()) {
      accelerations(i) = joint_accelerations.at(joint_name);
    } else {
      throw std::runtime_error("Joint name " + joint_name +
                               " not found in joint_accelerations map.");
    }
  }

    transformation_matrices = this->transformation_matrix(joint_angles);
    inertia_matrices = this->inertia_mat();
    uij_matrices = this->U_ij(transformation_matrices);
    uijk_matrices = this->U_ijk(transformation_matrices);

    m_mat = this->M_mat(inertia_matrices, uij_matrices);
    c_vec = this->C_vec(inertia_matrices, uij_matrices, uijk_matrices,
              joint_velocities);
    g_vec = this->G_mat(uij_matrices);

    Eigen::VectorXd tau = m_mat * accelerations + c_vec + g_vec;
  return EigenVectorToMap(tau, joint_names_);
}

} // namespace baxter