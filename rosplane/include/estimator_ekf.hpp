#ifndef ESTIMATOR_EKF_H
#define ESTIMATOR_EKF_H

#include "estimator_ros.hpp"
#include <Eigen/Geometry>
#include <math.h>
#include <tuple>
#include <cassert>
#include <yaml-cpp/yaml.h>

namespace rosplane
{

class estimator_ekf : public estimator_ros
{
public:
  estimator_ekf();

protected:
  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> measurement_update(Eigen::VectorXf x,
                                                                  Eigen::VectorXf inputs,
                                                                  std::function<Eigen::VectorXf(const Eigen::VectorXf, const Eigen::VectorXf)> measurement_model,
                                                                  Eigen::VectorXf y,
                                                                  std::function<Eigen::MatrixXf(const Eigen::VectorXf, const Eigen::VectorXf)> measurement_jacobian,
                                                                  Eigen::MatrixXf R,
                                                                  Eigen::MatrixXf P);

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> propagate_model(Eigen::VectorXf x,
                                                               std::function<Eigen::VectorXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> dynamic_model,
                                                               std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> jacobian,
                                                               Eigen::VectorXf inputs,
                                                               std::function<Eigen::MatrixXf(const Eigen::VectorXf&, const Eigen::VectorXf&)> input_jacobian,
                                                               Eigen::MatrixXf P,
                                                               Eigen::MatrixXf Q,
                                                               Eigen::MatrixXf Q_g,
                                                               float Ts);          

  std::tuple<Eigen::MatrixXf, Eigen::VectorXf> single_measurement_update(float measurement,
                                                                         float mesurement_prediction,
                                                                         float measurement_uncertainty,
                                                                         Eigen::VectorXf measurement_jacobian,
                                                                         Eigen::VectorXf x,
                                                                         Eigen::MatrixXf P);

private:
  virtual void estimate(const input_s & input, output_s & output) override = 0;
}; 

} // namespace rosplane

#endif // ESTIMATOR_EKF_H
