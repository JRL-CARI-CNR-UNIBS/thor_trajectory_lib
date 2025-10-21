#include <openmore/trajectories_processors/thor_trajectory_processor.h>
#include <openmore/trajectories_processors/spline_trajectory_processor.h>
#include <fstream>
#include <iostream>

// Function to create an example trajectory
std::deque<openmore::TrjPointPtr> create_trajectory()
{
  std::deque<openmore::TrjPointPtr> trajectory_deque;

  // --- Point 1 (Start) ---
  auto point1 = std::make_shared<openmore::TrjPoint>();
  point1->time_from_start_ = 0.0;
  point1->state_->pos_ = { 0.0, 0.0, 0.0 };  // Example position (3 joints)
  point1->state_->vel_ = { 0.0, 0.0, 0.0 };
  point1->state_->acc_ = { 0.0, 0.0, 0.0 };

  trajectory_deque.push_back(point1);  // Add to the back of the deque

  // --- Point 2 ---
  auto point2 = std::make_shared<openmore::TrjPoint>();
  point2->time_from_start_ = 3.0;
  point2->state_->pos_ = { 0.4, 0.5, 0.6 };
  point2->state_->vel_ = { 0.2, 0.2, 0.2 };
  point2->state_->acc_ = { 0.1, 0.1, 0.1 };

  trajectory_deque.push_back(point2);

  // --- Point 3 ---
  auto point3 = std::make_shared<openmore::TrjPoint>();
  point3->time_from_start_ = 6.0;
  point3->state_->pos_ = { 0.7, 0.8, 0.9 };
  point3->state_->vel_ = { 0.0, 0.0, 0.0 };
  point3->state_->acc_ = { -0.1, -0.1, -0.1 };

  trajectory_deque.push_back(point3);

  // --- Point 4 (End) ---
  // Let's create this one using the clone() method from point 3
  auto point4 = point3->clone();
  // Now modify the cloned point
  point4->time_from_start_ = 9.0;
  point4->state_->pos_ = { 1.0, 1.0, 1.0 };  // New position
  point4->state_->vel_ = { 0.0, 0.0, 0.0 };
  point4->state_->acc_ = { 0.0, 0.0, 0.0 };
  // Velocity and Acc are already copied from point3 (0.0s)

  trajectory_deque.push_back(point4);
  return trajectory_deque;
}

int main()
{
  // Receiding horizon parameters modify with the required ones
  int nax = 3;
  int nc = 5;
  double control_horizon = 1.0;
  double st = 0.01;

  openmore::QpIntervalsPtr intervals = std::make_shared<openmore::QpIntervals>(nax, nc, control_horizon, st);

  // Weigths for the cost function modify with the required ones
  double lambda_acc = 1.0e-06;    // weight for the acceleration
  double lambda_scaling = 1e+02;  // weight for the scaling
  double lambda_pos = 1e+03;      // weight for the position error
  double lambda_jerk = 1.0e-09;   // weight for the jerk
  double lambda_tau = 0.0;        // weight for the torque

  openmore::QpWeigthPtr weigths =
      std::make_shared<openmore::QpWeigth>(lambda_acc, lambda_tau, lambda_scaling, lambda_pos, lambda_jerk);

  // Constraints modify with the required ones
  Eigen::VectorXd qmax(nax);
  Eigen::VectorXd qmin(nax);
  Eigen::VectorXd Dqmax(nax);
  Eigen::VectorXd DDqmax(nax);
  Eigen::VectorXd tau_max(nax);

  qmax.setConstant(3.14);
  qmin.setConstant(1.57);
  Dqmax.setConstant(0.78);
  DDqmax.setConstant(0.39);
  tau_max.setConstant(0.2);

  openmore::KinodynamicConstraintsPtr constraints =
      std::make_shared<openmore::KinodynamicConstraints>(qmax, Dqmax, DDqmax, tau_max, qmin, -Dqmax, -DDqmax, tau_max);

  // Spline order for the trajectory processor
  openmore::SplineTrajectoryProcessor::spline_order_t order =
      static_cast<openmore::SplineTrajectoryProcessor::spline_order_t>(3);
  std::string param_ns = "your_param_namespace";  // Replace with your parameter namespace
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>();

  openmore::ThorTrajectoryProcessorPtr processor =
      std::make_shared<openmore::ThorTrajectoryProcessor>(constraints, param_ns, logger, order);

  processor->init(constraints, param_ns, logger, weigths, intervals);

  // Create an example trajectory
  std::deque<openmore::TrjPointPtr> your_trajectory = create_trajectory();

  processor->set_trajectory(your_trajectory);  // Replace 'your_trajectory' with the actual trajectory data
  // Set initial state
  openmore::RobotStatePtr ptr = processor->getTrj()[0]->state_;
  processor->setInitialState(ptr);

  double scl = 1;
  double target_scaling = 1;
  double t = 0;
  double t_nom = 0;
  double dt = st;

  double max_t = processor->getTrjDuration();
  // std::cout << spline_processor->trj_.back()->state_->vel_[0] << std::endl;
  // std::cout <<processor->trj_.back()->state_->vel_[0] << std::endl;
  while (t <= max_t)
  {
    t_nom += dt * scl;
    t += dt;
    openmore::TrjPointPtr point = std::make_shared<openmore::TrjPoint>();

    processor->interpolate(t_nom, point, target_scaling, scl);
    // Now point contains the interpolated state at time t_nom, scaled by scl.
    // You can use point->state_ to access the state information.
  }
  std::cout << "Test completed successfully." << std::endl;
  return 0;
}
