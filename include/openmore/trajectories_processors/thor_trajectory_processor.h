#pragma once

#include <openmore/trajectories_processors/spline_trajectory_processor.h>
#include <thor_math/thor_math.h>

/**
 * @file thor_trajectory_processor.h
 * @brief Contains the declaration of the ThorTrajectoryProcessor class.
 */

namespace openmore
{

struct QpIntervals
/**
 * @brief A Structure representing the receiding horizon parameters.
 */
{
public:
  int nax;  // number of DoF
  int nc;   // number of control intervals (equal to number of predictions)
  double control_horizon;
  double st;  // time step
};
typedef std::shared_ptr<QpIntervals> QpIntervalsPtr;

struct QpWeigth
/**
 * @brief A Structurwe representing the weigth of the cost function.
 */
{
public:
  double lambda_acc;      // weight for the acceleration
  double lambda_scaling;  // weight for the scaling
  double lambda_pos;      // weight for the position
  double lambda_jerk;     // weight for the jerk
};
typedef std::shared_ptr<QpWeigth> QpWeigthPtr;

class ThorTrajectoryProcessor;
typedef std::shared_ptr<ThorTrajectoryProcessor> ThorTrajectoryProcessorPtr;

/**
 * @brief The ThorTrajectoryProcessor class processes trajectories using spline interpolation and MPC
 */

class ThorTrajectoryProcessor : public virtual SplineTrajectoryProcessor
{
protected:
  thor::math::ThorQP thor;  // The Thor_math object for solving the QP problem.
  // double dt_ = 1e-3;
  openmore::QpWeigthPtr weigths_;
  openmore::QpIntervalsPtr intervals_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Default constructor.
   * Requires a call to init() aftwrwards.
   */
  ThorTrajectoryProcessor() : SplineTrajectoryProcessor()
  {
  }

  /**
   * @brief Constructors.
   */
  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger)
    : SplineTrajectoryProcessor(constraints, param_ns, logger)
  {
    ThorTrajectoryProcessor::setConstraints(constraints);
  }

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path)
    : SplineTrajectoryProcessor(constraints, param_ns, logger, path)
  {
    ThorTrajectoryProcessor::setConstraints(constraints);
  }

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger, const spline_order_t& spline_order)
    : SplineTrajectoryProcessor(constraints, param_ns, logger, spline_order)
  {
    ThorTrajectoryProcessor::setConstraints(constraints);
  }

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,
                          const spline_order_t& spline_order)
    : SplineTrajectoryProcessor(constraints, param_ns, logger, path, spline_order)
  {
    ThorTrajectoryProcessor::setConstraints(constraints);
  }

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                          const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,
                          const spline_order_t& spline_order, const QpWeigthPtr weigths, const QpIntervalsPtr intervals)
    : SplineTrajectoryProcessor(constraints, param_ns, logger, path, spline_order)
  {
    ThorTrajectoryProcessor::setWeigths(weigths);
    ThorTrajectoryProcessor::setIntervals(intervals);
    ThorTrajectoryProcessor::setConstraints(constraints);
    thor.activateTorqueBounds(false);
    if (thor.needUpdate())
    {
      thor.updateMatrices();
    }
  }

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is
   * called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @param spline_order The order of the spline interpolation.
   * @param weigths The weigths for the cost function.
   * @param intervals The intervals for the receiding horizon.
   * @return
   */
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                    const cnr_logger::TraceLoggerPtr& logger, const QpWeigthPtr weigths,
                    const QpIntervalsPtr intervals);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns,
                    const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,
                    const QpWeigthPtr weigths, const QpIntervalsPtr intervals);

  /**
   * @brief Function to compute the trajectory.
   * @param initial_state The initial robot state.
   * @param final_state The final robot state.
   * @return True if the trajectory computation is successful, false otherwise.
   */

  /**
   * @brief Interpolates a trajectory point at a given time.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param target_scaling Scaling factor for interpolation.
   * @param updated_scaling Updated scaling factor computed by Thor interpolation.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling,
                           double& updated_scaling) override;

  /**
   * @brief Sets the weights for the trajectory processor.
   *
   * This function assigns the provided weights to the trajectory processor,
   * which will be used in the optimization process.
   * @param weigths A shared pointer to the QpWeigth object containing the weights.
   */
  virtual void setWeigths(const QpWeigthPtr weigths);

  /**
   * @brief Sets the intervals for the trajectory processor.
   *
   * This function assigns the provided intervals to the trajectory processor.
   *
   * @param intervals A shared pointer to the QpIntervals object containing the intervals to be set.
   */
  virtual void setIntervals(const QpIntervalsPtr intervals);

  /**
   * @brief Sets the constraints for the trajectory processor.
   *
   * This method is used to define the constraints that will be applied
   * to the trajectory processing. Constraints can include limits on
   * velocity, acceleration, or other parameters that affect the
   * trajectory generation.
   */
  virtual void setConstraints();
  virtual void setConstraints(const KinodynamicConstraintsPtr& constraints);
  /**
   * @brief Sets the initial state of the robot.
   *
   * This function sets the initial state of the robot using the provided
   * RobotStatePtr. It is a virtual function and can be overridden by
   * derived classes to provide specific implementations.
   *
   * @param initial_state A shared pointer to the initial state of the robot.
   */
  virtual void setInitialState(const openmore::RobotStatePtr& initial_state);

  virtual void set_trajectory(const std::deque<openmore::TrjPointPtr>& trajectory);
  virtual bool computeTrj() override;
  virtual bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state) override;

  virtual TrajectoryProcessorBasePtr clone() override;
};

}  // namespace openmore
