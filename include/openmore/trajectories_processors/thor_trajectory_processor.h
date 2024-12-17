#pragma once

#include <openmore/trajectories_processors/spline_trajectory_processor.h>
#include  <thor_math/thor_math.h>

/**
 * @file thor_trajectory_processor.h
 * @brief Contains the declaration of the ThorTrajectoryProcessor class.
 */

namespace openmore
{

struct QpIntervals
/**
 * @brief A Structurerepresenting the receiding horizon parameters.
 */
{
public:
  int nax; //number of DoF
  int nc;  //number of control intervals (equal to number of predictions)
  double control_horizon; 
  double st;
};
typedef std::shared_ptr<QpIntervals> QpIntervalsPtr;

struct QpWeigth
/** 
 * @brief A Structurwe representing the weigth of the cost function.
 */
{
public:
   
  double lambda_acc;
  double lambda_tau;
  double lambda_scaling;
  double lambda_clik;
  double lambda_jerk;
};
typedef std::shared_ptr<QpWeigth> QpWeigthPtr;


class ThorTrajectoryProcessor;
typedef std::shared_ptr<ThorTrajectoryProcessor> ThorTrajectoryProcessorPtr;

/**
 * @brief The ThorTrajectoryProcessor class processes trajectories using spline interpolation and MPC
*/

class ThorTrajectoryProcessor: public SplineTrajectoryProcessor
{

public:
  double max_t_;  


protected:
  /**
   * @brief The Thor_math object for solving the QP problem.
   */
  thor::math::ThorQP thor;
  double dt_ = 1e-3;
  openmore::QpWeigthPtr weigths_;
  openmore::QpIntervalsPtr intervals_;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW



  /**
   * @brief Constructors.
   * TODO chiamata costruttore di thor, assegnazione dei constraints a thor, costruttore che riceve in ingresso anche le funzioni dei pesi (definire una struct?)
   * TODO INIT FUNCTION devono chiamare la funzione thor.setconstraints, fare nuova init function che riceve in ingresso anche i pesi e chiama thor.setweightfunction
   * TODO funzione per settare il calcolo dei torque e dei position bounds, funzione per settare lo stato iniziale (può ricevere in ingresso un robotState, devo decomporlo)
   * TODO ricordarsi di chiamare la funzione UpdateMatrices dopo aver settato tutte le cose di thor
   * TODO da valutare: come gestiamo il dt (tempo tra due interpolazioni), se sempre fisso conviene definire all'inizio (quindi inserire nelle funzioni di init, a questo punto
   * assieme anche a nc, control horizon e nax, così da chiamare assieme le fuinzioni setIntervals e updateMatrices). Se variabile invece bisogna definire una funzione che setta 
   * il dt e chiamare updateMatrices nell'interpolzazione.
   *
   * 
   */
  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger):
    SplineTrajectoryProcessor(constraints,param_ns,logger){}

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path):
    SplineTrajectoryProcessor(constraints,param_ns,logger,path){}

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns,
                            const cnr_logger::TraceLoggerPtr& logger,
                            const spline_order_t& spline_order):
    SplineTrajectoryProcessor(constraints,param_ns,logger,spline_order){}

  ThorTrajectoryProcessor(const KinodynamicConstraintsPtr& constraints,
                            const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger,
                            const std::vector<Eigen::VectorXd>& path,
                            const spline_order_t& spline_order):
    SplineTrajectoryProcessor(constraints,param_ns,logger,path,spline_order){}

  /**
   * @brief init Initializes the TrajectoryProcessor object. This function should be called when the void constructor is called and it is used mainly for plugins.
   * @param constraints The kinodynamics constraints of the robot to be considered for trajectory generation.
   * @param param_ns_ The namespace under which to read the parameters with cnr_param.
   * @param logger The logger for logging purposes.
   * @param path The path for which the time-law need to be computed.
   * @return
   */
  // virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger) override;
  // virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path) override;

  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const QpWeigthPtr weigths, const QpIntervalsPtr intervals);
  virtual bool init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,  const QpWeigthPtr weigths, const QpIntervalsPtr intervals);



  /**
   * @brief Function to compute the trajectory.
   * @param initial_state The initial robot state.
   * @param final_state The final robot state.
   * @return True if the trajectory computation is successful, false otherwise.
   */
  //  virtual bool computeTrj() override;
  //  virtual bool computeTrj(const RobotState& initial_state) override;
  //  virtual bool computeTrj(const RobotState& initial_state, const RobotState& final_state) override;


  /**
   * @brief Interpolates a trajectory point at a given time.
   * @param time The time at which to interpolate.
   * @param pnt The interpolated trajectory point.
   * @param scaling Scaling factor for interpolation.
   * @return True if the interpolation is successful, false otherwise.
   */
  virtual bool interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling) override;
 

/**
 * @brief Sets the weights for the trajectory processor.
 * 
 * This function assigns the provided weights to the trajectory processor, 
 * which will be used in the optimization process.

 * @param weigths A shared pointer to the QpWeigth object containing the weights.
 */
 virtual void  setWeigths(const QpWeigthPtr weigths);

/**
 * @brief Sets the intervals for the trajectory processor.
 * 
 * This function assigns the provided intervals to the trajectory processor.
 * 
 * @param intervals A shared pointer to the QpIntervals object containing the intervals to be set.
 */
 virtual void setIntervals(const QpIntervalsPtr intervals);

 virtual void setConstraints();

virtual bool computeTrj(std::string path_in);


  
  bool computeTrj(const RobotStatePtr& initial_state){
      return false;
  }
  bool computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state){
      return false;
  }
  virtual void setInitialState(const openmore::RobotStatePtr& initial_state);
 };


}