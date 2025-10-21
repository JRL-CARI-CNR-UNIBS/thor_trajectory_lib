#include <openmore/trajectories_processors/thor_trajectory_processor.h>
#include  <thor_math/thor_math.h>



namespace openmore
{
    // bool ThorTrajectoryProcessor::interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling)
    // {
    //     CNR_FATAL(logger_, "Target_scaling: "<<target_scaling);
    //     return SplineTrajectoryProcessor::interpolate(time, pnt, target_scaling, updated_scaling);
    // }
bool ThorTrajectoryProcessor::interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling)
{

    int nc = intervals_->nc;
    int n_ax = intervals_->nax;

    thor.activateTorqueBounds(false);
    if (thor.needUpdate())
    {
    thor.updateMatrices();
    }

    // Inizialization of the reference vectors
    Eigen::VectorXd target_Dq(nc*n_ax);
    target_Dq.setOnes();
    
    Eigen::VectorXd next_Q(nc*n_ax);
    next_Q.setZero(); 
    
    // Interpolate with the spline class for each prediction time instant
    Eigen::VectorXd istants =  thor.getPredictionTimeInstant();
    for (int i = 0; i < istants.size(); i++){
        if ( !openmore::SplineTrajectoryProcessor::interpolate(time+istants[i], pnt, 1, updated_scaling) ){
            return false;
        }
        next_Q.segment(i*n_ax, n_ax) = Eigen::VectorXd::Map(pnt->state_->pos_.data(), n_ax);
        target_Dq.segment(i*n_ax, n_ax) = Eigen::VectorXd::Map(pnt->state_->vel_.data(), n_ax);        
    }
    Eigen::VectorXd next_acc(n_ax);
    Eigen::VectorXd new_acc(n_ax);

    Eigen::VectorXd next_vel(n_ax);
    Eigen::VectorXd next_pos(n_ax);

    // Receiding horizon method
    thor.computedCostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,updated_scaling);
    
    // Update of the solver state and of the solution point
   
    thor.updateState(next_acc);
    next_pos = thor.getState().head(n_ax);
    next_vel = thor.getState().tail(n_ax);
    new_acc = next_acc;
    pnt->state_->acc_.assign(new_acc.data(), new_acc.data() + new_acc.size());
    pnt->state_->vel_.assign(next_vel.data(), next_vel.data() + next_vel.size());
    pnt->state_->pos_.assign(next_pos.data(), next_pos.data() + next_pos.size());
    
    return true;
}

void ThorTrajectoryProcessor::setWeigths(const QpWeigthPtr weigths)
{
    double lambda_acc = weigths->lambda_acc;
    double lambda_pos = weigths->lambda_pos;
    double lambda_tau = 0.0; //weigths->lambda_tau;
    double lambda_scaling = weigths->lambda_scaling;
    double lambda_jerk = weigths->lambda_jerk;
    weigths_ = weigths;
    thor.setWeigthFunction(lambda_acc,lambda_tau,lambda_jerk,lambda_scaling,lambda_pos);


}

void ThorTrajectoryProcessor::setIntervals(const QpIntervalsPtr intervals)
{
    int nax= intervals->nax;
    int nc=intervals->nc;
    double control_horizon=intervals->control_horizon;
    double st=intervals->st;
    intervals_ = intervals;
    thor.setIntervals(nc,nax,control_horizon,st);    

}

void ThorTrajectoryProcessor::setConstraints(){
    Eigen::VectorXd qmax = kinodynamic_constraints_->max_pos_;
    Eigen::VectorXd Dqmax = kinodynamic_constraints_->max_vel_;
    Eigen::VectorXd DDqmax = kinodynamic_constraints_->max_acc_;
    Eigen::VectorXd tau_max = kinodynamic_constraints_->max_eff_;

    Eigen::VectorXd qmin = kinodynamic_constraints_->min_pos_;
    Eigen::VectorXd Dqmin = kinodynamic_constraints_->min_vel_;
    Eigen::VectorXd DDqmin = kinodynamic_constraints_->min_acc_;
    Eigen::VectorXd tau_min = kinodynamic_constraints_->min_eff_;

    thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);

}

void ThorTrajectoryProcessor::setConstraints(const KinodynamicConstraintsPtr& constraints){
    Eigen::VectorXd qmax = constraints->max_pos_;
    Eigen::VectorXd Dqmax = constraints->max_vel_;
    Eigen::VectorXd DDqmax = constraints->max_acc_;
    Eigen::VectorXd tau_max = constraints->max_eff_;

    Eigen::VectorXd qmin = constraints->min_pos_;
    Eigen::VectorXd Dqmin = constraints->min_vel_;
    Eigen::VectorXd DDqmin = constraints->min_acc_;
    Eigen::VectorXd tau_min = constraints->min_eff_;

    thor.setConstraints(qmax,qmin,Dqmax,DDqmax,tau_max);

}
bool ThorTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const QpWeigthPtr weigths, const QpIntervalsPtr intervals){

    openmore::SplineTrajectoryProcessor::init(constraints, param_ns, logger);
    ThorTrajectoryProcessor::setWeigths(weigths);
    ThorTrajectoryProcessor::setIntervals(intervals);
    ThorTrajectoryProcessor::setConstraints();
    thor.activateTorqueBounds(false);
    if (thor.needUpdate())
    { 
    thor.updateMatrices();
    }
    return true;

}

bool ThorTrajectoryProcessor::init(const KinodynamicConstraintsPtr& constraints, const std::string& param_ns, const cnr_logger::TraceLoggerPtr& logger, const std::vector<Eigen::VectorXd>& path,  const QpWeigthPtr weigths, const QpIntervalsPtr intervals)
{
    openmore::SplineTrajectoryProcessor::init(constraints, param_ns, logger, path);
    ThorTrajectoryProcessor::setWeigths(weigths);
    ThorTrajectoryProcessor::setIntervals(intervals);
    ThorTrajectoryProcessor::setConstraints();
    return true;
}

void ThorTrajectoryProcessor::setInitialState(const openmore::RobotStatePtr& initial_state){
    int nax = intervals_->nax;
    Eigen::VectorXd state(2*nax);
    state.head(nax) = Eigen::VectorXd::Map(initial_state->pos_.data(), nax);
    state.tail(nax) = Eigen::VectorXd::Map(initial_state->vel_.data(), nax);
    thor.setInitialState(state);
}
void ThorTrajectoryProcessor::set_trajectory(const std::deque<openmore::TrjPointPtr>& trajectory)
{
    trj_ = trajectory;
}

bool ThorTrajectoryProcessor::computeTrj()
{
    return true;
}
bool ThorTrajectoryProcessor::computeTrj(const RobotStatePtr& initial_state, const RobotStatePtr& final_state)
{
    return true;
}

openmore::TrajectoryProcessorBasePtr ThorTrajectoryProcessor::clone() 
{
    auto cloned = std::make_shared<ThorTrajectoryProcessor>();
    // Clone base class members
    cloned->openmore::SplineTrajectoryProcessor::operator=(*this);

    // Deep copy or clone pointers if necessary
    if (weigths_)
        cloned->weigths_ = std::make_shared<QpWeigth>(*weigths_);
    if (intervals_)
        cloned->intervals_ = std::make_shared<QpIntervals>(*intervals_);
    if (kinodynamic_constraints_)
        cloned->kinodynamic_constraints_ = std::make_shared<KinodynamicConstraints>(*kinodynamic_constraints_);
    cloned->trj_ = trj_; // std::deque of shared_ptr, shallow copy is fine

    // Copy or clone thor object if needed (assuming copyable)
    cloned->thor = this->thor;
    return cloned;
}
}
