#include <openmore/trajectories_processors/thor_trajectory_processor.h>
#include  <thor_math/thor_math.h>



namespace openmore
{

bool ThorTrajectoryProcessor::interpolate(const double& time, TrjPointPtr& pnt, const double& target_scaling, double& updated_scaling)
{
    //TODO da valutare: Posso mettere  questi dati nelle funzioni di inizializzazione/costruttori? nel caso servirebbe un dt sempre fisso. è così?
    //TODO In caso prevedere funzione che mette in init nAx, nc e control_horizon, prevedere funzione che di soluzione in soluzione setta il dt.
    std::ofstream file_spline;
    std::string package_path = "/home/galileo/Desktop/";

    int nc = intervals_->nc;
    int n_ax = intervals_->nax;

    // Inizialization of the reference vectors
    Eigen::VectorXd target_Dq(nc*n_ax);
    target_Dq.setOnes();
    
    Eigen::VectorXd next_Q(n_ax);
    next_Q.setZero(); 
    
    // Interpolate with the spline class for each prediction time instant
    Eigen::VectorXd istants =  thor.getPredictionTimeInstant();
    for (int i = 0; i < istants.size(); i++){
        if ( !openmore::SplineTrajectoryProcessor::interpolate(time+istants[i], pnt, target_scaling, updated_scaling) ){
            return false;
        }
        if (i==0){
            // Assign values to traget vectors
            next_Q.head(n_ax) = Eigen::VectorXd::Map(pnt->state_->pos_.data(), n_ax);

             //DA ELIMINARE: salvo i dati della spline in un csv
            file_spline.open(package_path + "spline_results.csv", std::ios_base::app);
            file_spline << time;
            for (int i = 0; i < pnt->state_->pos_.size(); ++i) {
                file_spline << "," << pnt->state_->pos_[i];
            }
            for (int i = 0; i < pnt->state_->vel_.size(); ++i) {
                file_spline << "," << pnt->state_->vel_[i];
            }
            for (int i = 0; i < pnt->state_->acc_.size(); ++i) {
                file_spline << "," << pnt->state_->acc_[i];
            }
            file_spline << "," << target_scaling << "\n";
            file_spline.close();

            //fine salvataggio

            
        }

        target_Dq.segment(i*n_ax, n_ax) = Eigen::VectorXd::Map(pnt->state_->vel_.data(), n_ax);        
    }
    
    Eigen::VectorXd next_acc(n_ax);
    Eigen::VectorXd next_vel(n_ax);
    Eigen::VectorXd next_pos(n_ax);

    // Receiding horizon method
    thor.computedCostrainedSolution(target_Dq,next_Q,target_scaling,thor.getState(),next_acc,updated_scaling);
    
    // Update of the solver state and of the solution point
    next_pos = thor.getState().head(n_ax);
    next_vel = thor.getState().tail(n_ax);

    thor.updateState(next_acc);
    pnt->state_->acc_.assign(next_acc.data(), next_acc.data() + next_acc.size());
    pnt->state_->vel_.assign(next_vel.data(), next_vel.data() + next_vel.size());
    pnt->state_->pos_.assign(next_pos.data(), next_pos.data() + next_pos.size());
    return true;
}

void ThorTrajectoryProcessor::setWeigths(const QpWeigthPtr weigths)
{
    double lambda_acc = weigths->lambda_acc;
    double lambda_clik = weigths->lambda_clik;
    double lambda_tau = weigths->lambda_tau;
    double lambda_scaling = weigths->lambda_scaling;
    double lambda_jerk = weigths->lambda_jerk;
    weigths_ = weigths;
    thor.setWeigthFunction(lambda_acc,lambda_tau,lambda_jerk,lambda_scaling,lambda_clik);


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
bool ThorTrajectoryProcessor::computeTrj(std::string path_in){

    if (!intervals_) {
        throw std::runtime_error("Intervals are not initialized.");
    }
    int nax = intervals_->nax;
    


    std::string config_path = path_in;
    YAML::Node yaml_path_ns;
    try {
        yaml_path_ns = YAML::LoadFile(config_path);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error loading file: " << config_path << std::endl;
        return false;
    }
    std::vector<double> time = yaml_path_ns["thor_trajectory_test"]["trajectory"]["times"].as<std::vector<double>>();
    std::vector<std::vector<double>> pos_vec;
    std::vector<std::vector<double>> vel_vec;
    std::vector<std::vector<double>> acc_vec;


    for (const auto& node : yaml_path_ns["thor_trajectory_test"]["trajectory"]["positions"]) {
        std::vector<double> vec = node.as<std::vector<double>>();
        pos_vec.push_back(vec);
        
    }
    
    for (const auto& node : yaml_path_ns["thor_trajectory_test"]["trajectory"]["velocities"]) {
        std::vector<double> vec = node.as<std::vector<double>>();
       vel_vec.push_back(vec);
    }
    
    for (const auto& node : yaml_path_ns["thor_trajectory_test"]["trajectory"]["accelerations"]) {
        std::vector<double> vec = node.as<std::vector<double>>();
       acc_vec.push_back(vec);
    }
    
    for (int i =0; i< vel_vec.size(); i++){
        openmore::TrjPointPtr point = std::make_shared<openmore::TrjPoint>();
        point->state_ = std::make_shared<openmore::RobotState>();
        point->state_->pos_.resize(nax);
        point->state_->vel_.resize(nax);
        point->state_->acc_.resize(nax);
        point->state_->eff_.resize(nax);

        point->time_from_start_ = time[i];
        point->state_->pos_ = pos_vec[i];
        point->state_->vel_ = vel_vec[i]; 
        point->state_->acc_ = acc_vec[i];

        trj_.push_back(point);
        max_t_=time[i];
    }
        
      
    return true;

    }

    void ThorTrajectoryProcessor::setInitialState(const openmore::RobotStatePtr& initial_state){
        int nax = intervals_->nax;
        Eigen::VectorXd state(2*nax);
        state.head(nax) = Eigen::VectorXd::Map(initial_state->pos_.data(), nax);
        state.tail(nax) = Eigen::VectorXd::Map(initial_state->vel_.data(), nax);
        thor.setInitialState(state);
    }

}