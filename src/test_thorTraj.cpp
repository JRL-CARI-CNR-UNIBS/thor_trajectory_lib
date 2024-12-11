#include <openmore/trajectories_processors/thor_trajectory_processor.h>
#include <openmore/trajectories_processors/spline_trajectory_processor.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

void YAMLtoCSV(std::string path_in, std::string path_out){
    YAML::Node yaml_file;
    try {
        yaml_file = YAML::LoadFile(path_in);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error loading file: " << path_in << std::endl;
        return;
    }

    std::ofstream csv_file(path_out,  std::ios::trunc);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening file: " << path_out << std::endl;
        return;
    }

    // Write CSV header
    csv_file << "time,position1,position2,position3,velocity1,velocity2,velocity3\n";

    // Write CSV data
  
    int max_size =yaml_file["positions"].size() ;
    for (int i = 0; i < max_size; ++i) {

        csv_file << yaml_file["time"][i].as<double>() << ",";
        for (int j = 0; j < 3; ++j) {    
            csv_file << yaml_file["positions"][i][j].as<double>() << ",";
            }
            
        
        for (int j = 0; j < 3; ++j) {

                csv_file << yaml_file["velocities"][i][j].as<double>() << ",";
            }
        
        
        csv_file.seekp(-1, std::ios_base::end); // Remove the last comma
        csv_file << "\n";
    }

    csv_file.close();
    }

int main(){

    std::string path_in = "/home/galileo/projects/trajectory_ws/src/thor_trajectory_test/config/computed_path.yaml";
    std::string path_out = "/home/galileo/Desktop/robot_trajectory.csv";
    YAMLtoCSV(path_in, path_out);
    
   
    //load weigths
    std::string config_path = "/home/galileo/projects/trajectory_ws/src/thor_trajectory_lib/config/parameters.yaml";
    YAML::Node yaml_path_ns;
    try {
        yaml_path_ns = YAML::LoadFile(config_path);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Error loading file: " << config_path << std::endl;
        return false;
    }
    
    //Receiding horizon parameters
    int nax = yaml_path_ns["nax"].as<int>(); 
    int nc = yaml_path_ns["nc"].as<int>();
    double control_horizon = yaml_path_ns["control_horizon"].as<double>();
    double st = yaml_path_ns["st"].as<double>();

    std::cout << "nax: " << nax << std::endl;
    std::cout << "nc: " << nc << std::endl;
    std::cout << "control_horizon: " << control_horizon << std::endl;
    std::cout << "st: " << st << std::endl;
    
    openmore::QpIntervalsPtr intervals = std::make_shared<openmore::QpIntervals>(nax, nc, control_horizon, st);

    //Weigths
    std::vector<double> weigth_vec= yaml_path_ns["weigths"].as<std::vector<double>>();        
    double lambda_acc = weigth_vec[0];
    double lambda_tau = weigth_vec[1];
    double lambda_scaling = weigth_vec[2];
    double lambda_clik = weigth_vec[3];
    double lambda_jerk = weigth_vec[4];

    openmore::QpWeigthPtr weigths = std::make_shared<openmore::QpWeigth>(lambda_acc, lambda_tau, lambda_scaling, lambda_clik, lambda_jerk);

    //Constraints
    Eigen::VectorXd qmax(nax);
    Eigen::VectorXd qmin(nax);
    Eigen::VectorXd Dqmax(nax);
    Eigen::VectorXd DDqmax(nax);
    Eigen::VectorXd tau_max(nax);
    
    qmax.setConstant(yaml_path_ns["qmax"].as<double>());
    qmin.setConstant(yaml_path_ns["qmin"].as<double>());
    Dqmax.setConstant(yaml_path_ns["Dqmax"].as<double>());
    DDqmax.setConstant(yaml_path_ns["DDqmax"].as<double>());
    tau_max.setConstant(yaml_path_ns["tau_max"].as<double>());

    std::cout << "qmax: " << qmax.transpose() << std::endl;
    std::cout << "qmin: " << qmin.transpose() << std::endl;
    std::cout << "Dqmax: " << Dqmax.transpose() << std::endl;
    std::cout << "DDqmax: " << DDqmax.transpose() << std::endl;
    std::cout << "tau_max: " << tau_max.transpose() << std::endl;

    openmore::KinodynamicConstraintsPtr constraints = std::make_shared<openmore::KinodynamicConstraints>(qmax, Dqmax, DDqmax, tau_max, qmin, -Dqmax, -DDqmax, tau_max);

    std::string package_path = "/home/galileo/Desktop/";
    std::string logger_file = package_path+"logger.yaml";
    cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_interpolator",logger_file);
    std::string param_ns = "robotic namespace";

    openmore::SplineTrajectoryProcessor::spline_order_t order = static_cast<openmore::SplineTrajectoryProcessor::spline_order_t>(yaml_path_ns["spline_order"].as<int>());
    openmore::ThorTrajectoryProcessorPtr processor = std::make_shared<openmore::ThorTrajectoryProcessor>(constraints, param_ns, logger, order);
    openmore::SplineTrajectoryProcessorPtr spline_processor = std::make_shared<openmore::SplineTrajectoryProcessor>(constraints, param_ns, logger,order);

    processor->init(constraints, param_ns, logger, weigths, intervals);    
    bool res = processor->computeTrj(path_in);
    spline_processor->trj_=processor->trj_;
    //Set initial state
    openmore::RobotStatePtr ptr = processor->getTrj()[0]->state_;
    processor-> setInitialState(ptr);

    double scaling=1;
    double target_scaling=1;
    double t=0;
    double t_nom=0;
    double dt=st;

    std::ofstream file;
    file.open(package_path + "trajectory_results.csv", std::ios_base::trunc);
    file << "time,pos1,pos2,pos3,vel1,vel2,vel3,acc1,acc2,acc3,scaling\n";
    file.close();
    std::ofstream file_spline;
    file_spline.open(package_path + "spline_results.csv", std::ios_base::trunc);
    file_spline << "time_spline,pos1_spline,pos2_spline,pos3_spline,vel1_spline,vel2_spline,vel3_spline,acc1_spline,acc2_spline,acc3_spline,scaling\n";
    file_spline.close();

    while (t<=processor->max_t_+dt)
    {
        t_nom += dt*scaling;
        t+=dt;


        
        
        openmore::RobotStatePtr state = std::make_shared<openmore::RobotState>();
        state->pos_.resize(nax);
        state->vel_.resize(nax);
        state->acc_.resize(nax);
        state->eff_.resize(nax);

       
        openmore::TrjPointPtr point = std::make_shared<openmore::TrjPoint>();
        openmore::TrjPointPtr point_spl = std::make_shared<openmore::TrjPoint>();
        double scaling_spl;
        processor->interpolate(t_nom, point, target_scaling, scaling);

        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        Eigen::VectorXd pos_eigen = Eigen::Map<Eigen::VectorXd>(point->state_->pos_.data(), point->state_->pos_.size());
        Eigen::VectorXd vel_eigen = Eigen::Map<Eigen::VectorXd>(point->state_->vel_.data(), point->state_->vel_.size());
        Eigen::VectorXd acc_eigen = Eigen::Map<Eigen::VectorXd>(point->state_->acc_.data(), point->state_->acc_.size());
        
        std::cout<<"t_inserito: "<<t<<std::endl;
        std::cout<<"t interpolato: "<<point->time_from_start_<<std::endl;

        std::cout << "pos: " << pos_eigen.format(CleanFmt) << std::endl;
        std::cout << "vel: " << vel_eigen.format(CleanFmt) << std::endl;

        file.open(package_path + "trajectory_results.csv", std::ios_base::app);
        file << t;
        for (int i = 0; i < pos_eigen.size(); ++i) {
            file << "," << pos_eigen[i];
        }
        for (int i = 0; i < vel_eigen.size(); ++i) {
            file << "," << vel_eigen[i];
        }
        for (int i = 0; i < acc_eigen.size(); ++i) {
            file << "," << acc_eigen[i];
        }
        file << "," << scaling << "\n"; 
        file.close();

        spline_processor->interpolate(t_nom, point_spl, target_scaling, scaling_spl);
        //DA ELIMINARE: salvo i dati della spline in un csv
        file_spline.open(package_path + "spline_results.csv", std::ios_base::app);
        file_spline << t;
        for (int i = 0; i < point_spl->state_->pos_.size(); ++i) {
            file_spline << "," << point_spl->state_->pos_[i];
        }
        for (int i = 0; i < point_spl->state_->vel_.size(); ++i) {
            file_spline << "," << point_spl->state_->vel_[i];
        }
        for (int i = 0; i < point_spl->state_->acc_.size(); ++i) {
            file_spline << "," << point_spl->state_->acc_[i];
        }
        file_spline << "," << target_scaling << "\n";
        file_spline.close();

        //fine salvataggio

        

    }

    std :: cout << "TUTTO BENE!" << std::endl;
    return 0;
    //std::cout << order << std::endl;
} 