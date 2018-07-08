#include "GraphBasedBA.h"


//class for a node
//class for an edge
//appli class to read nodes and store them G20 format
//class for all poses

//firstcreate in MM code to export triplets in g20 format
//code to read (ceres ex)
//visualise the poses with python

//we need initial values for all positions
// scenario1 : i read only the relative orientation here and find the global poses
// scenario2 : i read relative and initial absolute => gain : least squares adjustement

// TODO:
//  + scenario2
//  + (maybe) scenario


class RotMat
{
    public :
        RotMat(const Eigen::Vector3d &aI,const Eigen::Vector3d &aJ,const Eigen::Vector3d &aK) :
            mI(aI),
            mJ(aJ),
            mK(aK) {};

        Eigen::Vector3d & I(){return mI;}
        Eigen::Vector3d & J(){return mJ;}
        Eigen::Vector3d & K(){return mK;}

        Eigen::Vector4d & Quat(){return mQuat;}

    private :
        Eigen::Vector3d   mI;
        Eigen::Vector3d   mJ;
        Eigen::Vector3d   mK;

        Eigen::Vector4d   mQuat;
};

int RotMat2Quat()
{


    return 1;
}


void PrintTestGraphHelp(std::string aArg)
{
    std::cerr << "Usage: " << aArg << " "
              << "Options:\n"
              << "\tGBA  <path to g2o file with poses and constraints>\n"
              << std::endl;

}

int TestGraphBA_main(int argc, char** argv)
{
    if (argc < 3 )
    {
        PrintTestGraphHelp(argv[1]);
        return(0);
    }

    std::string aG2oFile = std::string(argv[2]);
    std::cout << "G2oFile: " << aG2oFile << "\n";

    Eigen::Matrix3d aM(3,3);
    Eigen::Matrix3d aMOut(3,3);
    aM << 0.5, 0, 0,
          0.8, 1, 0,
          0, 0, 1;

    std::cout << aM << "\n";

    ceres::examples::MapOfPoses poses;
    ceres::examples::VectorOfConstraints constraints;


    CHECK(ceres::examples::ReadG2oFile(aG2oFile, &poses, &constraints)) << "sss";
    std::cout << "Number of poses: " << poses.size() << '\n';
    std::cout << "Number of constraints: " << constraints.size() << '\n';

    Eigen::Quaterniond aMQ;
    //ceres::RotationMatrixToAngleAxis(&aM(0,0),&aMOut(0,0));
    ceres::RotationMatrixToQuaternion(&aM(0,0), &aMQ.w());
    std::cout << "quaternionT=" << aMQ.w() << " " << aMQ.x() << " " << aMQ.y() << " " << aMQ.z() << "\n" ;


    return true;
}


/*
DEFINE_string(input, "", "The pose graph definition filename in g2o format.");
namespace ceres {
namespace examples {
// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses, ceres::Problem* problem) {
  CHECK(poses != NULL);
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }
  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization* quaternion_local_parameterization =
      new EigenQuaternionParameterization;
  for (VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const Constraint3d& constraint = *constraints_iter;
    MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";
    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);
    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                             pose_end_iter->second.q.coeffs().data());
    problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
  }
  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  MapOfPoses::iterator pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}
// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != NULL);
  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);
  std::cout << summary.FullReport() << '\n';
  return summary.IsSolutionUsable();
}
// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose3d, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
        value_type& pair = *poses_iter;
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}
}  // namespace examples
}  // namespace ceres
int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(FLAGS_input != "") << "Need to specify the filename to read.";
  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;


  CHECK(ceres::examples::ReadG2oFile(FLAGS_input, &poses, &constraints))
      << "Error reading the file: " << FLAGS_input;
  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';
  CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
      << "Error outputting to poses_original.txt";
  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
  CHECK(ceres::examples::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";
  CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
      << "Error outputting to poses_original.txt";
  return 0;
}*/

