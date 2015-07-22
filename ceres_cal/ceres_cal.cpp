
#include <ceres/ceres.h>
#include <stdint.h>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <vector>
#include <iostream>
#include <CSVReader.h>

using namespace std;
using namespace ceres;
using namespace Sophus;
using namespace Eigen;

//Assumes that we're given 6dof poses as x,y,z, r, p, y
struct TrajectoryResidual {
  TrajectoryResidual(Vector6d m_master, Vector6d m_slave)
  {
    master = m_master;
    slave = m_slave;
  }
  
  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    //X is a seven-parameter system, six for pose, one for scale
    //Prepare a residual for a single measurement
    //Only use x,y,z for now

    //Convert each measurement into SE3 by starting with a Eigen matrix that gets converted into a SE3d via exp()
    Eigen::Matrix<T,6,1> master_t = master.cast <T> ();
    Eigen::Matrix<T,6,1> slave_t = slave.cast <T> ();
    // Eigen::Matrix<T,6,1> pose_in;
    SE3Group<T> Tab;

    typename SE3Group<T>::Point trans(x[0], x[1], x[2]);

    
    SO3Group<T> pitch = SO3Group<T>::exp(typename SO3Group<T>::Tangent(x[4], T(0), T(0)));
    SO3Group<T> yaw = SO3Group<T>::exp(typename SO3Group<T>::Tangent( typename SO3Group<T>::Tangent(T(0), x[5], T(0))));
    SO3Group<T> roll = SO3Group<T>::exp(typename SO3Group<T>::Tangent(typename SO3Group<T>::Tangent(T(0), T(0), x[3])));


    //The implementation in Sophus doesn't allow for the proper ordering of matrices
    
    SO3Group<T> rotMatrix = yaw*pitch*roll;


    Tab = Sophus::SE3Group<T>(rotMatrix, trans);
    
    //cout << "Tab:" << Tab << endl;
    
    //Scale the translation component

    //Tab.translation() = Tab.translation() * T(x[6]);
    //Only use the x,y,z for now

    Eigen::Matrix<T,3,1> cartMaster = master_t.head(3);
    Eigen::Matrix<T,4,1> cartSlave;
    cartSlave << (slave_t.head(3) * T(x[6])), T(1);
    
    //Residual is the difference between the master and the slave positions
    Eigen::Matrix<T,3,1> res = cartMaster - Tab.matrix3x4() * cartSlave;

    //Mulitply residual by by S^(-1/2) (3x3) to include non-identity covariance data
    //Via chol decomposition
    
    //Only report the difference in x,y,z
     for (int i=0; i<3; i++)
      {
	residual[i] = res(i);
      }

     
    return true;
  }
  
private:
  Vector6d master, slave;
};

struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

void residualGenerator()
{
  CSVReader cam0(new string("cposes_0.csv"));
  CSVReader cam1(new string("cposes_1.csv"));

  vector<double> splits0 = cam0.getNextLineAsDouble();
  vector<double> splits1 = cam1.getNextLineAsDouble();

  int count = 0;
  while (!splits0.empty() && (count < 10))
    {
      //Test the residual generator first

      TrajectoryResidual *res = new TrajectoryResidual(Map<const Vector6d>(splits0.data()), Map<const Vector6d>(splits1.data()));
      double cand_x[7] = {0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 5};

      double output[3];
      
      (*res)(cand_x, output);
      for (int i=0; i<3; i++)
	{
	  cout << "x[" << i << "]:" << output[i] << endl;
	}
      
      delete res;
      splits0 = cam0.getNextLineAsDouble();
      splits1 = cam1.getNextLineAsDouble();
      count++;
    }
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  //residualGenerator();
  //return 0;
  
  
  // The variable to solve for with its initial value.
  double x[7] = {0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 1};


  
  // Build the problem.
  Problem problem;

  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

  CSVReader cam0(new string("cposes_0.csv"));
  CSVReader cam1(new string("cposes_1.csv"));

  vector<double> splits0 = cam0.getNextLineAsDouble();
  vector<double> splits1 = cam1.getNextLineAsDouble();


  //Cost function produces a residual array with 3 elements, with a 7 parameter \hat{x}
  //Separate block for each measurement, but with the same parameters
  int poseCount;
  
  while (!splits0.empty())
    {
      CostFunction* cost_function =
	new AutoDiffCostFunction<TrajectoryResidual, 3, 7>(new TrajectoryResidual(Map<const Vector6d>(splits0.data()), Map<const Vector6d>(splits1.data())));
      problem.AddResidualBlock(cost_function, NULL, x);

      splits0 = cam0.getNextLineAsDouble();
      splits1 = cam1.getNextLineAsDouble();
      poseCount++;
    }

  cout << "Solving against " << poseCount << " poses" << endl;
  
  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.function_tolerance = 1e-16;
  options.gradient_tolerance = 1e-16;
  options.use_nonmonotonic_steps = true;

  
  //options.minimizer_type = ceres::LINE_SEARCH;
  //options.line_search_direction_type = BFGS;
  
  //options.trust_region_strategy_type = DOGLEG;
  //options.dogleg_type = SUBSPACE_DOGLEG;
  
  //options.trust_region_minimizer_iterations_to_dump.push_back(0);
  //options.trust_region_minimizer_iterations_to_dump.push_back(1);
  //options.trust_region_minimizer_iterations_to_dump.push_back(2);
  //options.trust_region_problem_dump_format_type = CONSOLE;
  //options.check_gradients = true;

  
  options.max_num_iterations = 100;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

  cout << summary.BriefReport() << "\n";
  cout << "x :->\n";
  for (int i=0; i<7; i++)
      {
	cout << "x[" << i << "]:" << x[i] << endl;
      }
  //cout << summary.FullReport() << "\n";

  //Compute covariance of solution
  Covariance::Options cov_options;
  Covariance covariance(cov_options);

  vector<pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.push_back(make_pair(x, x));
  CHECK(covariance.Compute(covariance_blocks, &problem));

  double covariance_xx[7 * 7];

  covariance.GetCovarianceBlock(x, x, covariance_xx);

  //Output covariance block diagonals:
  for (int i=0; i< 7; i++)
    {
      cout << "Var(x[" << i << "]) = " << covariance_xx[i] << endl;
    }
  return 0;
}
