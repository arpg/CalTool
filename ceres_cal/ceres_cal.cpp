
#include <ceres/ceres.h>
#include <stdint.h>
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include <vector>
#include <iostream>
#include <CSVReader.h>
#include <getopt.h>

using namespace std;
using namespace ceres;
using namespace Sophus;
using namespace Eigen;

typedef struct opts
{
  bool useResGen;
  int numPoses;
  char* masterFile;
  char* slaveFile;
  
} opts_t;

void print_usage(const char *prog)
{
  printf("Usage: %s -m <master> -s <slave>\n", prog);
  printf("  -r\t Use the residual generator to verify algorithm\n");
  printf("  -n <count> \t Use <count> poses \n");
  printf("  -m <master>\t Master CSV file\n");
  printf("  -s <slave>\t Slave CSV file\n");
}

void parse_opts(int argc, char *argv[], opts_t *opts)
{
  while (1) {
    static const struct option lopts[] = {
      { "residual",  no_argument, 0, 0 },
      { "count", required_argument, 0, -1},
      { "master", required_argument, 0, 0},
      { "slave", required_argument, 0, 0},
      { NULL, 0, 0, 0 },
    };
    int c;
    
    c = getopt_long(argc, argv, "rn:m:s:", lopts, NULL);
    
    if (c == -1)
      break;
    
    switch (c) {
    case 'r':
      opts->useResGen = true;
      break;
    case 'n':
      opts->numPoses = atoi(optarg);
      break;
    case 'm':
      opts->masterFile = optarg;
      break;
    case 's':
      opts->slaveFile = optarg;
      break;
    default:
      print_usage(argv[0]);
      break;
    }
  }
}

//Assumes that we're given 6dof poses as x,y,z, r, p, y
//Where each trajectory is in the same reference system (robotics, vision, etc)

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


template <typename T>
Sophus::SE3Group<T> makeTransform(Eigen::Matrix<T,6,1> cart)
{
  typename SE3Group<T>::Point trans(cart[0], cart[1], cart[2]);
    
  SO3Group<T> pitch = SO3Group<T>::exp(typename SO3Group<T>::Tangent( T(0), cart[4], T(0)));
  SO3Group<T> yaw = SO3Group<T>::exp(typename SO3Group<T>::Tangent( typename SO3Group<T>::Tangent(T(0), T(0), cart[5])));
  SO3Group<T> roll = SO3Group<T>::exp(typename SO3Group<T>::Tangent(typename SO3Group<T>::Tangent(cart[3], T(0), T(0) )));
    
    
  //The implementation in Sophus doesn't allow for the proper ordering of matrices
  SO3Group<T> rotMatrix = yaw*pitch*roll;
  return Sophus::SE3Group<T>(rotMatrix, trans);
}

struct TrajectoryPanTiltResidual {
  TrajectoryPanTiltResidual(Vector6d m_master, Vector6d m_slave, Vector6d m_pan, Vector6d m_tilt)
  {
    master = m_master;
    slave = m_slave;
    pan = m_pan;
    tilt = m_tilt;
    
  }
  
  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    //X is a thirteen-parameter system, six for pose from base to the pan/tilt unit, six for pan/tilt to the camera, one for scale
    //Prepare a residual for a single measurement
    //Only use x,y,z for now

    //Convert each measurement into SE3 by starting with a Eigen matrix that gets converted into a SE3d via exp()
    Eigen::Matrix<T,6,1> master_t = master.cast <T> ();
    Eigen::Matrix<T,6,1> slave_t = slave.cast <T> ();

    Eigen::Matrix<T,6,1> master_ptBase;
    master_ptBase << x[0],x[1],x[2],x[3],x[4],x[5];
    
    Eigen::Matrix<T,6,1> tiltBase_slave;
    tiltBase_slave << x[6],x[7],x[8],x[9],x[10],x[11];
   
    
    Eigen::Matrix<T,4,4> T_master_ptbase = makeTransform<T>(master_ptBase).matrix();
    Eigen::Matrix<T,4,4> T_ptbase_pan = makeTransform<T>(pan.cast<T>()).matrix();
    Eigen::Matrix<T,4,4> T_pan_tiltbase = makeTransform<T>(tilt.cast<T>()).matrix();
    Eigen::Matrix<T,4,4> T_tiltbase_slave = makeTransform<T>(tiltBase_slave).matrix();
    

    //Scale the translation component
    //Only use the x,y,z for now
    //Represent as homogenous coords to make the matrix transforms easier
    
    Eigen::Matrix<T,4,1> cartMaster;
    cartMaster << master_t.head(3), T(1);
    
    Eigen::Matrix<T,4,1> cartSlave;
    cartSlave << (slave_t.head(3) * T(x[12])), T(1);

    Eigen::Matrix<T,4,4> T_total = T_master_ptbase*T_ptbase_pan*T_pan_tiltbase*T_tiltbase_slave;
    
    //Residual is the difference between the master and the slave positions
    Eigen::Matrix<T,4,1> res = cartMaster - T_total*cartSlave;

    //Mulitply residual by by S^(-1/2) (3x3) to include non-identity covariance data
    //Via chol decomposition
    
    //Only report the difference in x,y,z
     for (int i=0; i<3; i++)
      {
	residual[i] = ceres::abs(T(res(i)));
      }

     
    return true;
  }
  
private:
  Vector6d master, slave, pan, tilt;
};


void residualGenerator()
{
  CSVReader cam0(new string("center_spline.csv"));
  CSVReader cam1(new string("left_spline.csv"));

  vector<double> splits0 = cam0.getNextLineAsDouble();
  vector<double> splits1 = cam1.getNextLineAsDouble();

  int count = 0;

  Vector6d pan;
  Vector6d tilt;

  double panAngle =  1103.0/4096.0*2.0*M_PI;
  double tiltAngle = 2298.0/4096.0*2.0*M_PI;

  pan << 0, 0, 0, 0, 0, panAngle;
  tilt << 0, 0, 0, 0, tiltAngle, 0;

  double resSum = 0.0;
  /*
  for (int skip=0; skip < 899; skip++)
    {
      splits0 = cam0.getNextLineAsDouble();
      splits1 = cam1.getNextLineAsDouble();
    }
  */
  //&& (count < 600)
  while (!splits0.empty() )
    {
      //Test the residual generator first

      
      TrajectoryPanTiltResidual *res = new TrajectoryPanTiltResidual(Map<const Vector6d>(splits0.data()),
								     Map<const Vector6d>(splits1.data()),
								     pan,
								     tilt);

      double cand_x[13] = {0.1, 0.2, 0.3, 0.1, 0.2, 0.3,
			  0.1, 0.2, 0.3, 0.1, 0.2, 0.3,
			  0.5};

      double output[3];
      
      (*res)(cand_x, output);
      for (int i=0; i<3; i++)
	{
	  resSum += output[i];
	  cout << "x[" << i << "]:" << output[i] << endl;
	}
      cout << endl;
      delete res;
      splits0 = cam0.getNextLineAsDouble();
      splits1 = cam1.getNextLineAsDouble();
      count++;
    }

  cout << "Residual sum: " << resSum << endl;
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  /* check command line arguments */
  opts_t options;

  options.useResGen = 0;
  options.masterFile = 0;
  options.slaveFile = 0;
  options.numPoses = -1;
  parse_opts(argc, argv, &options);

  if (options.useResGen)
    {
      cout << "Running residual generator" << endl;
      residualGenerator();
      return 0;
    }

  if (options.masterFile == 0 || options.slaveFile == 0)
    {
      cout << "Please specify files for master and slave trajectories" << endl;
      return 0;
    }
  cout << "Using master file: " << options.masterFile << endl;
  cout << "Using slave file: " << options.slaveFile << endl;
  
  // The variable to solve for with its initial value.
   double pt_x[13] = {0.1, 0.2, 0.3, 0.1, 0.2, 0.3,
		 -0.1, -0.2, -0.3, 0.1, 0.2, 0.3,
		  0.5};
   
   double single_x[7] = {0.1, 0.2, 0.3, 0.1, 0.2, 0.3, 0.5};
  
  // Build the problem.
  Problem pt_problem;
  Problem single_problem;
  
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).

  CSVReader cam0(new string(options.masterFile));
  CSVReader cam1(new string(options.slaveFile));

  vector<double> splits0 = cam0.getNextLineAsDouble();
  vector<double> splits1 = cam1.getNextLineAsDouble();


  //Cost function produces a residual array with 3 elements, with a  13 parameter \hat{x}
  //Separate block for each measurement, but with the same parameters
  int poseCount;
  Vector6d pan;
  Vector6d tilt;

  double panAngle = 1103.0/4096.0*2.0*M_PI;
  double tiltAngle = 2298.0/4096.0*2.0*M_PI;

  pan << 0, 0, 0, 0, 0, panAngle;
  tilt << 0, 0, 0, 0, tiltAngle, 0;
  
  pt_problem.AddParameterBlock(pt_x, 13);
  single_problem.AddParameterBlock(single_x, 7);
  
  while (!splits0.empty() )
    {
      if ((options.numPoses > 0) && (poseCount > options.numPoses))
	{
	  poseCount--;
	  break;
	}
	
      CostFunction* pt_cost_function =
	new AutoDiffCostFunction<TrajectoryPanTiltResidual, 3, 13>(new TrajectoryPanTiltResidual(Map<const Vector6d>(splits0.data()),
											  Map<const Vector6d>(splits1.data()),
											  pan,
											  tilt));

      pt_problem.AddResidualBlock(pt_cost_function, NULL, pt_x);
      CostFunction* single_cost_function =
	new AutoDiffCostFunction<TrajectoryResidual, 3, 7>(new TrajectoryResidual(Map<const Vector6d>(splits0.data()),
											  Map<const Vector6d>(splits1.data())));
      single_problem.AddResidualBlock(single_cost_function, NULL, single_x);
      splits0 = cam0.getNextLineAsDouble();
      splits1 = cam1.getNextLineAsDouble();
      poseCount++;
    }

  cout << "Solving against " << poseCount << " poses" << endl;
  
  // Run the solver!
  Solver::Options c_options;
  c_options.linear_solver_type = ceres::DENSE_QR;
  c_options.minimizer_progress_to_stdout = true;
  c_options.function_tolerance = 1e-8;
  c_options.gradient_tolerance = 1e-8;
  c_options.use_nonmonotonic_steps = true;

  
  //options.minimizer_type = ceres::LINE_SEARCH;
  //options.line_search_direction_type = BFGS;
  
  //options.trust_region_strategy_type = DOGLEG;
  //options.dogleg_type = SUBSPACE_DOGLEG;
  
  //options.trust_region_minimizer_iterations_to_dump.push_back(0);
  //options.trust_region_minimizer_iterations_to_dump.push_back(1);
  //options.trust_region_minimizer_iterations_to_dump.push_back(2);
  //options.trust_region_problem_dump_format_type = CONSOLE;
  //options.check_gradients = true;

  
  c_options.max_num_iterations = 100;
  Solver::Summary summary;
  Solve(c_options, &pt_problem, &summary);

  cout << summary.BriefReport() << "\n";
  cout << "Parameters: x :->\n";
  for (int i=0; i<13; i++)
      {
	cout << "x[" << i << "]:" << pt_x[i] << endl;
      }

  for (int i=0; i<13; i++)
      {
	cout << pt_x[i] << ", ";
      }
  cout << endl;

  //cout << summary.FullReport() << "\n";

  //Compute covariance of solution
  Solve(c_options, &single_problem, &summary);
  cout << summary.BriefReport() << "\n";
  Covariance::Options cov_options;
  //cov_options.algorithm_type = DENSE_SVD;
  // cov_options.min_reciprocal_condition_number = 1e-35;
  Covariance covariance(cov_options);

  vector<pair<const double*, const double*> > covariance_blocks;
  covariance_blocks.push_back(make_pair(single_x, single_x));
  CHECK(covariance.Compute(covariance_blocks, &single_problem));

  double covariance_xx[7 * 7];

  covariance.GetCovarianceBlock(single_x, single_x, covariance_xx);

  //Output covariance block diagonals:

  cout << "Covariances:" << endl;
  for (int i=0; i< 7; i++)
    {
      cout << "Var(x[" << i << "]) = " << covariance_xx[i] << endl;
    }
  return 0;
}
