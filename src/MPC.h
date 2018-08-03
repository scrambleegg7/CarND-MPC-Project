#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <fstream>

using namespace std;

class MPC {
 public:
  MPC();

  const double _Lf = 2.67;

  virtual ~MPC();

  std::ofstream ostream_data;  
  void setFilename(std::string f);
  void writeData(std::string d, vector<double> data_vals);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
