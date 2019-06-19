/*
 * psn.hpp
 * Author: Prabal Basu
 * Email: prabalb@aggiemail.usu.edu
 */

#include "config.h"
#include "config_utils.hpp"

#include <vector>
#include <map>
#include <utility>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;

#ifndef PSN_H
#define PSN_H

typedef map<int, vector<pair<int, int> > > routerProcess;
typedef map<int, vector<double> > routerLoads;

class PSN {
  
private:
  routerProcess rP;
  routerLoads   rL;
  vector<vector<float> > regionDroop;

  int lidx;
  int m;
  int np;

  double maxLoad;
  bool dumpAvgDroop;

  std::vector<float> lds;

  RowVectorXd node1List;
  RowVectorXd node2List;
  
  RowVectorXd vpi;
  RowVectorXd res;
  RowVectorXd ind;
  RowVectorXd cap;
  RowVectorXd peakNodeVoltage;
  RowVectorXd peakRouterVoltage;
  
  VectorXd gNodes;
  VectorXd vfi;

  MatrixXd nodeMatrix;
  MatrixXd cycle_load;
  MatrixXd avr_load;
  MatrixXd fp; 
  MatrixXd cl; 
  MatrixXd vp; 
  
  SparseMatrix<double> xg; 

  std::ofstream output_file;

private:
  PSN(Configuration const & config) {
    initialize(config);
  }
  void operator=(PSN&);
  PSN(const PSN&);

public:
  ~PSN() {
    if(output_file.is_open()) {
      output_file.close();
    }
  }

  static PSN& getPSNObj(Configuration const & config) {
    static PSN psn(config);
    return psn;
  }

  void initialize(Configuration const & config);
  void populateDBs();

  void clearRouterProcessInfo();
  void collectRouterProcessInfo(int routerID, int process, int channel=-1);
  void generateLoadInfo(int cycle);
  void generateAvrLoad();
  void analyseLoads();
  void dumpNodeVoltages(int cycle);
  void dumpPeakNodeVoltages();
  double getDroopPercentage(double v);

  bool reportAvgDroop();
  bool parseGridFile();
  bool dlmRead();

  void createSparseMatrix(RowVectorXd &xt);
  void createDoubleSizedVector(RowVectorXd &vec);
  void createNodeMatrix();

  VectorXd uniquify(MatrixXd &mat, int col);
  VectorXd setDiff(VectorXd &gNodes, RowVectorXd &vpi);
  VectorXd find(VectorXd &tVec, double fi);
  VectorXd getSubMatrix(MatrixXd &mat, VectorXd &v, int col);

  double sum(MatrixXd &mat, int row, VectorXd &v);
  double sum(RowVectorXd &vec1, VectorXd &vec2);

  void distributeLoad(MatrixXd &cl, RowVectorXd &tile_nodes, double load);

};

#endif // PSN_H

