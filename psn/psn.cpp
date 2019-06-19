/*
 * psn.cpp
 * Author: Prabal Basu
 * Email: prabalb@aggiemail.usu.edu
 */

#include "psn.hpp"
#include "routers/iq_router.hpp"

#include "cstdio"

#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>
#include <cmath>

void PSN::initialize(Configuration const & config)
{
  regionDroop.resize(16);

  node1List.resize(NO_RLC_COMB);
  node2List.resize(NO_RLC_COMB);
  
  vpi.resize(NO_VOLTAGE_PINS);
  
  res.resize(NO_RLC_COMB);
  ind.resize(NO_RLC_COMB);
  cap.resize(NO_RLC_COMB);

  peakNodeVoltage = RowVectorXd::Ones(GRID_SIZE);
  peakRouterVoltage = RowVectorXd::Ones(NO_OF_TILES);
  
  nodeMatrix.resize(2*NO_RLC_COMB,2);
  fp.resize(NO_OF_TILES, int(GRID_SIZE) / int(NO_OF_TILES));
  avr_load.resize(1, NO_OF_TILES);

  std::string output_file_name = config.GetStr("psn_output_file");
  output_file.open(output_file_name.c_str());
  if(!output_file.good()) {
    std::cout << "Bad PSN output file ..." << std::endl;
    exit(-1);
  }
  output_file.sync_with_stdio(false);

  std::ifstream linkLoad;
  linkLoad.open("./psn/rndlds25.txt");
  if(!linkLoad.good()) {
    std::cout << "Could not find link load file ..." << std::endl;
    exit(1);
  }

  while(!linkLoad.eof()) {
    char buf[2048];
    linkLoad.getline(buf, 2048);
    char *c = strtok(buf, " ");
    if(!c) continue; // check for blank line
    lds.push_back(atof(c));
  }
  linkLoad.close();
  lidx = 0;

  maxLoad = 0.0;
  dumpAvgDroop = (config.GetInt("average_psn") > 0);
}

void PSN::populateDBs()
{
  // create double sized row-vectors (e.g. x = [x x])
  createDoubleSizedVector(res);
  createDoubleSizedVector(ind);
  createDoubleSizedVector(cap);

  // create nodeMatrix
  createNodeMatrix();

  RowVectorXd xt(2*NO_RLC_COMB);
  float ts = 1/FREQUENCY;
  xt = 6*ind + 3*res*ts;
  for(int i = 0; i < 2*NO_RLC_COMB; i++) {
    xt(i) = pow(ts,2) / xt(i);
  }
  //std::cout << "xt: " << xt << std::endl;
  
  // populate the sparse matrix xg
  createSparseMatrix(xt);

  // uniquify first (0th) column of nodeMatrix
  gNodes = uniquify(nodeMatrix, 0);
  //std::cout << "gNodes: " << gNodes << std::endl << std::endl;

  cl = MatrixXd::Zero(gNodes.size(), 1);
  
  // free voltage indexes (all except vpi)
  vfi = setDiff(gNodes, vpi);
  //std::cout << "vfi: " << vfi << std::endl;

  m = vfi.size();
  np = vpi.size();

  vp = MatrixXd::Ones(np, 1) * VDD;

}

void PSN::clearRouterProcessInfo()
{
  rP.clear();
}

void PSN::collectRouterProcessInfo(int routerID, int process, int channel)
{
  if(rP.find(routerID) == rP.end()) {
    vector<pair<int, int> > p;
    p.push_back(make_pair(process, channel));
    rP[routerID] = p;
  } else {
    rP[routerID].push_back(make_pair(process, channel));
  }
}

bool PSN::reportAvgDroop() {
  return dumpAvgDroop;
}

void PSN::generateAvrLoad()
{
  if(rP.size() == 0) return;

  routerProcess::iterator rpIter;
  for(rpIter = rP.begin(); rpIter != rP.end(); rpIter++) {
    int router = rpIter->first;
    vector<pair<int, int> > p = rpIter->second;
    for(size_t i = 0; i < p.size(); i++) {
    
      int process = p[i].first;

      int mxlds = lds.size();
      double proc_load = 0.0;
      switch(process) {
        case PROCESS_INCOMING: {
          proc_load = PWR_INCOMING;
          break;
        }
        case PROCESS_SWALLOC: {
          proc_load = PWR_SWALLOC;
          break;
        }
        case PROCESS_XBAR: {
          proc_load = PWR_XBAR;
          break;
        }
        case PROCESS_FORWARD: {
          proc_load = PWR_FORWARD;
          int output_ch = p[i].second;
          if(output_ch != DIRECTION_LOCAL) {
            lidx = (lidx % mxlds);
            double link_load = lds[lidx];
            proc_load += link_load;
          }
          break;
        }
        case PROCESS_STANDBY: {
          proc_load = PWR_STANDBY;
          break;
        }
      }
      avr_load(router) += proc_load;
    }
  }
}

void PSN::generateLoadInfo(int cycle)
{
  if(rP.size() == 0) return;

  cycle_load.resize(0, 0); // clear older content
  cycle_load.resize(1, NO_OF_TILES);
  cycle_load = MatrixXd::Zero(1, NO_OF_TILES);

  routerProcess::iterator rpIter;
  for(rpIter = rP.begin(); rpIter != rP.end(); rpIter++) {
    int router = rpIter->first;
    vector<pair<int, int> > p = rpIter->second;
    for(size_t i = 0; i < p.size(); i++) {
    
      int process = p[i].first;

      int mxlds = lds.size();
      double proc_load = 0.0;
      switch(process) {
        case PROCESS_INCOMING: {
          proc_load = PWR_INCOMING;
          break;
        }
        case PROCESS_SWALLOC: {
          proc_load = PWR_SWALLOC;
          break;
        }
        case PROCESS_XBAR: {
          proc_load = PWR_XBAR;
          break;
        }
        case PROCESS_FORWARD: {
          proc_load = PWR_FORWARD;
          int output_ch = p[i].second;
          if(output_ch != DIRECTION_LOCAL) {
            lidx = (lidx % mxlds);
            double link_load = lds[lidx];
            proc_load += link_load;
          }
          break;
        }
        case PROCESS_STANDBY: {
          proc_load = PWR_STANDBY;
          break;
        }
      }
      cycle_load(router) += proc_load;
    }
  }

  /*
  std::vector<double> router_loads;
  for(int i = 0; i < cycle_load.size(); i++) {
    router_loads.push_back(cycle_load(i));
    if(cycle_load(i) > maxLoad) {
      maxLoad = cycle_load(i);
    }
  }
  rL[cycle] = router_loads;
  */
}

void PSN::analyseLoads()
{
  routerLoads::iterator rlIter;
  for(rlIter = rL.begin(); rlIter != rL.end(); rlIter++) {
    int cycle = (*rlIter).first;
    std::vector<double> router_loads = (*rlIter).second;
    double maxCycleLoad = *std::max_element(router_loads.begin(), router_loads.end());
    if(float(maxCycleLoad)*100/maxLoad > 90) {
      output_file << cycle << "\n";
    }
  }
}

void PSN::dumpNodeVoltages(int cycle)
{
  MatrixXd qff = MatrixXd::Zero(m,m);
  MatrixXd qfp = MatrixXd::Zero(m,np);
  MatrixXd bf  = MatrixXd::Zero(m,1);

  // create a non-sparse copy of xg, sparse matrix (in Eigen) does not support some operations
  MatrixXd xg_nsp(xg.rows(), xg.cols());
  xg_nsp = xg;

  RowVectorXd router_loads(NO_OF_TILES);
  if(!reportAvgDroop()) {
    for(int i = 0; i < cycle_load.size(); i++) {
      router_loads(i) = cycle_load(i);
    }
  } else {
    std::cout << "\nCalculating average power supply noise ..." << std::endl;
    for(int i = 0; i < avr_load.size(); i++) {
      router_loads(i) = avr_load(i)/cycle;
    }
  }

  for(int r = 0; r < NO_OF_TILES; r++) { // for all switching routers
    RowVectorXd tile_nodes(fp.cols());
    tile_nodes << fp.row(r);
    double router_workload = router_loads(r);
    double avgLoad = router_workload / tile_nodes.size(); // distribute evenly
    distributeLoad(cl, tile_nodes, avgLoad);
  }
  
  // computing Qff
  // std::cout << "computing Qff ..." << std::endl;
  for(int j = 0; j < m; j++) {
    int fi = vfi(j);
    VectorXd tVec(2*NO_RLC_COMB);
    tVec << nodeMatrix.col(0);
    VectorXd jj = find(tVec, fi);
    VectorXd jn = getSubMatrix(nodeMatrix, jj, 1);
    double yj = sum(xg_nsp, fi, jn) + 0.5 * sum(cap, jj) + cl(fi);
    for(int k = 0; k < m; k++) {
      if(xg_nsp(fi, vfi(k)) > 0) { // save time
          qff(j,k) = xg_nsp(fi, vfi(k)) / yj;
      }
    }
    qff(j,j) = -1;
  }


  // computing Qfp
  // std::cout << "computing Qfp ..." << std::endl;
  for(int j = 0; j < m; j++) {
    int fi = vfi(j);
    VectorXd tVec(2*NO_RLC_COMB);
    tVec << nodeMatrix.col(0);
    VectorXd jj = find(tVec, fi);
    VectorXd jn = getSubMatrix(nodeMatrix, jj, 1);
    double yj = sum(xg_nsp, fi, jn) + 0.5 * sum(cap, jj) + cl(fi);

    for(int k = 0; k < np; k++) {
      qfp(j,k) = xg_nsp(fi, vpi(k)) / yj;
    }
  }

  // computing bf
  // std::cout << "computing bf ..." << std::endl;
  for(int i = 0; i < m; i++) {
    // int fi = vfi(i); // ********DEBUG*****
    int fi = vfi(np - 1);
    VectorXd tVec(2*NO_RLC_COMB);
    tVec << nodeMatrix.col(0);
    VectorXd jj = find(tVec, fi);
    VectorXd jn = getSubMatrix(nodeMatrix, jj, 1);
    double yj = sum(xg_nsp, fi, jn) + 0.5 * sum(cap, jj) + cl(fi);
    bf(i) = -1*(1 / (2 * yj)) * sum(cap, jj) * VDD;
  }

  MatrixXd b = bf - (qfp*vp);
  // MatrixXd vf = qff.colPivHouseholderQr().solve(b); // Ax = b; => x = A\b;
  MatrixXd vf = qff.householderQr().solve(b); // much faster, and accurate!!

  // Merging the voltages vp and vf
  // std::cout << "merging vp and vf ..." << std::endl;
  RowVectorXd vr(m + np);
  for(int i = 0; i < m; i++) {
    vr(vfi(i)) = vf(i);
  }

  for(int i = 0; i < np; i++) {
    vr(vpi(i)) = vp(i);
  }
   
  // track peak droop
  for(int i = 0; i < vr.size(); i++) {
    if(vr(i) < peakNodeVoltage(i)) {
      peakNodeVoltage(i) = vr(i);
    }
  }
  for(int r = 0; r < NO_OF_TILES; r++) {
    RowVectorXd tile_nodes(fp.cols());
    tile_nodes << fp.row(r);
    double v = 0.0;
    for(int i = 0; i < tile_nodes.size(); i++) {
      v += vr(tile_nodes(i));
    }
    v = v / tile_nodes.size();
    if(v < peakRouterVoltage(r)) {
      peakRouterVoltage(r) = v;
    }
  }

  if(!reportAvgDroop()) { // dump cycle wise node voltages
    /*
    output_file << cycle << " " << std::fixed;
    for(int i = 0; i < vr.size(); i++) {
      output_file << vr(i) << " ";
    }
    output_file << "\n";
    */

/////////////////// for CDF ///////////////////

    RowVectorXd rtrPeak = RowVectorXd::Ones(NO_OF_TILES);
    for(int r = 0; r < NO_OF_TILES; r++) {
      RowVectorXd tile_nodes(fp.cols());
      tile_nodes << fp.row(r);
      for(int i = 0; i < tile_nodes.size(); i++) {
        if(rtrPeak(r) > vr(tile_nodes(i))) {
          rtrPeak(r) = vr(tile_nodes(i));
        }
      }
    }
    RowVectorXd rgnPeak = RowVectorXd::Ones(NO_OF_TILES/4);
    Group &grp = Group::getGroupObj();
    for(int i = 0; i < NO_OF_TILES; i++) {
      int region = grp.getGroup(i);
      assert(region >= 0 && region <= 15);
      if(rgnPeak(region) > rtrPeak(i)) {
        rgnPeak(region) = rtrPeak(i);
      }   
    }
    for(int i = 0; i < rgnPeak.size(); i++) {
      regionDroop[i].push_back(getDroopPercentage(rgnPeak(i)));
    }

//////////////////////////////////////////////

  } else { // dump average voltages of nodes, routers and regions
    // grid nodes
    output_file << "Average Node Voltages" << "\n";
    output_file << "=====================" << "\n";

    for(int i = 0; i < vr.size(); i++) {
      output_file << i << " " << vr(i) << " " << getDroopPercentage(vr(i)) << "\n";
    }
    output_file << "\n\n";

    // routers
    output_file << "Average Router Voltages" << "\n";
    output_file << "=======================" << "\n";

    RowVectorXd rtrAvg = RowVectorXd::Zero(NO_OF_TILES);
    for(int r = 0; r < NO_OF_TILES; r++) {
      RowVectorXd tile_nodes(fp.cols());
      tile_nodes << fp.row(r);
      for(int i = 0; i < tile_nodes.size(); i++) {
        rtrAvg(r) += vr(tile_nodes(i));
      }
      rtrAvg(r) = rtrAvg(r) / tile_nodes.size();
    }
    for(int i = 0; i < NO_OF_TILES; i++) {
      output_file << i << " " << rtrAvg(i) << " " << getDroopPercentage(rtrAvg(i)) << "\n";
    }
    output_file << "\n\n";

    // regions
    output_file << "Average Regional Voltages" << "\n";
    output_file << "=========================" << "\n";

    RowVectorXd rgnAvg = RowVectorXd::Zero(NO_OF_TILES/4);
    Group &grp = Group::getGroupObj();
    for(int i = 0; i < NO_OF_TILES; i++) {
      int region = grp.getGroup(i);
      assert(region >= 0 && region <= 15);
      rgnAvg(region) += rtrAvg(i);
    }
    rgnAvg = rgnAvg / 4;
    for(int i = 0; i < rgnAvg.size(); i++) {
      output_file << i << " " << rgnAvg(i) << " " << getDroopPercentage(rgnAvg(i)) << "\n";
    }
    std::cout << "Finished calculating average power supply noise ..." << std::endl << std::endl;
  }
}

double PSN::getDroopPercentage(double v)
{
  return (0.7 - v)*100/0.7;
}

void PSN::dumpPeakNodeVoltages()
{
  output_file << "\n\n";

  for(size_t i = 0; i < regionDroop.size(); i++) {
    std::vector<float> cycleDroop = regionDroop[i];
    std::sort(cycleDroop.begin(), cycleDroop.end());
    int pos1 = cycleDroop.size() * 0.95;
    output_file << i << " " << cycleDroop[pos1-1];
    int pos2 = cycleDroop.size() * 0.90;
    output_file << " " << cycleDroop[pos2-1];
    int pos3 = cycleDroop.size() * 0.85;
    output_file << " " << cycleDroop[pos3-1];
    int pos4 = cycleDroop.size() * 0.80;
    output_file << " " << cycleDroop[pos4-1];
    int pos5 = cycleDroop.size() * 0.75;
    output_file << " " << cycleDroop[pos5-1];
    int pos6 = cycleDroop.size() * 0.70;
    output_file << " " << cycleDroop[pos6-1];
    output_file << "\n";
  }

  // grid nodes
  output_file << "\n\n";
  output_file << "Peak Node Voltages" << "\n";
  output_file << "==================" << "\n";

  for(int i = 0; i < peakNodeVoltage.size(); i++) {
    output_file << i << " " << peakNodeVoltage(i) << " " << getDroopPercentage(peakNodeVoltage(i)) << "\n";
  }

  output_file << "\n\n";

  // routers
  output_file << "Peak Router Voltages 1" << "\n";
  output_file << "======================" << "\n";

  RowVectorXd rtrPeak = RowVectorXd::Ones(NO_OF_TILES);
  for(int r = 0; r < NO_OF_TILES; r++) {
    RowVectorXd tile_nodes(fp.cols());
    tile_nodes << fp.row(r);
    for(int i = 0; i < tile_nodes.size(); i++) {
      if(rtrPeak(r) > peakNodeVoltage(tile_nodes(i))) {
        rtrPeak(r) = peakNodeVoltage(tile_nodes(i));
      }
    }
  }
  for(int i = 0; i < NO_OF_TILES; i++) {
    output_file << i << " " << rtrPeak(i) << " " << getDroopPercentage(rtrPeak(i)) << "\n";
  }
  output_file << "\n\n";

  output_file << "Peak Router Voltages 2" << "\n";
  output_file << "======================" << "\n";

  for(int i = 0; i < peakRouterVoltage.size(); i++) {
    output_file << i << " " << peakRouterVoltage(i) << " " << getDroopPercentage(peakRouterVoltage(i)) << "\n";
  }

  // regions
  output_file << "Peak Regional Voltages" << "\n";
  output_file << "======================" << "\n";

  RowVectorXd rgnPeak = RowVectorXd::Ones(NO_OF_TILES/4);
  Group &grp = Group::getGroupObj();
  for(int i = 0; i < NO_OF_TILES; i++) {
    int region = grp.getGroup(i);
    assert(region >= 0 && region <= 15);
    if(rgnPeak(region) > rtrPeak(i)) {
      rgnPeak(region) = rtrPeak(i);
    }   
  }
  for(int i = 0; i < rgnPeak.size(); i++) {
    output_file << i << " " << rgnPeak(i) << " " << getDroopPercentage(rgnPeak(i)) << "\n";
  }
}

// utility methods

bool PSN::parseGridFile()
{
  std::ifstream gridFile;  
  gridFile.open(GRID_FILE);

  if(!gridFile.good()) {
    return false;
  }
  
  std::cout << "Parsing grid file ..." << std::endl;

  int index = 0;
  int vpiIndex = 0;
  while(!gridFile.eof()) {
    char buf[2048];
    gridFile.getline(buf, 2048);
    if(!strtok(buf, " ")) continue; // check for blank line
    if(buf[0] == 'N') { // grid segment n1_id_x1_y1, n2_id_x2_y2, R, L, C
      char *s1 = strtok(0, " _");
      node1List(index) = atoi(s1) - 1;

      strtok(0, " ");
      char *s2 = strtok(0, " _");
      node2List(index) = atoi(s2) - 1;

      strtok(0, " ");
      char *s3 = strtok(0, " ");
      res(index) = atof(s3);

      char *s4 = strtok(0, " ");
      ind(index) = atof(s4);

      char *s5 = strtok(0, " ");
      cap(index) = atof(s5);
      index++;
    } else if(buf[0] == 'V') { // package pin V, id, VDD
      char *id = strtok(0, " ");
      vpi(vpiIndex) = atoi(id) - 1;
      vpiIndex++;
    }
  }
  gridFile.close();

  std::cout << "Finished parsing grid file ..." << std::endl << std::endl;
  return true;
}

// dlmRead(floorplan, ",");
bool PSN::dlmRead()
{
  std::ifstream fpFile;  
  fpFile.open(FLOORPLAN_FILE);

  if(!fpFile.good()) {
    return false;
  }

  std::cout << "Parsing floorplan file ..." << std::endl;

  int row = NO_OF_TILES; // number of routers
  int col = int(GRID_SIZE) / int(NO_OF_TILES); // each router corresponds to 'col' grid points

  int r = 0;
  while(!fpFile.eof() || r < row) {
    char buf[2048];
    fpFile.getline(buf, 2048);
    const char *token[2048] = {};
    token[0] = strtok(buf, ",");
    if(!token[0]) continue; // check for blank line
    fp(r,0) = atoi(token[0]) - 1;
    for(int i = 1; i < col ; i++) {
      char *p = strtok(0, ",");
      fp(r,i) = atoi(p) - 1;
    }
    r++;
  }
  fpFile.close();

  std::cout << "Finished parsing floorplan file ..." << std::endl << std::endl;
  return true;
}

void PSN::createDoubleSizedVector(RowVectorXd &vec)
{
  int dSize = 2 * vec.size();
  RowVectorXd bigVec(dSize);
  bigVec << vec,vec;
  vec.resize(dSize);
  vec << bigVec;
}

// nodeMatrix = [node1List' node2List' ; node2List' node1List']
void PSN::createNodeMatrix() 
{
  VectorXd node1ListT(node1List.size()); 
  node1ListT = node1List.transpose();

  VectorXd node2ListT(node2List.size()); 
  node2ListT = node2List.transpose();

  nodeMatrix.col(0) << node1ListT, node2ListT;
  nodeMatrix.col(1) << node2ListT, node1ListT;
}

VectorXd PSN::uniquify(MatrixXd &mat, int col)
{
  int s = 2*NO_RLC_COMB;
  VectorXd tVec(s);
  tVec << mat.col(col);

  std::set<int> tSet;
  for(int i = 0; i < s; i++) {
    tSet.insert(tVec(i));
  }

  VectorXd gNodes(tSet.size());
  int i = 0;
  for(std::set<int>::iterator it = tSet.begin(); it != tSet.end(); it++,i++) {
    gNodes(i) = *it;
  }
  return gNodes;
}

void PSN::createSparseMatrix(RowVectorXd &xt)
{
  VectorXd vec1;
  vec1 = nodeMatrix.col(0);
  VectorXd vec2;
  vec2 = nodeMatrix.col(1);

  std::map<std::pair<double, double>, double> gMap;
  for(int i = 0; i < 2*NO_RLC_COMB; i++) {
    double a = vec1(i);
    double b = vec2(i);
    std::pair <double,double> p = std::make_pair(a,b);
    if(gMap.find(p) == gMap.end()) {
      gMap[p] = xt(i);
    } else {
      gMap[p] += xt(i);
    }
  }

  typedef Eigen::Triplet<double> T;
  std::vector<T> tripletList;

  int row = 0;
  int col = 0;
  for(std::map<std::pair<double, double>, double>::iterator it = gMap.begin(); it != gMap.end(); it++) {
    std::pair <double, double> p = it->first;
    tripletList.push_back(T(p.first, p.second, it->second));

    if(p.first > row) row = p.first;
    if(p.second > col) col = p.second;
  }
  xg.resize(row+1,col+1);
  xg.setFromTriplets(tripletList.begin(), tripletList.end());

}

// an ad-hoc implementation of MATLAB like setdiff
VectorXd PSN::setDiff(VectorXd &gNodes, RowVectorXd &vpi)
{
  std::set<double> vpiSet;
  std::vector<double> vfiVec;
  for(int i = 0; i < vpi.size(); i++) {
    vpiSet.insert(vpi(i));
  }
  for(int i = 0; i < gNodes.size(); i++) {
    double elem = gNodes(i);
    if(vpiSet.find(elem) != vpiSet.end()) continue;
    vfiVec.push_back(elem);
  }
  VectorXd vfi(vfiVec.size());
  for(int i = 0; i < vfi.size(); i++) {
    vfi(i) = vfiVec[i];
  }
  return vfi;
}

VectorXd PSN::find(VectorXd &tVec, double fi)
{
  std::vector<int> indexVec;
  for(int i = 0; i < tVec.size(); i++) {
    if(tVec(i) == fi) {
      indexVec.push_back(i);
    }
  }
  VectorXd iVec(indexVec.size());
  for(size_t i = 0; i < indexVec.size(); i++) {
    iVec(i) = indexVec[i];
  }
  return iVec;
}

VectorXd PSN::getSubMatrix(MatrixXd &mat, VectorXd &v, int col)
{
  VectorXd vec(v.size());
  for(int i = 0; i < v.size(); i++) {
    if(v(i) > mat.rows()) {
      std::cout << "Index exceeds matrix dimensions ..." << std::endl;
      assert(0);
    }
    vec(i) = mat(v(i), col);
  }
  return vec;
}

double PSN::sum(MatrixXd &mat, int row, VectorXd &v)
{
  double sum = 0.0;
  for(int i = 0; i < v.size(); i++) {
    sum += mat(row, v(i));
  }
  return sum;
}

double PSN::sum(RowVectorXd &vec1, VectorXd &vec2)
{
  double sum = 0.0;
  for(int i = 0; i < vec2.size(); i++) {
    sum += vec1(vec2(i));
  }
  return sum;
}

void PSN::distributeLoad(MatrixXd &cl, RowVectorXd &tile_nodes, double load)
{
  for(int i = 0; i < tile_nodes.size(); i++) {
    cl(tile_nodes(i)) = load;
  }
}
