#include <iostream>
#include "tools.h"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the inputs
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals

	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];
		//cout<< ground_truth[i]<<endl;
		//square the delta
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

void Tools::writeNIStoFile(string filepath, double time, double NIS){
	  std::ofstream file;
      file.open(filepath, std::ios::out | std::ios::app);
      std::string data_to_write = std::to_string(time) + "," + std::to_string(NIS) + "\n";
      file << data_to_write;
}