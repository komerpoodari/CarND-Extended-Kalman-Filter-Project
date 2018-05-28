#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() 
{
  /**
  * Komer debug variables
  */
  max_rmse = VectorXd(4);
  max_rmse << -1.0, -1.0, -1.0, -1.0;
  
  sum_rmse = VectorXd(4);
  sum_rmse << 0.0, 0.0, 0.0, 0.0;
  
  ave_rmse = VectorXd(4);
  ave_rmse << 0.0, 0.0, 0.0, 0.0;
  
  data_count = 0;
  
  
}

Tools::~Tools() {}




VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
	VectorXd rmse(4);
	rmse << 0.0,0.0,0.0,0.0;
	

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here

    int est_size = estimations.size();
    int grd_size = ground_truth.size();
    
    if ( (est_size == 0 ) || (est_size != grd_size))
    {
        cout << "Estimations Size: " << est_size <<"; Ground Truth Size: " << grd_size << endl;
        return rmse;
	}
	
	
	//accumulate squared residuals
	for(int i=0; i < est_size; ++i){
        // ... your code here
        VectorXd residual = ground_truth [i] - estimations [i];
            
        residual = residual.array() * residual.array();
        rmse += residual;
    }
	    
	//calculate mean
	// ... your code here
	rmse = rmse / est_size;

	//calculate the squared root
    rmse = rmse.array().sqrt();
    
    // komer debug variables update
    data_count++;
    sum_rmse += rmse;
    ave_rmse = sum_rmse / data_count;
        
    if (max_rmse(0) <= rmse(0))
        max_rmse(0) = rmse(0);

    if (max_rmse(1) <= rmse(1))
        max_rmse(1) = rmse(1);

    if (max_rmse(2) <= rmse(2))
        max_rmse(2) = rmse(2);

    if (max_rmse(3) <= rmse(3))
        max_rmse(3) = rmse(3);

    //print
    if (data_count >= 499) {
        cout << "data count: " << data_count << "; max_rmse: " << max_rmse << endl;
        cout << "data count: " << data_count << "; ave_rmse: " << ave_rmse << endl;
    }
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
    float px2_py2  =  px*px + py*py;
   
	if (px2_py2 < 0.0001)
	{
	    cout << "px2_py2: " << px2_py2 << endl ;
        px2_py2 = 0.0001;
	}
	
	float px2_py2_05 = sqrt(px2_py2);
    float px2_py2_15 = pow(px2_py2, 3.0/2.0);
   //compute the Jacobian matrix

	//row 0
	Hj(0,0) = px / px2_py2_05;
	Hj(0,1) = py / px2_py2_05;
	Hj(0,2) = 0.0;
	Hj(0,3) = 0.0;
	    
	//row 1
	Hj(1,0) = -py / px2_py2;
	Hj(1,1) = px / px2_py2;
	Hj(1,2) = 0.0;
	Hj(1,3) = 0.0;

	//row 2
	Hj(2,0) = (py * (vx*py - vy*px)) / px2_py2_15;
	Hj(2,1) = (px * (vy*px - vx*py)) / px2_py2_15;
	Hj(2,2) = Hj(0,0);
	Hj(2,3) = Hj(0,1);

	return Hj;
}
