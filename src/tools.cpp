#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() 
{  
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
