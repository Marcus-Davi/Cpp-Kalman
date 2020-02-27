#include "Kalman.h"


namespace Kalman {

EKF::EKF(unsigned int n_states,unsigned int n_inputs, unsigned int n_outputs){
	
	this->n_states = n_states;
	this->n_inputs = n_inputs;
	this->n_states = n_outputs;
	
	Jf.resize(n_states,n_states);
	Jh.resize(n_outputs,n_states);
	Qn.resize(n_states,n_states);
	Rn.resize(n_outputs,n_outputs);
	Kk.resize(n_states,n_outputs);
//	Pk.resize(n_states,n_states);

	Xest.resize(n_states);
	Yest.resize(n_outputs);
	U.resize(n_inputs);
	Y.resize(n_outputs);
	
	I = MatrixXd::Identity(n_states,n_states);
	Pk = MatrixXd::Identity(n_states,n_states);


	std::cout << I << std::endl;


}

EKF::~EKF(){

}

void EKF::Predict(const float* Input){
   //ver forma melhor kk
	float *x;
	for(unsigned int k=0;k<n_inputs;++k){
	 U(k) = Input[k];
   }

	StateFun(Xest,U,&Xest);
	Jacobian_F(Xest,U,&Jf);


	std::cout << U << std::endl; //DEBUG
	
	std::cout << "Pk antes " <<  Pk << std::endl;

	Pk = Jf*Pk*Jf.transpose() + Qn;


	std::cout << "Pk depois : " << std::endl << Pk << std::endl;
}

















}




