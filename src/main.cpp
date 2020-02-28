#include <iostream>

#include "Kalman.h"
#define SystemTs 0.02f

void StateFunction(const VectorXd& Xk,const VectorXd& Uk,void* M_){
	//Uk -> entrada gyro 3x1

	VectorXd* M = static_cast<VectorXd*>(M_);

	//Quaternion qk(Xk[0],Xk[1],Xk[2],Xk[3]);
	//Quaternion qk_w(0,Uk[0],Uk[1],Uk[2]);
	//Quaternion qk1;
	//qk1 = qk*qk_w;
	//qk1 = qk1* (SystemTs/2);
	//qk1 = qk1 + qk;


(*M)(0) = 1;
(*M)(1) = 2;
(*M)(2) = 3;
(*M)(3) = 4;

}
static void StateJacobian(const VectorXd& Xk,const VectorXd& Uk,void* M_){
	// Uk -> entrada gyro 3x1
	// M-> 4x4
	MatrixXd* M = static_cast<MatrixXd*>(M_);

	(*M)(0,0) = 1.0f;
	(*M)(0,1) = -Uk(0)*SystemTs/2;
	(*M)(0,2) = -Uk[1]*SystemTs/2;
	(*M)(0,3) = -Uk[2]*SystemTs/2;

	(*M)(1,0) = Uk[0]*SystemTs/2;
	(*M)(1,1) = 1.0f;
	(*M)(1,2) = Uk[2]*SystemTs/2;
	(*M)(1,3) = -Uk[1]*SystemTs/2;

	(*M)(2,0) = Uk[1]*SystemTs/2;
	(*M)(2,1) = -Uk[2]*SystemTs/2;
	(*M)(2,2) = 1.0f;
	(*M)(2,3) = Uk[0]*SystemTs/2;

	(*M)(3,0) = Uk[2]*SystemTs/2;
	(*M)(3,1) = Uk[1]*SystemTs/2;
	(*M)(3,2) = -Uk[0]*SystemTs/2;
	(*M)(3,3) = 1.0f;

}
using namespace std;
using namespace Eigen;
int main(){

cout << "Teste Kalman" << endl;
	Kalman::EKF Filtro(4,3,6);

	double Qn[4*4] = {
			0.001, -0.0003, 0.0003, 0.0003,
			-0.0003,0.0001,-0.0001,-0.0001,
			0.0003,-0.0001,0.0001,0.0001,
			0.0003,-0.0001,0.0001,0.0001
	}; //3x3 n x n

	double Rn[6*6] = {
			0.1,0,0,0,0,0,
			0,0.1,0,0,0,0,
			0,0,0.1,0,0,0,
			0,0,0,2,0,0,
			0,0,0,0,2,0,
			0,0,0,0,0,3,
	}; //3x3 out x out

double saida[4];

  Filtro.SetQn(Qn);
	Filtro.SetRn(Rn);
	Filtro.SetStateJacobian(StateJacobian);
	Filtro.SetStateFunction(StateFunction);

	double input[3] = {5,6,7};

	Filtro.Predict(input);

	double output[6] = {1,2,3,4,5,6};

	Filtro.Update(output);

	//Filtro.GetEstimatedStates(saida);












}
