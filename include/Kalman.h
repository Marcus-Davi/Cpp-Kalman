#ifndef KALMAN_H
#define KALMAN_H
#include "Eigen/Dense"
#include <iostream> //deebug


using namespace Eigen;


typedef void (*KalmanFunctionPt)(const VectorXd& Xk,const VectorXd& Uk,void* Res); // M é entrada,saída ou uma matriz
//classe para implementação do filtro de kalman estendido
namespace Kalman {

	
class EKF {
	public:
		EKF(unsigned int n_states,unsigned int n_inputs, unsigned int n_outputs);


		~EKF();
		
		//Ponteiros para functions
		inline void SetStateFunction(KalmanFunctionPt F) {
		StateFun = F;
		}
		inline void  SetStateJacobian(KalmanFunctionPt F){
		Jacobian_F = F;
		}
		inline void  SetMeasurementJacobian(KalmanFunctionPt F){
		Jacobian_H = F;
		}
		inline void SetMeasurementFunction(KalmanFunctionPt F){
		MeasureFun = F;
		}

		void Predict(const float* input); //TODO fazer template ?
		void Update(const float* output);



	private:

	unsigned int n_states;
	unsigned int n_outputs;
	unsigned int n_inputs;

	KalmanFunctionPt StateFun;
	KalmanFunctionPt MeasureFun;
	KalmanFunctionPt Jacobian_F;
	KalmanFunctionPt Jacobian_H;
	
	MatrixXd Jf; //Jacobian F
	MatrixXd Jh; // Jacobian H

	MatrixXd Qn,Rn; //Covariancias
	MatrixXd Kk,Pk; //Ganho de Kalman e Covariancia erro

	VectorXd Xest,Y,Yest,U; //x estimado, saida Y, Y estimado, entrada U

	MatrixXd I; // Identidade


};


}


#endif
