#include "PID.h"
#include <iostream> 
#include <math.h>
#include <array>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	//cout << "************pid.Kp: "<<Kp << " pid.Ki: "<<Ki <<" pid.Kd: "<<Kd << "**********"<<endl;  
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;
}

bool printInfo = false;
double diff_cte, prev_cte;

void PID::UpdateError(double cte) {

	double total_cte = 0;

	//cout << "prev_cte beginning: " << prev_cte << endl;

	diff_cte = cte - prev_cte;
	total_cte += cte;   
	prev_cte = cte;

	if(printInfo){
	cout << "*************************" << endl;
	cout << "prev_cte updated: " << prev_cte << endl;
	cout << "cte: " << cte << endl;
	cout << "diff_cte: " << diff_cte << endl;
	cout << "total_cte: " << total_cte << endl;
	cout << "*************************" << endl;}

	p_error_ = Kp_ * cte;
	i_error_ = Ki_ * total_cte;
	d_error_ = Kd_ * diff_cte;	

	if(printInfo){
	cout << "*************************" << endl;
	cout << "p_error_: " << p_error_ << endl;
	cout << "i_error_: " << i_error_ << endl;
	cout << "d_error_: " << d_error_ << endl;
	cout << "*************************" << endl;}
}

double PID::TotalError() {

	double total_error = - p_error_ - i_error_ - d_error_;
	//total_error = fmod(total_error,1.0);

	return total_error;
}

