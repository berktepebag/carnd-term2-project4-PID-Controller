#include "PID.h"
#include <iostream> 
#include <math.h>
#include <array>

using namespace std;

/*
* TODO: Complete the PID class.
*/

bool printInfo = false;
double diff_cte, prev_cte;

PID::PID() {

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	//cout << "************pid.Kp: "<<Kp << " pid.Ki: "<<Ki <<" pid.Kd: "<<Kd << "**********"<<endl;  
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;	
}

void PID::UpdateError(double cte) {	

	//cout << "prev_cte beginning: " << prev_cte << endl;

	diff_cte = cte - prev_cte;
	cte_i += cte;   
	prev_cte = cte;

	if(printInfo){
	cout << "*************************" << endl;
	cout << "prev_cte updated: " << prev_cte << endl;
	cout << "cte: " << cte << endl;
	cout << "diff_cte: " << diff_cte << endl;
	cout << "total_cte: " << cte_i << endl;
	cout << "*************************" << endl;}

	p_error_ = Kp_ * cte;
	i_error_ = Ki_ * cte_i;
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

double PID::SetThrottle(double cte, double speed)
{
	//*************************************************************//
    //*****Control the Kp,Ki,Kd and throttle according to CTE.****//
    double throttle = 0.3;

    //Slow down, car is not stable.
	if (fabs(cte) > 0.5 && speed > 15)
	{
		throttle = 0.1;
		Init(0.1,0.0001,0.8);
	}
    //Car is stable enough, increase speed.
	else if(fabs(cte) >= 0.25 && fabs(cte) < 0.5 && speed > 15){
		throttle = 0.3;
		Init(0.075,0.0001,0.8);
	}
    //Car is doing fine but bit slow, increase the speed bit more.
	else if(fabs(cte) < 0.25){
		throttle = 0.5;            
	}
    //Especially at low speeds, increase speed fastly.   
	else if(speed < 10){
		throttle = 3.0;
	}

    return throttle;

}
