#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;
    prev_cte = p_error = d_error = i_error = 0.0;

    //Twiddle Tuning
    twiddle = false;
    dp = {0.1, 0.001, 0.1};
    n = 0;
    dp_i = 0;
    eval_window = 1000;
    total_error = 0;
    best_error = eval_window;
}

void PID::UpdateError(double cte) {

	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
	prev_cte = cte;
	
	if (twiddle && n % eval_window == 0) {
        total_error += pow(cte,2);
        UpdateParam(dp_i, dp[dp_i]); 
        // next parameter
        dp_i = (dp_i + 1) % 3;
		// cout << "Index: " << n << endl;
		cout << "Total Error: " << total_error << endl;
		cout << "Best Error: " << best_error << endl;

		if (total_error < best_error) {
            // cout << "improvement!" << endl;
            best_error = total_error;
            dp[dp_i] *= 1.1;
            
        }
        else {
            UpdateParam(dp_i, -2*dp[dp_i]);

            if (total_error < best_error) {
                best_error = total_error;
                dp[dp_i] *= 1.1;
            }
            else {
                UpdateParam(dp_i, dp[dp_i]);
                dp[dp_i] *= 0.9;
            }

        }

        total_error = 0;
        // cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }

    n++;

}

double PID::TotalError() {
	return p_error * Kp + i_error * Ki + d_error * Kd;
}

void PID::UpdateParam(int index, double amount) {
    if (index == 0) {
        Kp += amount;
    }
    else if (index == 1) {
        Ki += amount;
    }
    else if (index == 2) {
        Kd += amount;
    }
    else {
        std::cout << "UpdateParam: index out of bounds";
    }
}