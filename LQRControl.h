#define EPS 1.0e-4
#include <ArduinoEigen.h>
#include <vector>
#include <iostream>
using namespace std;
using namespace Eigen;

struct State{
        double v;
        double theta;
        double w;
    };

class LQRControl {
private:
    int N;//迭代精度
    MatrixXd A,B,Q,R,P;
    State now_state;
    State target_state;

public:
    LQRControl(int n) : N(n) {};
    LQRControl(int n, MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) : N(n), A(A), B(B), Q(Q), R(R) {};
    void set_tager_state(State target_state);
    void set_now_state(State now_state);
    void set_model(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    MatrixXd calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R);
    void P_update();
    double lqrControl();
};


#endif //CHHROBOTICS_CPP_LQRCONTROL_H
