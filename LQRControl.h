#define EPS 1.0e-4
// #include <ArduinoEigen.h>
#include <BasicLinearAlgebra.h>
#include <vector>
#include <iostream>
using namespace std;
using namespace BLA;

struct State{
        double v;
        double theta;
        double w;
    };

class LQRControl {
private:
    int N;//迭代精度
    Matrix<3,3> A;
    Matrix<3,1> B;
    Matrix<3,3> Q;
    Matrix<1,1> R;
    Matrix<3,3> P;
    State now_state;
    State target_state;

public:
    LQRControl(int n) : N(n) {};
    LQRControl(int n, Matrix<3,3> A, Matrix<3,1> B, Matrix<3,3> Q, Matrix<1,1> R) : N(n), A(A), B(B), Q(Q), R(R) {};
    void set_tager_state(State target_state);
    void set_now_state(State now_state);
    void set_model(Matrix<3,3> A, Matrix<3,1> B, Matrix<3,3> Q, Matrix<1,1> R;
    Matrix<3,3> calRicatti(Matrix<3,3> A, Matrix<3,1> B, Matrix<3,3> Q, Matrix<1,1> R);
    void P_update();
    double lqrControl();
};


#endif //CHHROBOTICS_CPP_LQRCONTROL_H
