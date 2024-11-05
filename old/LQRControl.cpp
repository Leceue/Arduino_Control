//
// Created by chh3213 on 2022/11/25.
//

#include "LQRControl.h"


/**
 * 解代数里卡提方程
 * @param A 状态矩阵A
 * @param B 状态矩阵B
 * @param Q Q为半正定的状态加权矩阵, 通常取为对角阵；Q矩阵元素变大意味着希望跟踪偏差能够快速趋近于零；
 * @param R R为正定的控制加权矩阵，R矩阵元素变大意味着希望控制输入能够尽可能小。
 * @return
 */
MatrixXd LQRControl::calRicatti(Matrix<3,3> A, Matrix<3,1> B, Matrix<3,3> Q, Matrix<1,1> R) {
    Matrix<3,3> Qf= Q;
    Matrix<3,3> P = Qf;
    Matrix<3,3> P_;
    for(int i=0;i<N;i++){
        P_ = Q+(~A)*P*A-(~A)*P*B*inverse(R+(~B)*P*B)*(~B)*P*A;
        //小于预设精度时返回
        if((P_-P).maxCoeff()<EPS&&(P-P_).maxCoeff()<EPS)break;
        //if((P_-P).cwiseAbs().maxCoeff()<EPS)break;

        P = P_;

    }
    return P_;
}

void LQRControl::set_tager_state(State target_state) {
    this->target_state = target_state;
}

void LQRControl::set_now_state(State now_state) {
    this->now_state = now_state;
}

void LQRControl::set_model(Matrix<3,3> A, Matrix<3,1> B, Matrix<3,3> Q, Matrix<1,1> R){
    this->A = A;
    this->B = B;
    this->Q = Q;
    this->R = R;
}

void LQRControl::P_update(){
    P = calRicatti(A,B,Q,R);
}

double LQRControl::lqrControl() {
    Matrix<3,1> X;
    X={now_state.v-target_state.v,now_state.theta-target_state.theta,now_state.w-target_state.w}
    Matrix<1,3> K = -inverse(R+(~B)*P*B))*(~B)*P*A;
    Matrix<1,1> u = K*X; //[v-ref_v,delta-ref_delta,w-ref_w]

    return u(1);
}
