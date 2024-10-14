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
MatrixXd LQRControl::calRicatti(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R) {
    MatrixXd Qf= Q;
    MatrixXd P = Qf;
    MatrixXd P_;
    for(int i=0;i<N;i++){
        P_ = Q+A.transpose()*P*A-A.transpose()*P*B*(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
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

void LQRControl::set_model(MatrixXd A, MatrixXd B, MatrixXd Q, MatrixXd R){
    this->A = A;
    this->B = B;
    this->Q = Q;
    this->R = R;
}

void P_update(){
    P = calRicatti(A,B,Q,R);
}

double LQRControl::lqrControl() {
    MatrixXd X(3,1);
    X<<now_state.v-target_state.v,now_state.theta-target_state.theta,now_state.w-target_state.w;
    MatrixXd K = -(R+B.transpose()*P*B).inverse()*B.transpose()*P*A;
    MatrixXd u = K*X; //[v-ref_v,delta-ref_delta,w-ref_w]

    return u(1,0);
}
