#ifndef LQR_CONTROL_H
#define LQR_CONTROL_H

#include <BasicLinearAlgebra.h>

using namespace BLA;

struct State {
    float v;      // velocity
    float theta;  // angle
    float w;      // angular velocity
};

class LQRControl {
private:
    // 系统矩阵
    BLA::Matrix<3,3> A;  // 状态转移矩阵
    BLA::Matrix<3,1> B;  // 输入矩阵
    BLA::Matrix<3,3> Q;  // 状态代价矩阵
    BLA::Matrix<1,1> R;  // 控制代价矩阵
    BLA::Matrix<3,3> P;  // Riccati方程的解

    State target_state;
    State now_state;
    
    const int N;        // 最大迭代次数
    const float EPS = 1e-6; // 收敛阈值

public:
    LQRControl(int max_iter = 1000) : N(max_iter) {
        // 初始化矩阵
        A.Fill(0);
        B.Fill(0);
        Q.Fill(0);
        R(0,0) = 1;
        P.Fill(0);
        
        // 初始化状态
        target_state = {0, 0, 0};
        now_state = {0, 0, 0};
    }
    
    /**
     * 解代数里卡提方程
     */
    BLA::Matrix<3,3> calRicatti(const BLA::Matrix<3,3>& A, 
                               const BLA::Matrix<3,1>& B,
                               const BLA::Matrix<3,3>& Q, 
                               const BLA::Matrix<1,1>& R) {
        BLA::Matrix<3,3> P = Q;
        BLA::Matrix<3,3> P_prev;
        
        for(int i = 0; i < N; i++) {
            P_prev = P;
            
            // 计算 B^T * P * B
            BLA::Matrix<1,1> BPB = ~B * P * B;
            
            // 计算 (R + B^T * P * B)^(-1)
            BLA::Matrix<1,1> inv_term = R + BPB;
            Invert(inv_term);
            
            // 更新P矩阵
            P = Q + (~A * P * A) - (~A * P * B * inv_term * ~B * P * A);
            
            // 检查收敛性
            float diff = 0;
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    diff = max(diff, abs(P(j,k) - P_prev(j,k)));
                }
            }
            if(diff < EPS) break;
        }
        
        return P;
    }
    
    /**
     * LQR控制计算
     */
    float lqrControl() {
        // 状态误差向量
        BLA::Matrix<3,1> X = {
            now_state.v - target_state.v,
            now_state.theta - target_state.theta,
            now_state.w - target_state.w
        };
        
        // 计算反馈增益K
        BLA::Matrix<1,1> temp = R + ~B * P * B;
        Invert(temp);
        BLA::Matrix<1,3> K = -temp * ~B * P * A;
        
        // 计算控制输入
        BLA::Matrix<1,1> u = K * X;
        
        return u(0,0);
    }
    
    // Setters
    void set_target_state(const State& state) {
        target_state = state;
    }
    
    void set_now_state(const State& state) {
        now_state = state;
    }
    
    void set_model(BLA::Matrix<3,3>& A_in, 
                  BLA::Matrix<3,1>& B_in,
                  BLA::Matrix<3,3>& Q_in, 
                  BLA::Matrix<1,1>& R_in) {
        A = A_in;
        B = B_in;
        Q = Q_in;
        R = R_in;
        P = calRicatti(A, B, Q, R);
    }
};

#endif