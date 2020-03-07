#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>
#include "ooqp_eigen_interface/OoqpEigenInterface.hpp"
#include "ooqp_eigen_interface/ooqpei_gtest_eigen.hpp"
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <iostream>

using namespace std;

class TrajectoryGeneratorWaypoint {
    private:
        double _qp_cost;
        Eigen::MatrixXd _Q;
        Eigen::VectorXd _Px, _Py, _Pz;
    public:
        TrajectoryGeneratorWaypoint();

        ~TrajectoryGeneratorWaypoint();

        Eigen::MatrixXd PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);

        Eigen::MatrixXd PolyCloseFormGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time); 

        int Factorial(int x);
};

class MinimumSnapSolver {
    private:
        int n_dim_;
        // order information
        int d_order_;
        int n_order_;
        // 
        int n_poly_perseg_;
        int n_poly_all_;
        int n_equals_;
        int n_seg_;

        // some input Matrixs
        Eigen::VectorXd ts_;
        Eigen::MatrixXd waypoints_;
        Eigen::MatrixXd derivative_cons_;

        // some Matrixs for formulation
        Eigen::SparseMatrix<double, Eigen::RowMajor> Q_;
        Eigen::SparseMatrix<double, Eigen::RowMajor> A_;
        Eigen::MatrixXd b_;

        // coef matirx result
        Eigen::MatrixXd poly_coef_;

        bool solve_ = true;

    public:
        MinimumSnapSolver(const int d_order,
                          const Eigen::MatrixXd &waypoints, 
                          const Eigen::MatrixXd &derivative_cons,
                          const Eigen::VectorXd &ts) : d_order_(d_order), ts_(ts), waypoints_(waypoints), derivative_cons_(derivative_cons)
        {
            n_dim_ = waypoints_.row(0).size();
            n_order_ = 2*d_order_ - 1;
            n_seg_ = ts_.size();

            n_poly_perseg_ = n_order_ + 1;
            n_poly_all_ = n_seg_ * n_poly_perseg_;
            n_equals_ = 2*d_order_ + (n_seg_-1) * (d_order_+1);

            Q_.resize(n_seg_ * n_poly_perseg_, n_seg_ * n_poly_perseg_);
            A_.resize(n_equals_, n_poly_all_);
            b_ = Eigen::MatrixXd::Zero(n_equals_, n_dim_);
            poly_coef_ = Eigen::MatrixXd::Zero(n_poly_all_, n_dim_);

        }
        ~MinimumSnapSolver(){}
        void test()
        {
            cout << "in tets function" << endl;
        }

        void getQMatrix()
        {
            for(int k = 0; k < n_seg_; k++) {
                int start_idx = k*n_poly_perseg_;
                for (int i = d_order_; i < n_poly_perseg_; i++) {
                    for (int j = i; j < n_poly_perseg_; j++) {
                        Q_.insert(start_idx+i, start_idx+j) = Factorial(i)/Factorial(i-d_order_) \
                                                              *Factorial(j)/Factorial(j-d_order_) \
                                                              *pow(ts_(k), i+j-n_order_)/(i+j-n_order_);
                        if(i != j) 
                            Q_.insert(start_idx+j, start_idx+i) = Q_.coeff(start_idx+i, start_idx+j);
                    }
                }
            }
        }

        void getAbeq()
        {
            int start_idx_1, start_idx_2;
            // start derivative constraints
            for (int k = 0; k < d_order_; k++) {
                A_.insert(k, k) = Factorial(k);
            }
            // end derivative constriants
            start_idx_1 = d_order_;
            start_idx_2 = (n_seg_-1)*n_poly_perseg_;
            for (int k = 0; k < d_order_; k++) {
                for (int i = k; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+k, start_idx_2+i) =  Factorial(i)/Factorial(i-k)*pow(ts_(n_seg_-1), i-k);
                }
            }
            //position constraints in middle points
            start_idx_1 = 2*d_order_;
            for (int j = 0; j < n_seg_-1; j++) {
                start_idx_2 = n_poly_perseg_ * (j+1);
                A_.insert(start_idx_1+j, start_idx_2) = 1;
            }

            //continuity constraints in middle points
            for (int k = 0; k < n_dim_; k++) {
                for (int j = 0; j < n_seg_-1; j++) {
                    for (int i = k; i < n_poly_perseg_; i++) {
                        start_idx_1 = 2*d_order_+(n_seg_-1)*(k+1);
                        start_idx_2 = j*n_poly_perseg_;
                        A_.insert(start_idx_1+j, start_idx_2+i) = Factorial(i)/Factorial(i-k)*pow(ts_(j), i-k);
                        // A_.insert(start_idx_1+j，start_idx_2+n_poly_perseg_+i) = -Factorial(i)/Factorial(i-k)*pow(0, i-k);
                        if (i == k) {
                            A_.insert(start_idx_1+j, start_idx_2+n_poly_perseg_+i) = -Factorial(i);
                        }
                    }
                }
            }

            for (int k = 0; k < n_dim_; k++) {
                for (int i = 0; i < 2*d_order_; i++) {
                    b_(i, k) = derivative_cons_(i, k);
                }
                start_idx_1 = 2*d_order_;
                for (int j = 0; j < n_seg_-1; j++) {
                    b_(start_idx_1+j, k) = waypoints_(j+1, k);
                }
            }
        }

        Eigen::MatrixXd QPSolver()
        {
            Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(n_seg_, n_poly_perseg_*n_dim_);
            ros::Time time_start = ros::Time::now();

            // STEP 1: get Q Matrix
            getQMatrix();

            // STEP 2: get Aeq and beq
            getAbeq();

            // STEP 3: solve qp 
            Eigen::VectorXd c = Eigen::VectorXd::Zero(n_poly_all_);
            Eigen::VectorXd l = Eigen::VectorXd::Constant(n_poly_all_, std::numeric_limits<double>::min());
            Eigen::VectorXd u = Eigen::VectorXd::Constant(n_poly_all_, std::numeric_limits<double>::max());
            int i = 0;
            Eigen::VectorXd x;
            while(!solve_ && i < n_dim_) {
                if (ooqpei::OoqpEigenInterface::solve(Q_, c, A_, b_.col(i), l, u, x)) {
                    poly_coef_.col(i) = x;
                }
                else {
                    solve_ = false;
                }
            }

            ros::Time time_end = ros::Time::now();

            // STEP 4: if QP solve success, then return result
            if (solve_) {
                ROS_WARN("OOQP solution success! Total time taken is %f ms.", (time_end - time_start).toSec()*1000.0);
                for(int i = 0; i < n_seg_; i++) {
                    for (int j = 0; j < n_dim_; j++) {
                        for (int k = 0; k < n_poly_perseg_; k++) {
                            PolyCoeff(i, j*n_poly_perseg_+k) = poly_coef_(i*n_poly_perseg_+k, j);
                        }
                    }
                }
            }
            else {
                ROS_WARN("OOQP solution failed!");
            }
            return PolyCoeff;
        }

        // Eigen::MatrixXd CloseFormSolver()
        // {
        //     // Eigen::MatrixXd PolyCoeff = Eigen::MatrixXd::Zero(n_seg_, n_poly_perseg_*n_dim_);
        //     // ros::Time time_start = ros::Time::now();

        //     // STEP 1: get Q and M Matrix
            
        //     // STEP 2: get Ct Matrix    
            
               
        // }
                   
        int Factorial(int x)
        {
            int fac = 1;
            for(int i = x; i > 0; i--)
                fac = fac * i;
            return fac;
        }
};     

#endif
