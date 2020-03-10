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

// #include "Array.hh"
// #include "QuadProg++.hh"
// #include <stdio.h>


using namespace std;


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
        Eigen::VectorXd b_;

        // coef matirx result
        Eigen::MatrixXd poly_coef_;

        bool solve_ = true;

    public:
        MinimumSnapSolver(const int d_order,
                          const Eigen::MatrixXd &waypoints, 
                          const Eigen::MatrixXd &derivative_cons,
                          const Eigen::VectorXd &ts) : d_order_(d_order), ts_(ts), waypoints_(waypoints), derivative_cons_(derivative_cons)
        {
            cout<<"waypoints:"<<endl<<waypoints_<<endl;
            n_dim_ = 1;
            cout << "dim: "<<n_dim_<<endl;
            n_order_ = 5;
            cout << "poly order: "<<n_order_<<endl; 
            n_seg_ = ts_.size();
            cout<<"n_seg: "<<n_seg_<<endl;
            cout<<"ts: "<<ts_.transpose()<<endl;

            n_poly_perseg_ = n_order_ + 1;
            n_poly_all_ = n_seg_ * n_poly_perseg_;
            n_equals_ = 2*d_order_ + (n_seg_-1) * (d_order_+1);

            Q_.resize(n_seg_ * n_poly_perseg_, n_seg_ * n_poly_perseg_);
            A_.resize(n_equals_, n_poly_all_);
            b_ = Eigen::VectorXd::Zero(n_equals_, 1);
            poly_coef_ = Eigen::MatrixXd::Zero(n_poly_all_, 1);

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
                for(int i = 3; i <= n_order_; i++){
                    for(int l = 3; l <= n_order_; l++){
                        Q_.insert(start_idx+i, start_idx+l) = (i)*(i-1)*(i-2)*l*(l-1)*(l-2)/(i+l-5) * pow(ts_(k),(i+l-5));
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

            for (int i = 0; i < 2*d_order_; i++) {
                b_(i) = derivative_cons_(i);
            }


            //position constraints in middle points
            start_idx_1 = 2*d_order_;
            for (int j = 0; j < n_seg_-1; j++) {
                start_idx_2 = n_poly_perseg_ * (j);
                for (int i = 0; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+i) = pow(ts_(j),i);
                }
            }

            for (int j = 0; j < n_seg_-1; j++) {
                b_(start_idx_1+j) = waypoints_(j+1);
            }



            //continuity constraints in middle points
            start_idx_1 = 2*d_order_+(n_seg_-1);
            for (int j = 0; j < n_seg_-1; j++) {
                start_idx_2 = j*n_poly_perseg_;
                for (int i = 0; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+i) = pow(ts_(j),i);
                }
                for (int i = 0; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+n_poly_perseg_+i) = -pow(0,i);
                }
            }       


            start_idx_1 = 2*d_order_+(n_seg_-1)*2;
            for (int j = 0; j < n_seg_-1; j++) {
                start_idx_2 = j*n_poly_perseg_;
                for (int i = 1; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+i) = i * pow(ts_(j),i-1);
                }
                for (int i = 1; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+n_poly_perseg_+i) = -i * pow(0,i);
                }
            }    


            start_idx_1 = 2*d_order_+(n_seg_-1)*3;
            for (int j = 0; j < n_seg_-1; j++) {
                start_idx_2 = j*n_poly_perseg_;
                for (int i = 2; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+i) = i*(i-1) * pow(ts_(j),i-2);
                }
                for (int i = 2; i < n_poly_perseg_; i++) {
                    A_.insert(start_idx_1+j, start_idx_2+n_poly_perseg_+i) = -i*(i-1) * pow(0,i-2);
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
            bool solveres;
            cout<<"Q:"<<endl<<Q_<<endl;
            cout<<"A:"<<endl<<A_<<endl;
            cout<<"b:"<<endl<<b_<<endl;

            cout<<"BEGIN"<<endl;

            // quadprogpp::Matrix<double>G(Q_.rows(),Q_.cols());
            // quadprogpp::Matrix<double>CE(A_.cols(), A_.rows());
            // quadprogpp::Vector<double>ce(b_.rows());

            // quadprogpp::Vector<double>g(0.0, b_.rows());
            // quadprogpp::Matrix<double>CI(0.0, A_.cols(), 1);
            // quadprogpp::Vector<double>ci(0.0, 1);
            // quadprogpp::Vector<double>xx(b_.rows());

            // for(int i = 0; i < Q_.rows(); i++){
            //     for(int j = 0; j < Q_.cols(); j++){
            //         G[i][j] = Q_.coeffRef(i,j);
            //     }
            // }
            // for(int i = 0; i < A_.rows(); i++){
            //     for(int j = 0; j < A_.cols(); j++){
            //         CE[j][i] = A_.coeffRef(i,j);
            //     }
            // }
            // for(int i = 0; i < b_.rows(); i++){
            //     ce[i] = -b_(i);
            // }
            // quadprogpp::solve_quadprog(G, g, CE, ce, CI, ci, xx);
            // cout<<"!!!!!!!!quadprogpp:"<<endl;
            // for(int i = 0; i < b_.rows(); i++){
            //     cout<<xx[i]<<", ";
            // }
            // cout<<endl;


            // ooqpei::OoqpEigenInterface OOQPsolver;

            // if(OOQPsolver.solve(Q_, c, A_, b_, l, u, x, true)) {
                
            //     //poly_coef_.col(i) = x;
            //     cout<<"mainXXXXXXXXX:"<<endl<<x<<endl;
            // }

            //ros::Time time_end = ros::Time::now();

            // STEP 4: if QP solve success, then return result
            // if (solve_) {
            //     ROS_WARN("OOQP solution success! Total time taken is %f ms.", (time_end - time_start).toSec()*1000.0);
            //     for(int i = 0; i < n_seg_; i++) {
            //         for (int j = 0; j < n_dim_; j++) {
            //             for (int k = 0; k < n_poly_perseg_; k++) {
            //                 PolyCoeff(i, j*n_poly_perseg_+k) = poly_coef_(i*n_poly_perseg_+k, j);
            //             }
            //         }
            //     }
            // }
            // else {
            //     ROS_WARN("OOQP solution failed!");
            // }
            cout<<"polyCoeff:"<<endl<<PolyCoeff<<endl;
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