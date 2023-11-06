/**
(C) Copyright 2022-2023 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
    - Responsible for the original implementation.
- Quentin Lin (qlin1806@g.ecc.u-tokyo.ac.jp)
    - Added support for equality constraints
*/
#pragma once

#include <vector>

#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

namespace DQ_robotics
{
    class DQ_QPOASESSolver: public DQ_QuadraticProgrammingSolver
    {
    protected:
        bool qpoases_solve_first_time_;
        SQProblem qpoases_problem_;
        //The integer argument nWSR specifies the maximum number of working set recalculations to be performed during the initial homotopy (on output it contains the number
        //of working set recalculations actually performed!)
        //Page 14 of https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
        int_t maximum_working_set_recalculations_;

        //implimentation of the equality constraints is done via appending it and recipical of it to the inequalities
        double_t equality_constraints_tolerance_ = DQ_threshold;

        //Overload this method in a child class to change the configuration.
        virtual void _config_solver()
        {
            Options options;
            options.printLevel = qpOASES::PrintLevel::PL_NONE;
            qpoases_problem_.setOptions( options );
            set_maximum_working_set_recalculations(500);
        }

        //https://github.com/SmartArmStack/sas_conversions/blob/master/src/eigen3_std_conversions.cpp
        //A copy from sas
        std::vector<double> _vectorxd_to_std_vector_double(const VectorXd& vectorxd)
        {
            std::vector<double> vec(vectorxd.data(), vectorxd.data() + vectorxd.rows() * vectorxd.cols());
            return vec;
        }

        //Another copy from sas
        VectorXd _std_vector_double_to_vectorxd(std::vector<double> std_vector_double)
        {
            double* ptr = &std_vector_double[0];
            Eigen::Map<Eigen::VectorXd> vec(ptr,std_vector_double.size());
            return vec;
        }


    public:
        DQ_QPOASESSolver():
        qpoases_solve_first_time_(true)
        {
        }
        ~DQ_QPOASESSolver()=default;

        void set_maximum_working_set_recalculations(const int& maximum_working_set_recalculations)
        {
            maximum_working_set_recalculations_ = maximum_working_set_recalculations;
        }

        /**
         * @brief
         *  Sets the tolerance for the equality constraints.
         * @param equality_constraints_tolerance the tolerance for the equality constraints.
         */
        void set_equality_constraints_tolerance(const double &equality_constraints_tolerance) {
            equality_constraints_tolerance_ = equality_constraints_tolerance;
        }

        /**
         * @brief
         *  Gets the tolerance for the equality constraints.
         * @return the tolerance for the equality constraints.
         */
        double get_equality_constraints_tolerance() {
            return equality_constraints_tolerance_;
        }

        /**
         * @brief
         *   Solves the following quadratic program
         *   min(x)  0.5*x'Hx + f'x
         *   s.t.    Ax <= b
         *           Aeqx = beq.
         * Method signature is compatible with MATLAB's 'quadprog'.
         * @param H the n x n matrix of the quadratic coeficitients of the decision variables.
         * @param f the n x 1 vector of the linear coeficients of the decision variables.
         * @param A the m x n matrix of inequality constraints.
         * @param b the m x 1 value for the inequality constraints.
         * @param Aeq the m x n matrix of equality constraints.
         * @param beq the m x 1 value for the inequality constraints.
         * @return the optimal x
         */
        VectorXd solve_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& A, const VectorXd& b, const MatrixXd& Aeq, const VectorXd& beq) override
        {
            const int PROBLEM_SIZE = H.rows();
            const int INEQUALITY_CONSTRAINT_SIZE = b.size();
            const int EQUALITY_CONSTRAINT_SIZE = beq.size();

            ///Check sizes
            //Objective function
            if(H.rows()!=H.cols())
                throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): H must be symmetric. H.rows()="+std::to_string(H.rows())+" but H.cols()="+std::to_string(H.cols())+".");
            if(f.size()!=H.rows())
                throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): f must be compatible with H. H.rows()=H.cols()="+std::to_string(H.rows())+" but f.size()="+std::to_string(f.size())+".");

            //Inequality constraints
            if(b.size()!=A.rows())
                throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): size of b="+std::to_string(b.size())+" should be compatible with rows of A="+std::to_string(A.rows())+".");

            //Equality constraints
            if(beq.size()!=Aeq.rows())
                throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): size of beq="+std::to_string(beq.size())+" should be compatible with rows of Aeq="+std::to_string(Aeq.rows())+".");

            //Append equality constraints to inequality constraints
            auto A_extended = A;
            auto ub_extended = b;
            if(EQUALITY_CONSTRAINT_SIZE!=0 && INEQUALITY_CONSTRAINT_SIZE!=0)
            {
                A_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE*2, PROBLEM_SIZE);
                A_extended << A, Aeq, -Aeq;
                ub_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE*2);
                ub_extended << b, beq + VectorXd::Constant(EQUALITY_CONSTRAINT_SIZE, equality_constraints_tolerance_),
                               -beq + VectorXd::Constant(EQUALITY_CONSTRAINT_SIZE, equality_constraints_tolerance_);
            } else if(EQUALITY_CONSTRAINT_SIZE!=0)
            {
                A_extended.resize(EQUALITY_CONSTRAINT_SIZE*2, PROBLEM_SIZE);
                A_extended << Aeq, -Aeq;
                ub_extended.resize(EQUALITY_CONSTRAINT_SIZE*2);
                ub_extended << beq + VectorXd::Constant(EQUALITY_CONSTRAINT_SIZE, equality_constraints_tolerance_),
                               -beq + VectorXd::Constant(EQUALITY_CONSTRAINT_SIZE, equality_constraints_tolerance_);
            }

            std::vector<double> H_std_vec(H.data(), H.data() + H.rows() * H.cols());
            real_t* H_vec = &H_std_vec[0];

            MatrixXd AT = A_extended.transpose();
            std::vector<double> A_std_vec(AT.data(), AT.data() + AT.rows() * AT.cols());
            real_t* A_vec = &A_std_vec[0];

            auto g_std_vec = _vectorxd_to_std_vector_double(f);
            real_t* g_vec = &g_std_vec[0];

            auto ub_std_vec = _vectorxd_to_std_vector_double(ub_extended);
            real_t *ubA_vec = &ub_std_vec[0];

            if(qpoases_solve_first_time_)
            {
                qpoases_problem_ = SQProblem(PROBLEM_SIZE, INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE*2, HST_POSDEF);
                _config_solver();
                auto maximum_working_set_recalculations_local = maximum_working_set_recalculations_; //qpOASES changes the value, so we make a local copy
                auto problem_init_return = qpoases_problem_.init( H_vec,g_vec,A_vec,NULL,NULL,NULL,ubA_vec, maximum_working_set_recalculations_local );
                if(problem_init_return != SUCCESSFUL_RETURN)
                    throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): Unable to solve quadratic program.");
                qpoases_solve_first_time_ = false;
            }
            else
            {
                auto maximum_working_set_recalculations_local = maximum_working_set_recalculations_; //qpOASES changes the value, so we make a local copy
                auto problem_init_return = qpoases_problem_.hotstart(H_vec,g_vec,A_vec,NULL,NULL,NULL,ubA_vec, maximum_working_set_recalculations_local );
                if(problem_init_return != SUCCESSFUL_RETURN)
                    throw std::runtime_error("DQ_QPOASESSolver::solve_quadratic_program(): Unable to solve quadratic program.");
            }

            real_t xOpt[PROBLEM_SIZE];
            qpoases_problem_.getPrimalSolution( xOpt );

            std::vector<double> return_value_std(xOpt, xOpt + PROBLEM_SIZE);

            return _std_vector_double_to_vectorxd(return_value_std);
        }

    };

}