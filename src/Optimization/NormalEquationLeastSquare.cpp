//
// Created by xiongyi on 4/7/16.
//

#include "NormalEquationLeastSquare.h"

namespace cxy
{


    void NormalEquationLeastSquare::initialize(const unsigned int maxnum_constraints)
    {
        A.setZero();
        A_opt.setZero();
        b.setZero();
        solved = false;
        error = 0;
        this->num_constraints = 0;
        this->maxnum_constraints = maxnum_constraints;
    }

    inline void NormalEquationLeastSquare::update(const Vector6f& J, const float& res, const float& weight)
    {
//	printf("up: %f, %f, %f, %f, %f, %f; res: %f; w: %f\n",
//			J[0],J[1],J[2],J[3],J[4],J[5],res, weight);

        A_opt.rankUpdate(J, weight);
        //MathSse<Sse::Enabled, float>::addOuterProduct(A, J, factor);
        //A += J * J.transpose() * factor;
        //MathSse<Sse::Enabled, float>::add(b, J, -res * factor); // not much difference :(
        b -= J * (res * weight);

        error += res * res * weight;
        num_constraints += 1;
    }

    void NormalEquationLeastSquare::combine(const NormalEquationLeastSquare& other)
    {
        A_opt += other.A_opt;
        b += other.b;
        error += other.error;
        num_constraints += other.num_constraints;
    }

    void NormalEquationLeastSquare::finish()
    {
        A_opt.toEigen(A);
        A /= (float) num_constraints;
        b /= (float) num_constraints;
        error /= (float) num_constraints;
    }
    void NormalEquationLeastSquare::finishNoDivide()
    {
        A_opt.toEigen(A);
    }

    void NormalEquationLeastSquare::solve(Vector6f& x)
    {
        x = A.ldlt().solve(b);
        solved = true;
    }

    void NormalEquationLeastSquare::setLambda(float lambda)
    {
        for(int i=0;i<6;i++)
            A(i,i) *= 1+lambda;

    }


}