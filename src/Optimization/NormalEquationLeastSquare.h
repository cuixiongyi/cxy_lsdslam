//
// Created by xiongyi on 4/7/16.
//

#ifndef CXY_LSDSLAM_NORMALEQUATIONLEASTSQUARE_H
#define CXY_LSDSLAM_NORMALEQUATIONLEASTSQUARE_H


#include <Eigen/Eigen>
#include "Optimization/OptimizedSelfAdjointMatrix66f.h"
#include "DataStructure/DataTypeDeclearation.h"

namespace cxy
{

class NormalEquationLeastSquare {


    public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    OptimizedSelfAdjointMatrix66f A_opt;
    Matrix66f A;
    Vector6f b;

        bool solved;
        float error;
        size_t maxnum_constraints, num_constraints;


        virtual void initialize(const unsigned int maxnum_constraints);
        virtual void update(const Vector6f& J, const float& res, const float& weight = 1.0f);
        virtual void finish();
        virtual void finishNoDivide();
        virtual void solve(Vector6f& x);
        virtual void setLambda(float lambda);

        void combine(const NormalEquationLeastSquare& other);

};
}


#endif //CXY_LSDSLAM_NORMALEQUATIONLEASTSQUARE_H
