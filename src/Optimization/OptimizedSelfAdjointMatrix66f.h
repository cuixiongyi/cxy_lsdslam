//
// Created by xiongyi on 4/7/16.
//


#ifndef CXY_LSDSLAM_OPTIMIZEDSELFADJOINTMATRIX6X6F_H
#define CXY_LSDSLAM_OPTIMIZEDSELFADJOINTMATRIX6X6F_H

#include "DataStructure/DataTypeDeclearation.h"
#include <Eigen/Eigen>
#include <xmmintrin.h>


/// This is referenced from LSD-SLAM
namespace cxy
{

class OptimizedSelfAdjointMatrix66f
{
public:
    OptimizedSelfAdjointMatrix66f();


    inline void rankUpdate(const Vector6f& u, const float alpha)
    {
#if defined(ENABLE_NEON)

        const float* in_ptr = u.data();
        float* out_ptr = data;
        __asm__ __volatile__
        (
            // NOTE: may reduce count of used registers by calculating some value(s?) later.

            "vdup.32  q15, %[alpha]               \n\t" // alpha(q15)
            "vldmia   %[in_ptr], {q9-q10}         \n\t" // v1234(q9), v56xx(q10)
            "vldmia   %[out_ptr], {q0-q5}         \n\t"

            "vmov     d21, d20                    \n\t" // v5656(q10)
            "vmov     q11, q10                    \n\t"
            "vmov     q12, q10                    \n\t"
            "vzip.32  q11, q12                    \n\t" // v5566(q11)
            "vmul.f32 q11, q11, q15               \n\t" // alpha*v3344(q14)

            "vmov     q12, q9                     \n\t"
            "vmov     d19, d18                    \n\t" // v1212(q9)
            "vmov     d24, d25                    \n\t" // v3434(q12)

            "vmov     q13, q9                     \n\t"
            "vmov     q14, q9                     \n\t"
            "vzip.32  q13, q14                    \n\t" // v1122(q13)
            "vmul.f32 q13, q13, q15               \n\t" // alpha*v1122(q13)

            "vmov     q14, q12                    \n\t"
            "vmov     q8, q12                     \n\t"
            "vzip.32  q14, q8                     \n\t" // v3344(q14)
            "vmul.f32 q14, q14, q15               \n\t" // alpha*v3344(q14)

            "vmla.f32 q0, q13, q9                 \n\t"
            "vmla.f32 q1, q13, q12                \n\t"
            "vmla.f32 q2, q13, q10                \n\t"

            "vmla.f32 q3, q14, q12                \n\t"
            "vmla.f32 q4, q14, q10                \n\t"

            "vmla.f32 q5, q11, q10                \n\t"

            "vstmia %[out_ptr], {q0-q5}           \n\t"

        : /* outputs */
        : /* inputs  */ [alpha]"r"(alpha), [in_ptr]"r"(in_ptr), [out_ptr]"r"(out_ptr)
        : /* clobber */ "memory", "cc", // TODO: is cc necessary?
                        "q0", "q1", "q2", "q3", "q4", "q5", "q8", "q9", "q10", "q11", "q12", "q13", "q14"
        );

#else

        __m128 s = _mm_set1_ps(alpha);
        __m128 v1234 = _mm_loadu_ps(u.data());
        __m128 v56xx = _mm_loadu_ps(u.data() + 4);

        __m128 v1212 = _mm_movelh_ps(v1234, v1234);
        __m128 v3434 = _mm_movehl_ps(v1234, v1234);
        __m128 v5656 = _mm_movelh_ps(v56xx, v56xx);

        __m128 v1122 = _mm_mul_ps(s, _mm_unpacklo_ps(v1212, v1212));

        _mm_store_ps(data + 0, _mm_add_ps(_mm_load_ps(data + 0), _mm_mul_ps(v1122, v1212)));
        _mm_store_ps(data + 4, _mm_add_ps(_mm_load_ps(data + 4), _mm_mul_ps(v1122, v3434)));
        _mm_store_ps(data + 8, _mm_add_ps(_mm_load_ps(data + 8), _mm_mul_ps(v1122, v5656)));

        __m128 v3344 = _mm_mul_ps(s, _mm_unpacklo_ps(v3434, v3434));

        _mm_store_ps(data + 12, _mm_add_ps(_mm_load_ps(data + 12), _mm_mul_ps(v3344, v3434)));
        _mm_store_ps(data + 16, _mm_add_ps(_mm_load_ps(data + 16), _mm_mul_ps(v3344, v5656)));

        __m128 v5566 = _mm_mul_ps(s, _mm_unpacklo_ps(v5656, v5656));

        _mm_store_ps(data + 20, _mm_add_ps(_mm_load_ps(data + 20), _mm_mul_ps(v5566, v5656)));

#endif
    }

    void setZero();

    void toEigen(Matrix66f& m) const;


    inline void operator +=(const OptimizedSelfAdjointMatrix66f& other)
    {
#if defined(ENABLE_SSE)
        _mm_store_ps(data +  0, _mm_add_ps(_mm_load_ps(data +  0), _mm_load_ps(other.data +  0)));
      _mm_store_ps(data +  4, _mm_add_ps(_mm_load_ps(data +  4), _mm_load_ps(other.data +  4)));
      _mm_store_ps(data +  8, _mm_add_ps(_mm_load_ps(data +  8), _mm_load_ps(other.data +  8)));
      _mm_store_ps(data + 12, _mm_add_ps(_mm_load_ps(data + 12), _mm_load_ps(other.data + 12)));
      _mm_store_ps(data + 16, _mm_add_ps(_mm_load_ps(data + 16), _mm_load_ps(other.data + 16)));
      _mm_store_ps(data + 20, _mm_add_ps(_mm_load_ps(data + 20), _mm_load_ps(other.data + 20)));
#elif defined(ENABLE_NEON)
        const float* other_data_ptr = other.data;
        __asm__ __volatile__
        (
            // NOTE: The way of loading the data was benchmarked and this was the
            // fastest variant (faster than loading everything in order, and faster
            // than loading both blocks at once).
            "vldmia   %[other_data]!, {q0-q2}     \n\t"
            "vldmia   %[data], {q9-q14}           \n\t"
            "vldmia   %[other_data], {q3-q5}      \n\t"

            "vadd.f32 q0, q0, q9                  \n\t"
            "vadd.f32 q1, q1, q10                 \n\t"
            "vadd.f32 q2, q2, q11                 \n\t"
            "vadd.f32 q3, q3, q12                 \n\t"
            "vadd.f32 q4, q4, q13                 \n\t"
            "vadd.f32 q5, q5, q14                 \n\t"

            "vstmia %[data], {q0-q5}              \n\t"

        : /* outputs */ [other_data]"+&r"(other_data_ptr)
        : /* inputs  */ [data]"r"(data)
        : /* clobber */ "memory",
                        "q0", "q1", "q2", "q3", "q4", "q5", "q9", "q10", "q11", "q12", "q13", "q14"
        );
#else
        for(size_t idx = 0; idx < Size; idx++)
            data[idx] += other.data[idx];
#endif
    }
private:
    enum {
        Size = 24
    };
    EIGEN_ALIGN16 float data[Size];
};

}

#endif //CXY_LSDSLAM_OPTIMIZEDSELFADJOINTMATRIX6X6F_H
