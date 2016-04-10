//
// Created by xiongyi on 4/7/16.
//

#include "OptimizedSelfAdjointMatrix66f.h"

namespace cxy
{

    OptimizedSelfAdjointMatrix66f::OptimizedSelfAdjointMatrix66f()
    {
    }

    void OptimizedSelfAdjointMatrix66f::setZero()
    {
        for(size_t idx = 0; idx < Size; idx++)
            data[idx] = 0.0f;
    }

    #if !defined(ENABLE_SSE) && !defined(__SSE__)

    // TODO: Ugly temporary replacement for SSE instructions to make rankUpdate() work.
        // TODO: code faster version

        struct __m128 {
            __m128(float a, float b, float c, float d) {
                data[0] = a;
                data[1] = b;
                data[2] = c;
                data[3] = d;
            }
            float& operator[](int idx) {
                return data[idx];
            }
            const float& operator[](int idx) const {
                return data[idx];
            }
            float data[4];
        };
        __m128 _mm_set1_ps(float v) {
            return __m128(v, v, v, v);
        }
        __m128 _mm_loadu_ps(const float* d) {
            return __m128(d[0], d[1], d[2], d[3]);
        }
        __m128 _mm_load_ps(const float* d) {
            return __m128(d[0], d[1], d[2], d[3]);
        }
        __m128 _mm_movelh_ps(const __m128& a, const __m128& b) {
            return __m128(a[0], a[1], b[0], b[1]);
        }
        __m128 _mm_movehl_ps(const __m128& a, const __m128& b) {
            return __m128(b[2], b[3], a[2], a[3]);
        }
        __m128 _mm_unpacklo_ps(const __m128& a, const __m128& b) {
            return __m128(a[0], b[0], a[1], b[1]);
        }
        __m128 _mm_mul_ps(const __m128& a, const __m128& b) {
            return __m128(a[0] * b[0], a[1] * b[1], a[2] * b[2], a[3] * b[3]);
        }
        __m128 _mm_add_ps(const __m128& a, const __m128& b) {
            return __m128(a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]);
        }
        void _mm_store_ps(float* p, const __m128& a) {
            p[0] = a[0];
            p[1] = a[1];
            p[2] = a[2];
            p[3] = a[3];
        }

    #endif



    void OptimizedSelfAdjointMatrix66f::toEigen(Matrix66f& m) const
    {
        Eigen::Matrix<float, 6, 6> tmp;
        size_t idx = 0;

        for(size_t i = 0; i < 6; i += 2)
        {
            for(size_t j = i; j < 6; j += 2)
            {
                tmp(i  , j  ) = data[idx++];
                tmp(i  , j+1) = data[idx++];
                tmp(i+1, j  ) = data[idx++];
                tmp(i+1, j+1) = data[idx++];
            }
        }

        tmp.selfadjointView<Eigen::Upper>().evalTo(m);
    }
}





