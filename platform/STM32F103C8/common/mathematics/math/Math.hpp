/**
 * @file Math.hpp
 *
 * Math libraries
 *
 */
 
#pragma once

/* Exponential and Logarithmic constants */

#define M_E             2.7182818284590452353602874713526625
#define M_SQRT2         1.4142135623730950488016887242096981
#define M_SQRT1_2       0.7071067811865475244008443621048490
#define M_LOG2E         1.4426950408889634073599246810018921
#define M_LOG10E        0.4342944819032518276511289189166051
#define M_LN2           0.6931471805599453094172321214581765
#define M_LN10          2.3025850929940456840179914546843642
#define M_DEG_TO_RAD    0.01745329251994329576923690768489
#define M_RAD_TO_DEG    57.295779513082320876798154814105

/* Trigonometric Constants */
#define M_PI        3.1415926535897932384626433832795029
#define M_TWOPI     6.283185307179586476925286766559
#define M_PI_2      1.5707963267948966192313216916397514
#define M_PI_4      0.7853981633974483096156608458198757
#define M_1_PI      0.3183098861837906715377675267450287
#define M_2_PI      0.6366197723675813430755350534900574
#define M_2_SQRTPI  1.1283791670955125738961589031215452

/* State indicators */
# define ERROR  (-1)
# define OK     (0)

// general lib
#include "LowPassFilter2p.hpp"
#include "Limit.hpp"
#include "Geo.hpp"

/*
// matrix lib
#include "Matrix.hpp"
#include "SquareMatrix.hpp"
#include "Vector.hpp"
#include "Vector2.hpp"
#include "Vector3.hpp"
#include "Dcm.hpp"
#include "Euler.hpp"
#include "Quaternion.hpp"
#include "Scalar.hpp"
*/

// math lib
#include "Vector.hpp"
#include "Matrix.hpp"
#include "Quaternion.hpp"
