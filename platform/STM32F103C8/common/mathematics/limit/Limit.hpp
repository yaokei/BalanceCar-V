
/**
 * @file Limits.hpp
 *
 * Limiting / constrain helper functions
 *
 */

#pragma once

#include <stdint.h>
#include <float.h>
#include <math.h>

namespace math
{

float min(float val1, float val2);

int min(int val1, int val2);

unsigned min(unsigned val1, unsigned val2);

uint64_t min(uint64_t val1, uint64_t val2);

double min(double val1, double val2);

float max(float val1, float val2);

int max(int val1, int val2);

unsigned max(unsigned val1, unsigned val2);

uint64_t max(uint64_t val1, uint64_t val2);

double max(double val1, double val2);

float constrain(float val, float min, float max);

int constrain(int val, int min, int max);

unsigned constrain(unsigned val, unsigned min, unsigned max);

uint64_t constrain(uint64_t val, uint64_t min, uint64_t max);

double constrain(double val, double min, double max);

float radians(float degrees);

double radians(double degrees);

float degrees(float radians);

double degrees(double radians);

// are two floats equal
inline bool is_equal(const float fVal1, const float fVal2) { return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; }

// is a float is zero
inline bool is_zero(const float fVal1) { return fabsf(fVal1) < FLT_EPSILON ? true : false; }

}
