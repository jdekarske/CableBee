#ifndef UTIL_H
#define UTIL_H

// Absolute value
#define ABS(a) ((a) < 0 ? -(a) : (a))

// Find if the number is positive
#define SIGN(num) (((num) > 0) ? 1 : 0)

// Constrain the value between two other numbers
#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

#endif