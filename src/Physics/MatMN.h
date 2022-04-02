#ifndef MATMN_H
#define MATMN_H

#include "VecN.h"

struct MatMN {
    int M;      // rows
    int N;      // cols
    VecN* rows; // the rows of the matrix with N columns inside

    MatMN();
    MatMN(int M, int N);
    MatMN(const MatMN& m);
    ~MatMN();

    void Zero();
    MatMN Transpose() const;

    const MatMN& operator = (const MatMN& m);  // m1 = m2
    VecN operator * (const VecN& v) const;     // m1 * v
    MatMN operator * (const MatMN& m) const;   // m1 * m2

    static VecN SolveGaussSeidel(const MatMN& A, const VecN& b);
};

#endif
