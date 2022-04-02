#ifndef VECN_H
#define VECN_H

struct VecN {
    int N;
    float* data;

    VecN();
    VecN(int N);
    VecN(const VecN& v);
    ~VecN();

    void Zero();                               // v1.Zero()
    float Dot(const VecN& v) const;            // v1.Dot(v2)

    VecN& operator = (const VecN& v);          // v1 = v2
    VecN operator + (const VecN& v) const;     // v1 + v2
    VecN operator - (const VecN& v) const;     // v1 - v2
    VecN operator * (const float n) const;     // v1 * n
    const VecN& operator += (const VecN& v);   // v1 += v2
    const VecN& operator -= (const VecN& v);   // v1 -= v2
    const VecN& operator *= (const float n);   // v1 *= n
    float operator [] (const int index) const; // v1[index]
    float& operator [] (const int index);      // v1[index]
};

#endif
