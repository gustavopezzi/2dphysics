#ifndef VEC2_H
#define VEC2_H

struct Vec2 {
    float x;
    float y;

    Vec2();
    Vec2(float x, float y);
    
    ~Vec2() = default;

    void Add(const Vec2& v);                 // v1.Add(v2)
    void Sub(const Vec2& v);                 // v1.Sub(v2)
    void Scale(const float n);               // v1.Scale(n)
    Vec2 Rotate(const float angle) const;    // v1.Rotate(angle)

    float Magnitude() const;                 // v1.Magnitude()
    float MagnitudeSquared() const;          // v1.MagnitudeSquared()

    Vec2& Normalize();                       // v1.Normalize()
    Vec2 UnitVector() const;                 // v1.UnitVector()
    Vec2 Normal() const;                     // n = v1.Normal()

    float Dot(const Vec2& v) const;          // v1.Dot(v2)
    float Cross(const Vec2& v) const;        // v1.Cross(v2)

    Vec2& operator = (const Vec2& v);        // v1 = v2
    bool operator == (const Vec2& v) const;  // v1 == v2
    bool operator != (const Vec2& v) const;  // v1 != v2
    
    Vec2 operator + (const Vec2& v) const;   // v1 + v2
    Vec2 operator - (const Vec2& v) const;   // v1 - v2
    Vec2 operator * (const float n) const;   // v1 * n
    Vec2 operator / (const float n) const;   // v1 / n
    Vec2 operator - ();                      // -v1

    Vec2& operator += (const Vec2& v);       // v1 += v2
    Vec2& operator -= (const Vec2& v);       // v1 -= v2
    Vec2& operator *= (const float n);       // v1 *= n
    Vec2& operator /= (const float n);       // v1 /= n
};

#endif
