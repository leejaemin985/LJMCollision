using System;

namespace LJMCollision
{
    public struct Vec3
    {
        public float X;
        public float Y;
        public float Z;

        public Vec3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vec3 Zero => new(0, 0, 0);
        public static Vec3 One => new(1, 1, 1);
        public static Vec3 Up => new(0, 1, 0);
        public static Vec3 Down => new(0, -1, 0);
        public static Vec3 Right => new(1, 0, 0);
        public static Vec3 Forward => new(0, 0, 1);

        public float SqrMagnitude => X * X + Y * Y + Z * Z;
        public float Magnitude => MathF.Sqrt(SqrMagnitude);

        public Vec3 Normalized
        {
            get
            {
                float mag = Magnitude;
                if (mag < 1e-6f) return Zero;
                return this * (1f / mag);
            }
        }

        // 연산자
        public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Vec3 operator *(Vec3 v, float s) => new(v.X * s, v.Y * s, v.Z * s);
        public static Vec3 operator *(float s, Vec3 v) => v * s;
        public static Vec3 operator -(Vec3 v) => new(-v.X, -v.Y, -v.Z);

        // 벡터 연산
        public static float Dot(Vec3 a, Vec3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;

        public static Vec3 Cross(Vec3 a, Vec3 b) => new(
            a.Y * b.Z - a.Z * b.Y,
            a.Z * b.X - a.X * b.Z,
            a.X * b.Y - a.Y * b.X);

        public static Vec3 Min(Vec3 a, Vec3 b) => new(
            MathF.Min(a.X, b.X), MathF.Min(a.Y, b.Y), MathF.Min(a.Z, b.Z));

        public static Vec3 Max(Vec3 a, Vec3 b) => new(
            MathF.Max(a.X, b.X), MathF.Max(a.Y, b.Y), MathF.Max(a.Z, b.Z));

        public static float Distance(Vec3 a, Vec3 b) => (a - b).Magnitude;

        public static Vec3 Lerp(Vec3 a, Vec3 b, float t) => a + (b - a) * t;

        public static Vec3 Clamp(Vec3 v, Vec3 min, Vec3 max) => new(
            Math.Clamp(v.X, min.X, max.X),
            Math.Clamp(v.Y, min.Y, max.Y),
            Math.Clamp(v.Z, min.Z, max.Z));

        public override string ToString() => $"({X:F2}, {Y:F2}, {Z:F2})";
    }
}
