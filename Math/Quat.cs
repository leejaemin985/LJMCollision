using System;

namespace LJMCollision
{
    public struct Quat
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Quat(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static Quat Identity => new(0, 0, 0, 1);

        /// <summary>Y축 회전 (degree)</summary>
        public static Quat FromYaw(float degrees)
        {
            float rad = degrees * MathF.PI / 180f * 0.5f;
            return new Quat(0f, MathF.Sin(rad), 0f, MathF.Cos(rad));
        }

        /// <summary>오일러 각(degree)에서 쿼터니언 생성 (YXZ 순서)</summary>
        public static Quat FromEuler(float pitchDeg, float yawDeg, float rollDeg)
        {
            float p = pitchDeg * MathF.PI / 180f * 0.5f;
            float y = yawDeg * MathF.PI / 180f * 0.5f;
            float r = rollDeg * MathF.PI / 180f * 0.5f;

            float sp = MathF.Sin(p), cp = MathF.Cos(p);
            float sy = MathF.Sin(y), cy = MathF.Cos(y);
            float sr = MathF.Sin(r), cr = MathF.Cos(r);

            return new Quat(
                cy * sp * cr + sy * cp * sr,
                sy * cp * cr - cy * sp * sr,
                cy * cp * sr - sy * sp * cr,
                cy * cp * cr + sy * sp * sr);
        }

        /// <summary>쿼터니언으로 벡터를 회전</summary>
        public Vec3 Rotate(Vec3 v)
        {
            // q * v * q^-1
            float tx = 2f * (Y * v.Z - Z * v.Y);
            float ty = 2f * (Z * v.X - X * v.Z);
            float tz = 2f * (X * v.Y - Y * v.X);

            return new Vec3(
                v.X + W * tx + (Y * tz - Z * ty),
                v.Y + W * ty + (Z * tx - X * tz),
                v.Z + W * tz + (X * ty - Y * tx));
        }

        /// <summary>로컬 축 3개를 반환 (Right, Up, Forward)</summary>
        public void GetAxes(out Vec3 right, out Vec3 up, out Vec3 forward)
        {
            right = Rotate(Vec3.Right);
            up = Rotate(Vec3.Up);
            forward = Rotate(Vec3.Forward);
        }

        public override string ToString() => $"({X:F2}, {Y:F2}, {Z:F2}, {W:F2})";
    }
}
