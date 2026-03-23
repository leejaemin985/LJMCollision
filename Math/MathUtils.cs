using System;

namespace LJMCollision
{
    public static class MathUtils
    {
        public const float Epsilon = 1e-6f;

        public static float Clamp(float v, float min, float max)
            => Math.Clamp(v, min, max);

        public static bool Approximately(float a, float b, float eps = Epsilon)
            => MathF.Abs(a - b) < eps;
    }
}
