using System;

namespace LJMCollision
{
    /// <summary>
    /// 캡슐 (양 끝이 반구인 실린더).
    /// 플레이어 충돌체에 사용.
    /// </summary>
    public struct Capsule
    {
        public Vec3 Center;
        public float Radius;
        public float Height;

        public Capsule(Vec3 center, float radius, float height)
        {
            Center = center;
            Radius = radius;
            Height = MathF.Max(height, radius * 2f);
        }

        /// <summary>캡슐 내부 선분의 상단 점</summary>
        public Vec3 Top => Center + Vec3.Up * HalfSegment;

        /// <summary>캡슐 내부 선분의 하단 점</summary>
        public Vec3 Bottom => Center + Vec3.Down * HalfSegment;

        /// <summary>선분 절반 길이 (캡슐 중심에서 반구 중심까지)</summary>
        float HalfSegment => MathF.Max(0f, Height * 0.5f - Radius);
    }
}
