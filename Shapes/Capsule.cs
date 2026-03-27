using System;

namespace LJMCollision
{
    /// <summary>
    /// 캡슐 (양 끝이 반구인 실린더).
    /// 임의 방향을 지원하며, 기본값은 Y축 정렬.
    /// </summary>
    public struct Capsule
    {
        public Vec3 Center;
        public float Radius;
        public float Height;
        /// <summary>캡슐의 축 방향 (정규화). 기본값 Up(0,1,0).</summary>
        public Vec3 Direction;

        /// <summary>Y축 정렬 캡슐 (기존 호환)</summary>
        public Capsule(Vec3 center, float radius, float height)
        {
            Center = center;
            Radius = radius;
            Height = MathF.Max(height, radius * 2f);
            Direction = Vec3.Up;
        }

        /// <summary>임의 방향 캡슐</summary>
        public Capsule(Vec3 center, float radius, float height, Vec3 direction)
        {
            Center = center;
            Radius = radius;
            Height = MathF.Max(height, radius * 2f);
            Direction = direction.Normalized;
        }

        /// <summary>두 끝점으로 캡슐 생성 (Swept Sphere 등)</summary>
        public static Capsule FromEndpoints(Vec3 pointA, Vec3 pointB, float radius)
        {
            Vec3 diff = pointB - pointA;
            float segLen = diff.Magnitude;
            Vec3 dir = segLen > MathUtils.Epsilon ? diff * (1f / segLen) : Vec3.Up;
            Vec3 center = (pointA + pointB) * 0.5f;
            float height = segLen + radius * 2f;
            return new Capsule(center, radius, height, dir);
        }

        /// <summary>캡슐 내부 선분의 상단 점</summary>
        public Vec3 Top => Center + Direction * HalfSegment;

        /// <summary>캡슐 내부 선분의 하단 점</summary>
        public Vec3 Bottom => Center - Direction * HalfSegment;

        /// <summary>선분 절반 길이 (캡슐 중심에서 반구 중심까지)</summary>
        float HalfSegment => MathF.Max(0f, Height * 0.5f - Radius);
    }
}
