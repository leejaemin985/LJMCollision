using System;

namespace LJMCollision
{
    public static partial class CollisionDetection
    {
        /// <summary>캡슐과 OBB의 충돌 판정 + 밀어내기 정보 반환</summary>
        public static CollisionResult CapsuleVsOBB(Capsule capsule, OBB box)
        {
            Vec3 segClosest = ClosestPointOnSegmentToOBB(capsule.Top, capsule.Bottom, box);
            Vec3 boxClosest = box.ClosestPoint(segClosest);

            Vec3 diff = segClosest - boxClosest;
            float distSq = diff.SqrMagnitude;

            if (distSq > capsule.Radius * capsule.Radius)
                return CollisionResult.None;

            float dist = MathF.Sqrt(distSq);
            Vec3 normal;
            float depth;

            if (dist < MathUtils.Epsilon)
            {
                normal = ComputeOBBPenetrationNormal(segClosest, box, out float penDepth);
                depth = penDepth + capsule.Radius;
            }
            else
            {
                normal = diff * (1f / dist);
                depth = capsule.Radius - dist;
            }

            return new CollisionResult
            {
                Hit = true,
                ContactPoint = boxClosest,
                Normal = normal,
                Depth = depth,
            };
        }

        /// <summary>캡슐과 캡슐의 충돌 판정 (Swept Sphere 투사체 vs 플레이어 등)</summary>
        public static CollisionResult CapsuleVsCapsule(Capsule a, Capsule b)
        {
            ClosestPointsSegmentSegment(a.Bottom, a.Top, b.Bottom, b.Top,
                out Vec3 closestA, out Vec3 closestB);

            Vec3 diff = closestA - closestB;
            float distSq = diff.SqrMagnitude;
            float combinedRadius = a.Radius + b.Radius;

            if (distSq > combinedRadius * combinedRadius)
                return CollisionResult.None;

            float dist = MathF.Sqrt(distSq);

            Vec3 normal;
            if (dist < MathUtils.Epsilon)
                normal = Vec3.Up;
            else
                normal = diff * (1f / dist);

            return new CollisionResult
            {
                Hit = true,
                ContactPoint = closestB + normal * b.Radius,
                Normal = normal,
                Depth = combinedRadius - dist,
            };
        }
    }
}
