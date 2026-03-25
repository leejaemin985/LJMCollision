using System;

namespace LJMCollision
{
    public static partial class CollisionDetection
    {
        /// <summary>구체와 OBB의 충돌 판정 (투사체 vs 맵)</summary>
        public static CollisionResult SphereVsOBB(Sphere sphere, OBB box)
        {
            Vec3 closest = box.ClosestPoint(sphere.Center);
            Vec3 diff = sphere.Center - closest;
            float distSq = diff.SqrMagnitude;

            if (distSq > sphere.Radius * sphere.Radius)
                return CollisionResult.None;

            float dist = MathF.Sqrt(distSq);
            Vec3 normal;
            float depth;

            if (dist < MathUtils.Epsilon)
            {
                normal = ComputeOBBPenetrationNormal(sphere.Center, box, out float penDepth);
                depth = penDepth + sphere.Radius;
            }
            else
            {
                normal = diff * (1f / dist);
                depth = sphere.Radius - dist;
            }

            return new CollisionResult
            {
                Hit = true,
                ContactPoint = closest,
                Normal = normal,
                Depth = depth,
            };
        }

        /// <summary>구체와 캡슐의 충돌 판정 (투사체 vs 플레이어)</summary>
        public static CollisionResult SphereVsCapsule(Sphere sphere, Capsule capsule)
        {
            Vec3 segDir = capsule.Top - capsule.Bottom;
            float segLenSq = segDir.SqrMagnitude;
            float t;

            if (segLenSq < MathUtils.Epsilon)
            {
                t = 0f;
            }
            else
            {
                t = Vec3.Dot(sphere.Center - capsule.Bottom, segDir) / segLenSq;
                t = MathUtils.Clamp(t, 0f, 1f);
            }

            Vec3 closestOnSeg = capsule.Bottom + segDir * t;
            Vec3 diff = sphere.Center - closestOnSeg;
            float dist = diff.Magnitude;
            float combinedRadius = sphere.Radius + capsule.Radius;

            if (dist > combinedRadius)
                return CollisionResult.None;

            Vec3 normal;
            if (dist < MathUtils.Epsilon)
                normal = Vec3.Up;
            else
                normal = diff * (1f / dist);

            return new CollisionResult
            {
                Hit = true,
                ContactPoint = closestOnSeg + normal * capsule.Radius,
                Normal = normal,
                Depth = combinedRadius - dist,
            };
        }
    }
}
