using System;

namespace LJMCollision
{
    public static partial class CollisionDetection
    {
        /// <summary>레이와 OBB의 교차 판정 (OBB 로컬 공간에서 Slab method)</summary>
        public static RaycastResult RayVsOBB(Ray ray, OBB box, float maxDistance = float.MaxValue)
        {
            Vec3 localOrigin = box.WorldToLocal(ray.Origin);
            Vec3 localDir = new Vec3(
                Vec3.Dot(ray.Direction, box.AxisX),
                Vec3.Dot(ray.Direction, box.AxisY),
                Vec3.Dot(ray.Direction, box.AxisZ));

            float tMin = 0f;
            float tMax = maxDistance;

            if (!SlabTest(localOrigin.X, localDir.X, -box.HalfSize.X, box.HalfSize.X, ref tMin, ref tMax))
                return RaycastResult.None;
            if (!SlabTest(localOrigin.Y, localDir.Y, -box.HalfSize.Y, box.HalfSize.Y, ref tMin, ref tMax))
                return RaycastResult.None;
            if (!SlabTest(localOrigin.Z, localDir.Z, -box.HalfSize.Z, box.HalfSize.Z, ref tMin, ref tMax))
                return RaycastResult.None;

            Vec3 point = ray.GetPoint(tMin);
            Vec3 normal = ComputeOBBSurfaceNormal(point, box);

            return new RaycastResult
            {
                Hit = true,
                Distance = tMin,
                Point = point,
                Normal = normal,
            };
        }

        /// <summary>레이와 캡슐의 교차 판정 (플레이어 피격용)</summary>
        public static RaycastResult RayVsCapsule(Ray ray, Capsule capsule)
        {
            Vec3 segDir = capsule.Top - capsule.Bottom;
            Vec3 w0 = ray.Origin - capsule.Bottom;

            float a = Vec3.Dot(ray.Direction, ray.Direction);
            float b = Vec3.Dot(ray.Direction, segDir);
            float c = Vec3.Dot(segDir, segDir);
            float d = Vec3.Dot(ray.Direction, w0);
            float e = Vec3.Dot(segDir, w0);

            float denom = a * c - b * b;

            float tRay, tSeg;
            if (MathF.Abs(denom) < MathUtils.Epsilon)
            {
                tRay = 0f;
                tSeg = c > MathUtils.Epsilon ? e / c : 0f;
            }
            else
            {
                tRay = (b * e - c * d) / denom;
                tSeg = (a * e - b * d) / denom;
            }

            tRay = MathF.Max(0f, tRay);
            tSeg = MathUtils.Clamp(tSeg, 0f, 1f);

            Vec3 closestOnRay = ray.GetPoint(tRay);
            Vec3 closestOnSeg = capsule.Bottom + segDir * tSeg;

            float dist = Vec3.Distance(closestOnRay, closestOnSeg);
            if (dist > capsule.Radius)
                return RaycastResult.None;

            Vec3 normal = (closestOnRay - closestOnSeg).Normalized;
            Vec3 point = closestOnSeg + normal * capsule.Radius;

            return new RaycastResult
            {
                Hit = true,
                Distance = tRay,
                Point = point,
                Normal = normal,
            };
        }

        /// <summary>레이와 구체의 교차 판정</summary>
        public static RaycastResult RayVsSphere(Ray ray, Sphere sphere)
        {
            Vec3 oc = ray.Origin - sphere.Center;
            float b = Vec3.Dot(oc, ray.Direction);
            float c = oc.SqrMagnitude - sphere.Radius * sphere.Radius;

            float discriminant = b * b - c;
            if (discriminant < 0f)
                return RaycastResult.None;

            float sqrtD = MathF.Sqrt(discriminant);
            float t = -b - sqrtD;

            if (t < 0f)
                t = -b + sqrtD;
            if (t < 0f)
                return RaycastResult.None;

            Vec3 point = ray.GetPoint(t);
            Vec3 normal = (point - sphere.Center).Normalized;

            return new RaycastResult
            {
                Hit = true,
                Distance = t,
                Point = point,
                Normal = normal,
            };
        }
    }
}
