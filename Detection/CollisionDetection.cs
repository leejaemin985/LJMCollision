using System;

namespace LJMCollision
{
    public static class CollisionDetection
    {
        // ── Capsule vs OBB ──

        /// <summary>캡슐과 OBB의 충돌 판정 + 밀어내기 정보 반환</summary>
        public static CollisionResult CapsuleVsOBB(Capsule capsule, OBB box)
        {
            // 캡슐 선분에서 OBB에 가장 가까운 점을 구함
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
                // 캡슐 선분이 OBB 내부에 있음 → 로컬 공간에서 가장 얕은 축으로 밀어냄
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

        // ── Ray vs OBB ──

        /// <summary>레이와 OBB의 교차 판정 (OBB 로컬 공간에서 Slab method)</summary>
        public static RaycastResult RayVsOBB(Ray ray, OBB box, float maxDistance = float.MaxValue)
        {
            // 레이를 OBB 로컬 공간으로 변환
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

        // ── Ray vs Capsule ──

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

        // ══════════════════════════════════════
        //  내부 유틸리티
        // ══════════════════════════════════════

        /// <summary>선분(a→b) 위에서 OBB에 가장 가까운 점</summary>
        static Vec3 ClosestPointOnSegmentToOBB(Vec3 a, Vec3 b, OBB box)
        {
            Vec3 dir = b - a;
            float segLen = dir.Magnitude;
            if (segLen < MathUtils.Epsilon) return a;

            Vec3 segNorm = dir * (1f / segLen);
            float t = Vec3.Dot(box.Center - a, segNorm);
            t = MathUtils.Clamp(t, 0f, segLen);

            return a + segNorm * t;
        }

        /// <summary>OBB 내부의 점에 대한 밀어내기 방향과 깊이 (로컬 공간 기반)</summary>
        static Vec3 ComputeOBBPenetrationNormal(Vec3 point, OBB box, out float depth)
        {
            Vec3 local = box.WorldToLocal(point);

            float dx = box.HalfSize.X - MathF.Abs(local.X);
            float dy = box.HalfSize.Y - MathF.Abs(local.Y);
            float dz = box.HalfSize.Z - MathF.Abs(local.Z);

            if (dx < dy && dx < dz)
            {
                depth = dx;
                return (local.X > 0 ? box.AxisX : -box.AxisX);
            }
            if (dy < dz)
            {
                depth = dy;
                return (local.Y > 0 ? box.AxisY : -box.AxisY);
            }
            depth = dz;
            return (local.Z > 0 ? box.AxisZ : -box.AxisZ);
        }

        /// <summary>OBB 표면의 법선 (교차점 기준, 로컬 공간에서 판별)</summary>
        static Vec3 ComputeOBBSurfaceNormal(Vec3 point, OBB box)
        {
            Vec3 local = box.WorldToLocal(point);

            float absX = MathF.Abs(local.X / box.HalfSize.X);
            float absY = MathF.Abs(local.Y / box.HalfSize.Y);
            float absZ = MathF.Abs(local.Z / box.HalfSize.Z);

            if (absX > absY && absX > absZ)
                return local.X > 0 ? box.AxisX : -box.AxisX;
            if (absY > absZ)
                return local.Y > 0 ? box.AxisY : -box.AxisY;
            return local.Z > 0 ? box.AxisZ : -box.AxisZ;
        }

        /// <summary>Slab method 축별 교차 테스트</summary>
        static bool SlabTest(float origin, float dir, float min, float max, ref float tMin, ref float tMax)
        {
            if (MathF.Abs(dir) < MathUtils.Epsilon)
                return origin >= min && origin <= max;

            float invDir = 1f / dir;
            float t1 = (min - origin) * invDir;
            float t2 = (max - origin) * invDir;

            if (t1 > t2) (t1, t2) = (t2, t1);

            tMin = MathF.Max(tMin, t1);
            tMax = MathF.Min(tMax, t2);

            return tMin <= tMax;
        }
    }
}
