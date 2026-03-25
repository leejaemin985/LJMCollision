using System;

namespace LJMCollision
{
    /// <summary>
    /// 충돌 판정 유틸리티.
    /// 카테고리별 partial class로 분리:
    ///   - CollisionDetection.Capsule.cs  (CapsuleVsOBB, CapsuleVsCapsule)
    ///   - CollisionDetection.Ray.cs      (RayVsOBB, RayVsCapsule, RayVsSphere)
    ///   - CollisionDetection.Sphere.cs   (SphereVsOBB, SphereVsCapsule)
    ///   - CollisionDetection.OBB.cs      (OBBVsOBB)
    /// </summary>
    public static partial class CollisionDetection
    {
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

        /// <summary>두 선분 사이의 최근접점 쌍을 구함</summary>
        internal static void ClosestPointsSegmentSegment(
            Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2,
            out Vec3 closestA, out Vec3 closestB)
        {
            Vec3 d1 = q1 - p1;
            Vec3 d2 = q2 - p2;
            Vec3 r = p1 - p2;

            float a = d1.SqrMagnitude;
            float e = d2.SqrMagnitude;
            float f = Vec3.Dot(d2, r);

            float s, t;

            if (a < MathUtils.Epsilon && e < MathUtils.Epsilon)
            {
                closestA = p1;
                closestB = p2;
                return;
            }

            if (a < MathUtils.Epsilon)
            {
                s = 0f;
                t = MathUtils.Clamp(f / e, 0f, 1f);
            }
            else
            {
                float c = Vec3.Dot(d1, r);
                if (e < MathUtils.Epsilon)
                {
                    t = 0f;
                    s = MathUtils.Clamp(-c / a, 0f, 1f);
                }
                else
                {
                    float b = Vec3.Dot(d1, d2);
                    float denom = a * e - b * b;

                    if (MathF.Abs(denom) > MathUtils.Epsilon)
                        s = MathUtils.Clamp((b * f - c * e) / denom, 0f, 1f);
                    else
                        s = 0f;

                    t = (b * s + f) / e;

                    if (t < 0f)
                    {
                        t = 0f;
                        s = MathUtils.Clamp(-c / a, 0f, 1f);
                    }
                    else if (t > 1f)
                    {
                        t = 1f;
                        s = MathUtils.Clamp((b - c) / a, 0f, 1f);
                    }
                }
            }

            closestA = p1 + d1 * s;
            closestB = p2 + d2 * t;
        }

        /// <summary>OBB를 특정 축에 투영한 반길이</summary>
        static float ProjectOBBOnAxis(OBB box, Vec3 axis)
        {
            return MathF.Abs(Vec3.Dot(axis, box.AxisX)) * box.HalfSize.X
                 + MathF.Abs(Vec3.Dot(axis, box.AxisY)) * box.HalfSize.Y
                 + MathF.Abs(Vec3.Dot(axis, box.AxisZ)) * box.HalfSize.Z;
        }

        /// <summary>SAT 축 하나에 대한 겹침 테스트</summary>
        static bool TestSATAxis(Vec3 axis, OBB a, OBB b, Vec3 centerDiff,
            ref float minDepth, ref Vec3 minAxis)
        {
            float projA = ProjectOBBOnAxis(a, axis);
            float projB = ProjectOBBOnAxis(b, axis);
            float dist = MathF.Abs(Vec3.Dot(centerDiff, axis));

            float overlap = projA + projB - dist;
            if (overlap < 0f) return false;

            if (overlap < minDepth)
            {
                minDepth = overlap;
                minAxis = axis;
            }
            return true;
        }

        /// <summary>점이 OBB 내부에 있는지 판별</summary>
        static bool IsPointInsideOBB(Vec3 point, OBB box)
        {
            Vec3 local = box.WorldToLocal(point);
            return MathF.Abs(local.X) <= box.HalfSize.X
                && MathF.Abs(local.Y) <= box.HalfSize.Y
                && MathF.Abs(local.Z) <= box.HalfSize.Z;
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
