using System;

namespace LJMCollision
{
    public static partial class CollisionDetection
    {
        /// <summary>
        /// OBB와 OBB의 충돌 판정 (SAT 15축).
        /// ContactPoint는 꼭짓점 기반 근사값 (정밀 contact manifold 아님).
        /// </summary>
        public static CollisionResult OBBVsOBB(OBB a, OBB b)
        {
            Vec3 centerDiff = b.Center - a.Center;

            float minDepth = float.MaxValue;
            Vec3 minAxis = Vec3.Zero;

            // A의 3개 면 축
            if (!TestSATAxis(a.AxisX, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;
            if (!TestSATAxis(a.AxisY, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;
            if (!TestSATAxis(a.AxisZ, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;

            // B의 3개 면 축
            if (!TestSATAxis(b.AxisX, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;
            if (!TestSATAxis(b.AxisY, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;
            if (!TestSATAxis(b.AxisZ, a, b, centerDiff, ref minDepth, ref minAxis)) return CollisionResult.None;

            // 9개 edge-edge 교차축 (Ai x Bj)
            Span<Vec3> axesA = stackalloc Vec3[] { a.AxisX, a.AxisY, a.AxisZ };
            Span<Vec3> axesB = stackalloc Vec3[] { b.AxisX, b.AxisY, b.AxisZ };

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Vec3 cross = Vec3.Cross(axesA[i], axesB[j]);
                    // 평행한 축 → 외적이 0에 가까움 → 스킵
                    if (cross.SqrMagnitude < MathUtils.Epsilon)
                        continue;
                    cross = cross.Normalized;
                    if (!TestSATAxis(cross, a, b, centerDiff, ref minDepth, ref minAxis))
                        return CollisionResult.None;
                }
            }

            // Normal이 A→B 방향을 가리키도록 보정
            if (Vec3.Dot(minAxis, centerDiff) < 0f)
                minAxis = -minAxis;

            // Contact point: 꼭짓점 기반 근사
            Vec3 contactPoint = ComputeOBBVsOBBContactPoint(a, b, minAxis);

            return new CollisionResult
            {
                Hit = true,
                ContactPoint = contactPoint,
                Normal = minAxis,
                Depth = minDepth,
            };
        }

        /// <summary>
        /// OBB vs OBB의 접촉점 근사.
        /// 양쪽 OBB 꼭짓점 중 상대 내부에 있는 것들의 평균을 사용.
        /// 둘 다 없으면 상대 중심에서 가장 가까운 표면점 평균.
        /// </summary>
        static Vec3 ComputeOBBVsOBBContactPoint(OBB a, OBB b, Vec3 normal)
        {
            Span<Vec3> aVerts = stackalloc Vec3[8];
            Span<Vec3> bVerts = stackalloc Vec3[8];
            a.GetVertices(aVerts);
            b.GetVertices(bVerts);

            Vec3 sumA = Vec3.Zero;
            int countA = 0;
            Vec3 sumB = Vec3.Zero;
            int countB = 0;

            for (int i = 0; i < 8; i++)
            {
                if (IsPointInsideOBB(aVerts[i], b))
                {
                    sumA = sumA + aVerts[i];
                    countA++;
                }
                if (IsPointInsideOBB(bVerts[i], a))
                {
                    sumB = sumB + bVerts[i];
                    countB++;
                }
            }

            if (countA > 0 && countB > 0)
                return (sumA * (1f / countA) + sumB * (1f / countB)) * 0.5f;
            if (countA > 0)
                return sumA * (1f / countA);
            if (countB > 0)
                return sumB * (1f / countB);

            // 꼭짓점이 하나도 상대 내부에 없음 (edge-edge 접촉)
            Vec3 closestA = a.ClosestPoint(b.Center);
            Vec3 closestB = b.ClosestPoint(a.Center);
            return (closestA + closestB) * 0.5f;
        }
    }
}
