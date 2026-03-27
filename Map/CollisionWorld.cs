using System;

namespace LJMCollision
{
    /// <summary>
    /// 충돌 월드. 맵의 OBB들을 보관하고, 캡슐/레이와의 충돌 판정을 제공한다.
    /// </summary>
    public class CollisionWorld
    {
        OBB[] _boxes = Array.Empty<OBB>();

        public int BoxCount => _boxes.Length;

        public void Load(MapData mapData)
        {
            _boxes = mapData.ToOBBArray();
        }

        const int MaxPushIterations = 4;

        public MoveResult MoveAndSlide(Capsule capsule, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(capsule.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.CapsuleVsOBB(new Capsule(center, capsule.Radius, capsule.Height), box));

        public MoveResult MoveAndSlide(Sphere sphere, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(sphere.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.SphereVsOBB(new Sphere(center, sphere.Radius), box));

        public MoveResult MoveAndSlide(OBB obb, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(obb.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.OBBVsOBB(new OBB(center, obb.HalfSize, obb.Rotation), box));

        /// <summary>공통 MoveAndSlide 루프. Shape별 충돌 검사만 delegate로 주입.</summary>
        MoveResult MoveAndSlideInternal(Vec3 center, float maxSlopeAngle, Func<Vec3, OBB, CollisionResult> testFunc)
        {
            bool grounded = false;
            bool hitCeiling = false;
            Vec3 groundNormal = Vec3.Up;

            for (int iter = 0; iter < MaxPushIterations; iter++)
            {
                bool resolved = false;

                for (int i = 0; i < _boxes.Length; i++)
                {
                    var result = testFunc(center, _boxes[i]);
                    if (!result.Hit) continue;

                    center = center + result.Normal * result.Depth;
                    resolved = true;

                    float slopeAngle = MathF.Acos(MathF.Min(result.Normal.Y, 1f)) * (180f / MathF.PI);
                    if (slopeAngle <= maxSlopeAngle)
                    {
                        grounded = true;
                        groundNormal = result.Normal;
                    }
                    else if (slopeAngle >= 180f - maxSlopeAngle)
                    {
                        hitCeiling = true;
                    }
                }

                if (!resolved) break;
            }

            return new MoveResult { Position = center, Grounded = grounded, GroundNormal = groundNormal, HitCeiling = hitCeiling };
        }

        /// <summary>레이캐스트. 가장 가까운 OBB와의 교차를 반환.</summary>
        public RaycastResult Raycast(Ray ray, float maxDistance = 100f)
        {
            var closest = RaycastResult.None;
            closest.Distance = maxDistance;

            for (int i = 0; i < _boxes.Length; i++)
            {
                var result = CollisionDetection.RayVsOBB(ray, _boxes[i], closest.Distance);
                if (result.Hit && result.Distance < closest.Distance)
                    closest = result;
            }

            return closest.Hit ? closest : RaycastResult.None;
        }

    }
}
