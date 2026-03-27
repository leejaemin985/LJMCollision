using System;

namespace LJMCollision
{
    /// <summary>
    /// 충돌 월드. 맵의 OBB들을 보관하고, 충돌 검출/응답/레이캐스트를 제공한다.
    ///
    /// 사용 패턴:
    ///   OverlapTest → 검출만 (응답은 호출자가 결정: 파괴, 반사 등)
    ///   MoveAndSlide → 검출 + 슬라이드 응답 (플레이어 이동용)
    ///   Raycast → 선분 검사
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

        // ── 검출 전용 (충돌 응답은 호출자가 결정) ──

        public OverlapResult OverlapTest(Capsule capsule, Vec3 velocity)
            => OverlapTestInternal(capsule.Center + velocity,
                (center, box) => CollisionDetection.CapsuleVsOBB(new Capsule(center, capsule.Radius, capsule.Height), box));

        public OverlapResult OverlapTest(Sphere sphere, Vec3 velocity)
            => OverlapTestInternal(sphere.Center + velocity,
                (center, box) => CollisionDetection.SphereVsOBB(new Sphere(center, sphere.Radius), box));

        public OverlapResult OverlapTest(OBB obb, Vec3 velocity)
            => OverlapTestInternal(obb.Center + velocity,
                (center, box) => CollisionDetection.OBBVsOBB(new OBB(center, obb.HalfSize, obb.Rotation), box));

        OverlapResult OverlapTestInternal(Vec3 center, Func<Vec3, OBB, CollisionResult> testFunc)
        {
            var contacts = new CollisionResult[_boxes.Length];
            int count = 0;

            for (int i = 0; i < _boxes.Length; i++)
            {
                var result = testFunc(center, _boxes[i]);
                if (result.Hit)
                    contacts[count++] = result;
            }

            return new OverlapResult
            {
                Position = center,
                HasCollision = count > 0,
                Contacts = contacts,
                ContactCount = count,
            };
        }

        // ── MoveAndSlide (OverlapTest + 슬라이드 응답) ──

        public MoveResult MoveAndSlide(Capsule capsule, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(capsule.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.CapsuleVsOBB(new Capsule(center, capsule.Radius, capsule.Height), box));

        public MoveResult MoveAndSlide(Sphere sphere, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(sphere.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.SphereVsOBB(new Sphere(center, sphere.Radius), box));

        public MoveResult MoveAndSlide(OBB obb, Vec3 velocity, float maxSlopeAngle = 45f)
            => MoveAndSlideInternal(obb.Center + velocity, maxSlopeAngle,
                (center, box) => CollisionDetection.OBBVsOBB(new OBB(center, obb.HalfSize, obb.Rotation), box));

        /// <summary>OverlapTest를 반복하며 슬라이드(밀어냄) 응답을 적용.</summary>
        MoveResult MoveAndSlideInternal(Vec3 center, float maxSlopeAngle, Func<Vec3, OBB, CollisionResult> testFunc)
        {
            bool grounded = false;
            bool hitCeiling = false;
            bool hitWall = false;
            Vec3 groundNormal = Vec3.Up;
            Vec3 wallNormal = Vec3.Zero;

            for (int iter = 0; iter < MaxPushIterations; iter++)
            {
                var overlap = OverlapTestInternal(center, testFunc);
                if (!overlap.HasCollision) break;

                for (int i = 0; i < overlap.ContactCount; i++)
                {
                    var contact = overlap.Contacts[i];

                    // 슬라이드 응답: normal 방향으로 밀어냄
                    center = center + contact.Normal * contact.Depth;

                    // 분류
                    float slopeAngle = MathF.Acos(MathF.Min(contact.Normal.Y, 1f)) * (180f / MathF.PI);
                    if (slopeAngle <= maxSlopeAngle)
                    {
                        grounded = true;
                        groundNormal = contact.Normal;
                    }
                    else if (slopeAngle >= 180f - maxSlopeAngle)
                    {
                        hitCeiling = true;
                    }
                    else
                    {
                        hitWall = true;
                        wallNormal = contact.Normal;
                    }
                }
            }

            return new MoveResult
            {
                Position = center, Grounded = grounded, GroundNormal = groundNormal,
                HitCeiling = hitCeiling, HitWall = hitWall, WallNormal = wallNormal
            };
        }

        // ── 레이캐스트 ──

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
