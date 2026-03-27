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

        /// <summary>캡슐을 이동시키되, OBB에 충돌하면 밀어냄 처리된 최종 위치와 바닥 정보를 반환</summary>
        public MoveResult MoveAndSlide(Capsule capsule, Vec3 velocity, float maxSlopeAngle = 45f)
        {
            Vec3 newCenter = capsule.Center + velocity;
            var moved = new Capsule(newCenter, capsule.Radius, capsule.Height);
            bool grounded = false;
            bool hitCeiling = false;
            Vec3 groundNormal = Vec3.Up;

            for (int iter = 0; iter < 4; iter++)
            {
                bool resolved = false;

                for (int i = 0; i < _boxes.Length; i++)
                {
                    var result = CollisionDetection.CapsuleVsOBB(moved, _boxes[i]);
                    if (!result.Hit) continue;

                    newCenter = newCenter + result.Normal * result.Depth;
                    moved = new Capsule(newCenter, capsule.Radius, capsule.Height);
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

            return new MoveResult { Position = newCenter, Grounded = grounded, GroundNormal = groundNormal, HitCeiling = hitCeiling };
        }

        /// <summary>구체를 이동시키되, OBB에 충돌하면 밀어냄 처리된 최종 위치와 바닥 정보를 반환</summary>
        public MoveResult MoveAndSlide(Sphere sphere, Vec3 velocity, float maxSlopeAngle = 45f)
        {
            Vec3 newCenter = sphere.Center + velocity;
            var moved = new Sphere(newCenter, sphere.Radius);
            bool grounded = false;
            bool hitCeiling = false;
            Vec3 groundNormal = Vec3.Up;

            for (int iter = 0; iter < 4; iter++)
            {
                bool resolved = false;

                for (int i = 0; i < _boxes.Length; i++)
                {
                    var result = CollisionDetection.SphereVsOBB(moved, _boxes[i]);
                    if (!result.Hit) continue;

                    newCenter = newCenter + result.Normal * result.Depth;
                    moved = new Sphere(newCenter, sphere.Radius);
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

            return new MoveResult { Position = newCenter, Grounded = grounded, GroundNormal = groundNormal, HitCeiling = hitCeiling };
        }

        /// <summary>OBB를 이동시키되, 맵 OBB에 충돌하면 밀어냄 처리된 최종 위치와 바닥 정보를 반환</summary>
        public MoveResult MoveAndSlide(OBB box, Vec3 velocity, float maxSlopeAngle = 45f)
        {
            Vec3 newCenter = box.Center + velocity;
            var moved = new OBB(newCenter, box.HalfSize, box.Rotation);
            bool grounded = false;
            bool hitCeiling = false;
            Vec3 groundNormal = Vec3.Up;

            for (int iter = 0; iter < 4; iter++)
            {
                bool resolved = false;

                for (int i = 0; i < _boxes.Length; i++)
                {
                    var result = CollisionDetection.OBBVsOBB(moved, _boxes[i]);
                    if (!result.Hit) continue;

                    newCenter = newCenter + result.Normal * result.Depth;
                    moved = new OBB(newCenter, box.HalfSize, box.Rotation);
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

            return new MoveResult { Position = newCenter, Grounded = grounded, GroundNormal = groundNormal, HitCeiling = hitCeiling };
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
