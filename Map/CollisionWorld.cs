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

        /// <summary>캡슐을 이동시키되, OBB에 충돌하면 밀어냄 처리된 최종 위치를 반환</summary>
        public Vec3 MoveAndSlide(Capsule capsule, Vec3 velocity)
        {
            Vec3 newCenter = capsule.Center + velocity;
            var moved = new Capsule(newCenter, capsule.Radius, capsule.Height);

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
                }

                if (!resolved) break;
            }

            return newCenter;
        }

        /// <summary>구체를 이동시키되, OBB에 충돌하면 밀어냄 처리된 최종 위치를 반환</summary>
        public Vec3 MoveAndSlide(Sphere sphere, Vec3 velocity)
        {
            Vec3 newCenter = sphere.Center + velocity;
            var moved = new Sphere(newCenter, sphere.Radius);

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
                }

                if (!resolved) break;
            }

            return newCenter;
        }

        /// <summary>OBB를 이동시키되, 맵 OBB에 충돌하면 밀어냄 처리된 최종 위치를 반환</summary>
        public Vec3 MoveAndSlide(OBB box, Vec3 velocity)
        {
            Vec3 newCenter = box.Center + velocity;
            var moved = new OBB(newCenter, box.HalfSize, box.Rotation);

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
                }

                if (!resolved) break;
            }

            return newCenter;
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

        /// <summary>특정 위치 아래에 바닥이 있는지 확인 (중력용)</summary>
        public bool GroundCheck(Vec3 position, float radius, out float groundY, out Vec3 groundNormal)
        {
            var ray = new Ray(position, Vec3.Down);
            float closestDist = float.MaxValue;
            groundY = float.MinValue;
            groundNormal = Vec3.Up;
            bool found = false;

            for (int i = 0; i < _boxes.Length; i++)
            {
                var result = CollisionDetection.RayVsOBB(ray, _boxes[i], closestDist);
                if (result.Hit && result.Distance < closestDist)
                {
                    closestDist = result.Distance;
                    groundY = result.Point.Y;
                    groundNormal = result.Normal;
                    found = true;
                }
            }

            return found;
        }
    }
}
