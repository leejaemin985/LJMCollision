using System;

namespace LJMCollision
{
    public struct CollisionResult
    {
        public bool Hit;
        public Vec3 ContactPoint;
        public Vec3 Normal;
        public float Depth;

        public static CollisionResult None => new() { Hit = false };
    }

    public struct MoveResult
    {
        public Vec3 Position;
        public bool Grounded;
        public Vec3 GroundNormal;
        public bool HitCeiling;
        public bool HitWall;
        public Vec3 WallNormal;
    }

    /// <summary>이동 후 겹침 검사 결과. 충돌한 면들의 정보만 담고, 밀어냄은 하지 않음.</summary>
    public struct OverlapResult
    {
        public Vec3 Position;
        public bool HasCollision;
        public CollisionResult[] Contacts;
        public int ContactCount;

        /// <summary>경사 각도 기준으로 바닥/천장/벽 여부를 판별</summary>
        public void Classify(float maxSlopeAngle, out bool grounded, out Vec3 groundNormal,
            out bool hitCeiling, out bool hitWall, out Vec3 wallNormal)
        {
            grounded = false;
            groundNormal = Vec3.Up;
            hitCeiling = false;
            hitWall = false;
            wallNormal = Vec3.Zero;

            for (int i = 0; i < ContactCount; i++)
            {
                float slopeAngle = MathF.Acos(MathF.Min(Contacts[i].Normal.Y, 1f)) * (180f / MathF.PI);
                if (slopeAngle <= maxSlopeAngle)
                {
                    grounded = true;
                    groundNormal = Contacts[i].Normal;
                }
                else if (slopeAngle >= 180f - maxSlopeAngle)
                {
                    hitCeiling = true;
                }
                else
                {
                    hitWall = true;
                    wallNormal = Contacts[i].Normal;
                }
            }
        }
    }

    public struct RaycastResult
    {
        public bool Hit;
        public float Distance;
        public Vec3 Point;
        public Vec3 Normal;

        public static RaycastResult None => new() { Hit = false };
    }
}
