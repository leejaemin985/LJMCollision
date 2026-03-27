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
