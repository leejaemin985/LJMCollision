namespace LJMCollision
{
    /// <summary>
    /// 레이 (시작점 + 방향).
    /// 히트스캔 총알 판정에 사용.
    /// </summary>
    public struct Ray
    {
        public Vec3 Origin;
        public Vec3 Direction;

        public Ray(Vec3 origin, Vec3 direction)
        {
            Origin = origin;
            Direction = direction.Normalized;
        }

        public Vec3 GetPoint(float distance) => Origin + Direction * distance;
    }
}
