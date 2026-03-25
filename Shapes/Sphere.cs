namespace LJMCollision
{
    /// <summary>
    /// 구체. 투사체 충돌체에 사용.
    /// </summary>
    public struct Sphere
    {
        public Vec3 Center;
        public float Radius;

        public Sphere(Vec3 center, float radius)
        {
            Center = center;
            Radius = radius;
        }
    }
}
