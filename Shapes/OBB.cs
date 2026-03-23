namespace LJMCollision
{
    /// <summary>
    /// Oriented Bounding Box. 회전 가능한 박스.
    /// 맵 지형에 사용.
    /// </summary>
    public struct OBB
    {
        public Vec3 Center;
        public Vec3 HalfSize;
        public Quat Rotation;

        // 캐시된 축 (생성 시 계산)
        public Vec3 AxisX;
        public Vec3 AxisY;
        public Vec3 AxisZ;

        public OBB(Vec3 center, Vec3 halfSize, Quat rotation)
        {
            Center = center;
            HalfSize = halfSize;
            Rotation = rotation;
            AxisX = rotation.Rotate(Vec3.Right);
            AxisY = rotation.Rotate(Vec3.Up);
            AxisZ = rotation.Rotate(Vec3.Forward);
        }

        /// <summary>회전 없는 OBB (AABB와 동일)</summary>
        public static OBB FromAABB(Vec3 center, Vec3 halfSize)
        {
            return new OBB(center, halfSize, Quat.Identity);
        }

        /// <summary>Unity Transform 호환: position, scale, rotation</summary>
        public static OBB FromTransform(Vec3 position, Vec3 scale, Quat rotation)
        {
            return new OBB(position, scale * 0.5f, rotation);
        }

        /// <summary>월드 좌표를 OBB 로컬 좌표로 변환</summary>
        public Vec3 WorldToLocal(Vec3 worldPoint)
        {
            Vec3 d = worldPoint - Center;
            return new Vec3(
                Vec3.Dot(d, AxisX),
                Vec3.Dot(d, AxisY),
                Vec3.Dot(d, AxisZ));
        }

        /// <summary>OBB 로컬 좌표를 월드 좌표로 변환</summary>
        public Vec3 LocalToWorld(Vec3 localPoint)
        {
            return Center
                + AxisX * localPoint.X
                + AxisY * localPoint.Y
                + AxisZ * localPoint.Z;
        }

        /// <summary>OBB 표면에서 가장 가까운 점 (월드 좌표)</summary>
        public Vec3 ClosestPoint(Vec3 point)
        {
            Vec3 local = WorldToLocal(point);
            Vec3 clamped = Vec3.Clamp(local, -HalfSize, HalfSize);
            return LocalToWorld(clamped);
        }
    }
}
