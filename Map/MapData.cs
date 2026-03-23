using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace LJMCollision
{
    /// <summary>
    /// 맵 전체 데이터. Unity에서 Export한 JSON을 서버에서 로드한다.
    /// </summary>
    public class MapData
    {
        [JsonPropertyName("boxes")]
        public List<BoxData> Boxes { get; set; } = new();

        public static MapData FromJson(string json)
        {
            return JsonSerializer.Deserialize<MapData>(json)!;
        }

        public static MapData FromFile(string path)
        {
            string json = System.IO.File.ReadAllText(path);
            return FromJson(json);
        }

        public OBB[] ToOBBArray()
        {
            var result = new OBB[Boxes.Count];
            for (int i = 0; i < Boxes.Count; i++)
            {
                var b = Boxes[i];
                result[i] = OBB.FromTransform(
                    new Vec3(b.X, b.Y, b.Z),
                    new Vec3(b.SX, b.SY, b.SZ),
                    Quat.FromEuler(b.RX, b.RY, b.RZ));
            }
            return result;
        }
    }

    /// <summary>
    /// 개별 박스 데이터.
    /// Unity에서 position, scale, rotation(euler)을 Export한다.
    /// </summary>
    public class BoxData
    {
        [JsonPropertyName("x")] public float X { get; set; }
        [JsonPropertyName("y")] public float Y { get; set; }
        [JsonPropertyName("z")] public float Z { get; set; }
        [JsonPropertyName("sx")] public float SX { get; set; }
        [JsonPropertyName("sy")] public float SY { get; set; }
        [JsonPropertyName("sz")] public float SZ { get; set; }
        [JsonPropertyName("rx")] public float RX { get; set; }
        [JsonPropertyName("ry")] public float RY { get; set; }
        [JsonPropertyName("rz")] public float RZ { get; set; }
    }
}
