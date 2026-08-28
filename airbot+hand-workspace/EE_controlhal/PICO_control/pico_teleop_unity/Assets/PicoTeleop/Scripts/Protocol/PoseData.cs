using UnityEngine;

namespace EgoARX.PicoTeleop.Protocol
{
    /// <summary>
    /// Tracking-space pose. Position: meters [x, y, z]. Rotation: quaternion [x, y, z, w].
    /// </summary>
    public struct PoseData
    {
        public bool Valid;
        public Vector3 Position;
        public Quaternion Rotation;

        public static PoseData From(Vector3 position, Quaternion rotation, bool valid)
        {
            return new PoseData
            {
                Valid = valid,
                Position = position,
                Rotation = rotation
            };
        }
    }
}
