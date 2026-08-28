using UnityEngine;

namespace EgoARX.PicoTeleop.Input
{
    /// <summary>
    /// HMD pose in XR Origin / tracking space. Units: meters. Quaternion: (x, y, z, w).
    /// </summary>
    public struct XRHeadState
    {
        public bool Valid;
        public Vector3 Position;
        public Quaternion Rotation;

        public static XRHeadState Invalid => new XRHeadState
        {
            Valid = false,
            Position = Vector3.zero,
            Rotation = Quaternion.identity
        };
    }
}
