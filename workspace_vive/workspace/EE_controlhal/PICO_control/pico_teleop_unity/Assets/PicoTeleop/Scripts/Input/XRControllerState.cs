using UnityEngine;

namespace EgoARX.PicoTeleop.Input
{
    /// <summary>
    /// Controller pose and buttons in XR Origin / tracking space.
    /// Position is meters. Rotation quaternion is (x, y, z, w).
    /// Velocity is optional; pose remains valid if velocity cannot be read.
    /// </summary>
    public struct XRControllerState
    {
        public bool Valid;
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
        public bool HasLinearVelocity;
        public bool HasAngularVelocity;
        public float Trigger;
        public float Grip;
        public bool PrimaryButton;
        public bool SecondaryButton;
        public Vector2 Joystick;

        public static XRControllerState Invalid => new XRControllerState
        {
            Valid = false,
            Position = Vector3.zero,
            Rotation = Quaternion.identity,
            LinearVelocity = Vector3.zero,
            AngularVelocity = Vector3.zero,
            Trigger = 0f,
            Grip = 0f,
            Joystick = Vector2.zero
        };
    }
}
