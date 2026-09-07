using EgoARX.PicoTeleop.Input;
using UnityEngine;

namespace EgoARX.PicoTeleop.Protocol
{
    /// <summary>
    /// One controller snapshot in tracking space. Quaternion order is [x, y, z, w].
    /// </summary>
    public struct ControllerData
    {
        public bool Valid;
        public Vector3 Position;
        public Quaternion Rotation;
        public Vector3 LinearVelocity;
        public Vector3 AngularVelocity;
        public float Trigger;
        public float Grip;
        public bool PrimaryButton;
        public bool SecondaryButton;
        public Vector2 Joystick;

        public static ControllerData From(XRControllerState state)
        {
            return new ControllerData
            {
                Valid = state.Valid,
                Position = state.Position,
                Rotation = state.Rotation,
                LinearVelocity = state.LinearVelocity,
                AngularVelocity = state.AngularVelocity,
                Trigger = state.Trigger,
                Grip = state.Grip,
                PrimaryButton = state.PrimaryButton,
                SecondaryButton = state.SecondaryButton,
                Joystick = state.Joystick
            };
        }
    }
}
