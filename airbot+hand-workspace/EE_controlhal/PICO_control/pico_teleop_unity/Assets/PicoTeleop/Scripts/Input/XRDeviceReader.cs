using UnityEngine;
using UnityEngine.XR;

namespace EgoARX.PicoTeleop.Input
{
    /// <summary>
    /// Reads HMD and controller state through the standard Unity XR / OpenXR InputDevices API.
    /// Does not convert into a robot frame. Values stay in tracking space.
    /// </summary>
    public sealed class XRDeviceReader
    {
        public XRHeadState ReadHead()
        {
            return ReadHead(InputDevices.GetDeviceAtXRNode(XRNode.Head));
        }

        public XRControllerState ReadLeftController()
        {
            return ReadController(InputDevices.GetDeviceAtXRNode(XRNode.LeftHand));
        }

        public XRControllerState ReadRightController()
        {
            return ReadController(InputDevices.GetDeviceAtXRNode(XRNode.RightHand));
        }

        public static XRHeadState ReadHead(InputDevice device)
        {
            if (!device.isValid)
            {
                return XRHeadState.Invalid;
            }

            var state = XRHeadState.Invalid;
            bool isTracked = TryIsTracked(device);
            bool hasPos = device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
            bool hasRot = device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);

            state.Valid = isTracked && hasPos && hasRot && IsFinite(position) && IsFinite(rotation);
            state.Position = state.Valid ? position : Vector3.zero;
            state.Rotation = state.Valid ? rotation : Quaternion.identity;
            return state;
        }

        public static XRControllerState ReadController(InputDevice device)
        {
            if (!device.isValid)
            {
                return XRControllerState.Invalid;
            }

            var state = XRControllerState.Invalid;
            bool isTracked = TryIsTracked(device);
            bool hasPos = device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
            bool hasRot = device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);

            state.Valid = isTracked && hasPos && hasRot && IsFinite(position) && IsFinite(rotation);
            state.Position = state.Valid ? position : Vector3.zero;
            state.Rotation = state.Valid ? rotation : Quaternion.identity;

            if (device.TryGetFeatureValue(CommonUsages.deviceVelocity, out Vector3 linear) && IsFinite(linear))
            {
                state.LinearVelocity = linear;
                state.HasLinearVelocity = true;
            }

            if (device.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out Vector3 angular) && IsFinite(angular))
            {
                state.AngularVelocity = angular;
                state.HasAngularVelocity = true;
            }

            if (device.TryGetFeatureValue(CommonUsages.trigger, out float trigger) && IsFinite(trigger))
            {
                state.Trigger = Mathf.Clamp01(trigger);
            }

            if (device.TryGetFeatureValue(CommonUsages.grip, out float grip) && IsFinite(grip))
            {
                state.Grip = Mathf.Clamp01(grip);
            }

            device.TryGetFeatureValue(CommonUsages.primaryButton, out state.PrimaryButton);
            device.TryGetFeatureValue(CommonUsages.secondaryButton, out state.SecondaryButton);

            if (device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 stick) && IsFinite(stick))
            {
                state.Joystick = stick;
            }

            return state;
        }

        static bool TryIsTracked(InputDevice device)
        {
            if (device.TryGetFeatureValue(CommonUsages.isTracked, out bool isTracked))
            {
                return isTracked;
            }

            return true;
        }

        static bool IsFinite(Vector3 v)
        {
            return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        }

        static bool IsFinite(Vector2 v)
        {
            return IsFinite(v.x) && IsFinite(v.y);
        }

        static bool IsFinite(Quaternion q)
        {
            return IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        }

        static bool IsFinite(float v)
        {
            return !float.IsNaN(v) && !float.IsInfinity(v);
        }
    }
}
