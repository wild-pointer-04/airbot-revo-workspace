using System.Globalization;
using System.Text;
using EgoARX.PicoTeleop.Input;
using UnityEngine;

namespace EgoARX.PicoTeleop.Protocol
{
    /// <summary>
    /// pico_teleop_v1 UDP JSON payload.
    /// Coordinates are XR tracking space. Quaternion order is [x, y, z, w].
    /// Unity does not convert into a robot frame.
    /// </summary>
    public sealed class PicoTeleopPacket
    {
        public const string ProtocolName = "pico_teleop";
        public const int ProtocolVersion = 1;

        public uint Seq;
        public long DeviceTimeNs;
        public long WallTimeUnixMs;
        public PoseData Hmd;
        public ControllerData LeftController;
        public ControllerData RightController;

        public static PicoTeleopPacket Create(
            uint seq,
            XRHeadState hmd,
            XRControllerState left,
            XRControllerState right)
        {
            return new PicoTeleopPacket
            {
                Seq = seq,
                DeviceTimeNs = (long)(Time.realtimeSinceStartupAsDouble * 1_000_000_000d),
                WallTimeUnixMs = System.DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                Hmd = PoseData.From(hmd.Position, hmd.Rotation, hmd.Valid),
                LeftController = ControllerData.From(left),
                RightController = ControllerData.From(right)
            };
        }

        public string ToJson()
        {
            var sb = new StringBuilder(1024);
            sb.Append('{');
            AppendString(sb, "protocol", ProtocolName);
            sb.Append(',');
            AppendNumber(sb, "version", ProtocolVersion);
            sb.Append(',');
            AppendNumber(sb, "seq", Seq);
            sb.Append(',');
            AppendNumber(sb, "device_time_ns", DeviceTimeNs);
            sb.Append(',');
            AppendNumber(sb, "wall_time_unix_ms", WallTimeUnixMs);
            sb.Append(',');
            AppendPose(sb, "hmd", Hmd);
            sb.Append(',');
            AppendController(sb, "left_controller", LeftController);
            sb.Append(',');
            AppendController(sb, "right_controller", RightController);
            sb.Append('}');
            return sb.ToString();
        }

        static void AppendPose(StringBuilder sb, string key, PoseData pose)
        {
            sb.Append('"').Append(key).Append("\":{");
            AppendBool(sb, "valid", pose.Valid);
            sb.Append(',');
            AppendVec3(sb, "position", pose.Position);
            sb.Append(',');
            AppendQuat(sb, "rotation", pose.Rotation);
            sb.Append('}');
        }

        static void AppendController(StringBuilder sb, string key, ControllerData c)
        {
            sb.Append('"').Append(key).Append("\":{");
            AppendBool(sb, "valid", c.Valid);
            sb.Append(',');
            AppendVec3(sb, "position", c.Position);
            sb.Append(',');
            AppendQuat(sb, "rotation", c.Rotation);
            sb.Append(',');
            AppendVec3(sb, "linear_velocity", c.LinearVelocity);
            sb.Append(',');
            AppendVec3(sb, "angular_velocity", c.AngularVelocity);
            sb.Append(',');
            AppendNumber(sb, "trigger", c.Trigger);
            sb.Append(',');
            AppendNumber(sb, "grip", c.Grip);
            sb.Append(',');
            AppendBool(sb, "primary_button", c.PrimaryButton);
            sb.Append(',');
            AppendBool(sb, "secondary_button", c.SecondaryButton);
            sb.Append(',');
            AppendVec2(sb, "joystick", c.Joystick);
            sb.Append('}');
        }

        static void AppendString(StringBuilder sb, string key, string value)
        {
            sb.Append('"').Append(key).Append("\":\"").Append(value).Append('"');
        }

        static void AppendBool(StringBuilder sb, string key, bool value)
        {
            sb.Append('"').Append(key).Append("\":").Append(value ? "true" : "false");
        }

        static void AppendNumber(StringBuilder sb, string key, long value)
        {
            sb.Append('"').Append(key).Append("\":").Append(value.ToString(CultureInfo.InvariantCulture));
        }

        static void AppendNumber(StringBuilder sb, string key, float value)
        {
            sb.Append('"').Append(key).Append("\":");
            InvariantFloat.Append(sb, value);
        }

        static void AppendVec3(StringBuilder sb, string key, Vector3 v)
        {
            sb.Append('"').Append(key).Append("\":[");
            InvariantFloat.Append(sb, v.x);
            sb.Append(',');
            InvariantFloat.Append(sb, v.y);
            sb.Append(',');
            InvariantFloat.Append(sb, v.z);
            sb.Append(']');
        }

        static void AppendVec2(StringBuilder sb, string key, Vector2 v)
        {
            sb.Append('"').Append(key).Append("\":[");
            InvariantFloat.Append(sb, v.x);
            sb.Append(',');
            InvariantFloat.Append(sb, v.y);
            sb.Append(']');
        }

        static void AppendQuat(StringBuilder sb, string key, Quaternion q)
        {
            sb.Append('"').Append(key).Append("\":[");
            InvariantFloat.Append(sb, q.x);
            sb.Append(',');
            InvariantFloat.Append(sb, q.y);
            sb.Append(',');
            InvariantFloat.Append(sb, q.z);
            sb.Append(',');
            InvariantFloat.Append(sb, q.w);
            sb.Append(']');
        }
    }
}
