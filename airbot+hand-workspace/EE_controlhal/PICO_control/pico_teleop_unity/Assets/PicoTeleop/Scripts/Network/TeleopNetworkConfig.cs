using UnityEngine;

namespace EgoARX.PicoTeleop.Network
{
    [CreateAssetMenu(fileName = "TeleopNetworkConfig", menuName = "PicoTeleop/Network Config")]
    public sealed class TeleopNetworkConfig : ScriptableObject
    {
        [Tooltip("Ubuntu middle_interface IP on the same LAN.")]
        public string receiverIp = "127.0.0.1";

        public int receiverPort = 15000;

        [Tooltip("Independent UDP send rate. Typical: 30, 60, 72, 90.")]
        public int sendRateHz = 60;

        public bool enableDebugLog = false;

        public void ApplyDefaultsIfInvalid()
        {
            if (string.IsNullOrWhiteSpace(receiverIp))
            {
                receiverIp = "127.0.0.1";
            }

            receiverPort = Mathf.Clamp(receiverPort, 1, 65535);
            sendRateHz = Mathf.Clamp(sendRateHz, 1, 120);
        }
    }
}
