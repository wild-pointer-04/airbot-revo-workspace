using EgoARX.PicoTeleop.Hud;
using EgoARX.PicoTeleop.Input;
using EgoARX.PicoTeleop.Network;
using EgoARX.PicoTeleop.Protocol;
using UnityEngine;
using Unity.XR.PICO.TOBSupport;

namespace EgoARX.PicoTeleop.Core
{
    /// <summary>
    /// Combines XR input, packet assembly, and UDP send. Does not talk to any robot SDK.
    /// Poses remain in XR tracking space.
    /// </summary>
    public sealed class PicoTeleopManager : MonoBehaviour
    {
        [Header("Network")]
        [SerializeField] TeleopNetworkConfig networkConfig;

        [Tooltip("Ubuntu middle_interface IP. Used when Network Config asset is empty.")]
        [SerializeField] string receiverIp = "127.0.0.1";

        [SerializeField] int receiverPort = 15000;

        [Tooltip("Independent send rate, not tied to screen FPS. Typical: 30 / 60 / 72 / 90.")]
        [SerializeField] int sendRateHz = 60;

        [SerializeField] bool enableDebugLog = false;

        [Header("Debug")]
        [SerializeField] bool showDebugPanel = true;

        [Tooltip("PICO Enterprise: keep display/tracking active while the headset is off-head.")]
        [SerializeField] bool keepAwakeOffHead = true;

        XRDeviceReader _reader;
        UdpTeleopSender _sender;
        PicoTeleopDebugUI _debugUi;
        uint _seq;
        float _sendAccumulator;

        public string ReceiverIp => ActiveIp;
        public int ReceiverPort => ActivePort;
        public int SendRateHz => ActiveRate;
        public uint Seq => _seq;
        public XRHeadState LastHead { get; private set; }
        public XRControllerState LastLeft { get; private set; }
        public XRControllerState LastRight { get; private set; }
        public string LastSendError => _sender != null ? _sender.LastError : string.Empty;

        string ActiveIp => networkConfig != null ? networkConfig.receiverIp : receiverIp;
        int ActivePort => networkConfig != null ? networkConfig.receiverPort : receiverPort;
        int ActiveRate => networkConfig != null ? networkConfig.sendRateHz : sendRateHz;
        bool ActiveLog => networkConfig != null ? networkConfig.enableDebugLog : enableDebugLog;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        static void AvoidIl2CppLocaleAbort()
        {
            var invariant = System.Globalization.CultureInfo.InvariantCulture;
            System.Globalization.CultureInfo.DefaultThreadCurrentCulture = invariant;
            System.Globalization.CultureInfo.DefaultThreadCurrentUICulture = invariant;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void EnsureInstance()
        {
#if UNITY_2023_1_OR_NEWER
            if (FindFirstObjectByType<PicoTeleopManager>() != null)
#else
            if (FindObjectOfType<PicoTeleopManager>() != null)
#endif
            {
                return;
            }

            var go = new GameObject("PicoTeleopManager");
            go.AddComponent<PicoTeleopManager>();
        }

        void Awake()
        {
            if (networkConfig != null)
            {
                networkConfig.ApplyDefaultsIfInvalid();
            }

            receiverPort = Mathf.Clamp(receiverPort, 1, 65535);
            sendRateHz = Mathf.Clamp(sendRateHz, 1, 120);
            _reader = new XRDeviceReader();
        }

        void OnEnable()
        {
            RecreateSender();
            ConfigureEnterpriseKeepAwake();
            if (showDebugPanel && _debugUi == null)
            {
                _debugUi = gameObject.AddComponent<PicoTeleopDebugUI>();
                _debugUi.Bind(this);
            }
        }

        void OnDisable()
        {
            _sender?.Dispose();
            _sender = null;
            RestoreEnterprisePSensor();
        }

        void ConfigureEnterpriseKeepAwake()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (!keepAwakeOffHead)
            {
                return;
            }

            if (!PXR_Enterprise.InitEnterpriseService())
            {
                Debug.LogWarning("[PicoTeleop] Enterprise service initialization failed; off-head keep-awake unavailable.");
                return;
            }

            PXR_Enterprise.BindEnterpriseService(bound =>
            {
                if (!bound)
                {
                    Debug.LogWarning("[PicoTeleop] Enterprise service bind failed; off-head keep-awake unavailable.");
                    return;
                }

                PXR_Enterprise.PropertySetSleepDelay(SleepDelayTimeEnum.NEVER);
                PXR_Enterprise.PropertySetScreenOffDelay(
                    ScreenOffDelayTimeEnum.NEVER,
                    result => Debug.Log($"[PicoTeleop] Screen-off NEVER result={result}"));
                PXR_Enterprise.SwitchSystemFunction(
                    SystemFunctionSwitchEnum.SFS_PSENSOR,
                    SwitchEnum.S_OFF);
                Debug.Log("[PicoTeleop] Off-head keep-awake enabled (PSensor disabled while app runs).");
            });
#endif
        }

        void RestoreEnterprisePSensor()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (!keepAwakeOffHead)
            {
                return;
            }

            PXR_Enterprise.SwitchSystemFunction(
                SystemFunctionSwitchEnum.SFS_PSENSOR,
                SwitchEnum.S_ON);
            PXR_Enterprise.UnBindEnterpriseService();
#endif
        }

        void Update()
        {
            if (Time.frameCount < 8)
            {
                return;
            }

            int rate = Mathf.Clamp(ActiveRate, 1, 120);
            _sendAccumulator += Time.unscaledDeltaTime;
            float interval = 1f / rate;
            int safety = 0;
            while (_sendAccumulator >= interval && safety < 4)
            {
                _sendAccumulator -= interval;
                SendOneFrame();
                safety++;
            }
        }

        public void ApplyNetworkSettings(string ip, int port, int rateHz)
        {
            receiverIp = ip;
            receiverPort = Mathf.Clamp(port, 1, 65535);
            sendRateHz = Mathf.Clamp(rateHz, 1, 120);
            if (networkConfig != null)
            {
                networkConfig.receiverIp = receiverIp;
                networkConfig.receiverPort = receiverPort;
                networkConfig.sendRateHz = sendRateHz;
            }

            RecreateSender();
        }

        void SendOneFrame()
        {
            LastHead = _reader.ReadHead();
            LastLeft = _reader.ReadLeftController();
            LastRight = _reader.ReadRightController();

            _seq++;
            var packet = PicoTeleopPacket.Create(_seq, LastHead, LastLeft, LastRight);
            _sender?.Send(packet);
        }

        void RecreateSender()
        {
            _sender?.Dispose();
            try
            {
                _sender = new UdpTeleopSender(ActiveIp, ActivePort, ActiveLog);
                Debug.Log($"[PicoTeleop] UDP target {ActiveIp}:{ActivePort} at {ActiveRate} Hz");
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[PicoTeleop] Failed to create UDP sender: {ex.Message}");
                _sender = null;
            }
        }
    }
}
