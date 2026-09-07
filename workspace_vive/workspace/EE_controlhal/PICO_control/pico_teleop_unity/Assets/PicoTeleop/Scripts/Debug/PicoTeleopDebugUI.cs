using EgoARX.PicoTeleop.Core;
using EgoARX.PicoTeleop.Protocol;
using UnityEngine;

namespace EgoARX.PicoTeleop.Hud
{
    /// <summary>
    /// World-space status panel so tracking and UDP send can be checked while wearing the HMD.
    /// </summary>
    public sealed class PicoTeleopDebugUI : MonoBehaviour
    {
        PicoTeleopManager _manager;
        Transform _panel;
        TextMesh _text;
        float _refreshTimer;

        public void Bind(PicoTeleopManager manager)
        {
            _manager = manager;
        }

        void Start()
        {
            var panelGo = new GameObject("PicoTeleopDebugPanel");
            _panel = panelGo.transform;
            _panel.localScale = Vector3.one * 0.01f;

            var background = GameObject.CreatePrimitive(PrimitiveType.Quad);
            background.name = "Background";
            background.transform.SetParent(_panel, false);
            background.transform.localPosition = new Vector3(8f, -5f, 0.1f);
            background.transform.localScale = new Vector3(22f, 14f, 1f);
            var meshRenderer = background.GetComponent<MeshRenderer>();
            var shader = Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default") ?? Shader.Find("Standard");
            if (shader != null)
            {
                meshRenderer.material = new Material(shader);
            }
            meshRenderer.material.color = new Color(0f, 0f, 0f, 0.65f);
            var collider = background.GetComponent<Collider>();
            if (collider != null)
            {
                Destroy(collider);
            }

            var textGo = new GameObject("Text");
            textGo.transform.SetParent(_panel, false);
            textGo.transform.localPosition = Vector3.zero;
            _text = textGo.AddComponent<TextMesh>();
            _text.fontSize = 42;
            _text.characterSize = 0.35f;
            _text.anchor = TextAnchor.UpperLeft;
            _text.alignment = TextAlignment.Left;
            _text.color = Color.white;
            _text.text = "PicoTeleop";
        }

        void LateUpdate()
        {
            if (_manager == null || _panel == null || _text == null)
            {
                return;
            }

            Camera cam = Camera.main;
            if (cam != null)
            {
                Vector3 forward = cam.transform.forward;
                forward.y = 0f;
                if (forward.sqrMagnitude < 1e-4f)
                {
                    forward = cam.transform.forward;
                }

                _panel.position = cam.transform.position + forward.normalized * 1.4f + Vector3.up * 0.15f;
                _panel.rotation = Quaternion.LookRotation(_panel.position - cam.transform.position, Vector3.up);
            }

            _refreshTimer += Time.unscaledDeltaTime;
            if (_refreshTimer < 0.1f)
            {
                return;
            }

            _refreshTimer = 0f;
            var right = _manager.LastRight;
            var left = _manager.LastLeft;
            _text.text =
                "PicoTeleop\n" +
                "Target: " + _manager.ReceiverIp + ":" + _manager.ReceiverPort + "\n" +
                "Send: " + _manager.SendRateHz + " Hz\n" +
                "Seq: " + _manager.Seq + "\n" +
                "Right Tracking: " + (right.Valid ? "OK" : "LOST") + "\n" +
                "Right: " + InvariantFloat.Format(right.Position.x) + " " +
                InvariantFloat.Format(right.Position.y) + " " +
                InvariantFloat.Format(right.Position.z) + "\n" +
                "Left Tracking: " + (left.Valid ? "OK" : "LOST") + "\n" +
                "Left: " + InvariantFloat.Format(left.Position.x) + " " +
                InvariantFloat.Format(left.Position.y) + " " +
                InvariantFloat.Format(left.Position.z) + "\n" +
                "R trig=" + InvariantFloat.Format(right.Trigger) +
                " grip=" + InvariantFloat.Format(right.Grip) + "\n" +
                "HMD: " + (_manager.LastHead.Valid ? "OK" : "LOST");
        }
    }
}
