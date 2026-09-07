using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using EgoARX.PicoTeleop.Protocol;
using UnityEngine;

namespace EgoARX.PicoTeleop.Network
{
    /// <summary>
    /// Loopback (USB + adb reverse) uses TCP because reverse only forwards TCP.
    /// Other hosts keep one JSON datagram per UDP packet (WiFi).
    /// </summary>
    public sealed class UdpTeleopSender : IDisposable
    {
        readonly bool _useTcp;
        UdpClient _udp;
        TcpClient _tcp;
        NetworkStream _tcpStream;
        IPEndPoint _endpoint;
        bool _logEnabled;
        float _nextWarnTime;

        public string TargetHost { get; private set; }
        public int TargetPort { get; private set; }
        public int SendCount { get; private set; }
        public string LastError { get; private set; } = string.Empty;

        public UdpTeleopSender(string host, int port, bool enableDebugLog)
        {
            TargetHost = host;
            TargetPort = port;
            _logEnabled = enableDebugLog;
            _endpoint = new IPEndPoint(ParseHost(host), port);
            _useTcp = IPAddress.IsLoopback(_endpoint.Address);
            if (!_useTcp)
            {
                _udp = new UdpClient();
            }
        }

        public void UpdateEndpoint(string host, int port, bool enableDebugLog)
        {
            TargetHost = host;
            TargetPort = port;
            _logEnabled = enableDebugLog;
            _endpoint = new IPEndPoint(ParseHost(host), port);
        }

        public bool Send(PicoTeleopPacket packet)
        {
            try
            {
                string json = packet.ToJson();
                byte[] bytes = _useTcp
                    ? Encoding.UTF8.GetBytes(json + "\n")
                    : Encoding.UTF8.GetBytes(json);

                if (_useTcp)
                {
                    EnsureTcp();
                    _tcpStream.Write(bytes, 0, bytes.Length);
                }
                else
                {
                    _udp.Send(bytes, bytes.Length, _endpoint);
                }

                SendCount++;
                LastError = string.Empty;

                if (_logEnabled && SendCount % 60 == 1)
                {
                    string proto = _useTcp ? "TCP" : "UDP";
                    Debug.Log($"[PicoTeleop] {proto} seq={packet.Seq} -> {TargetHost}:{TargetPort} bytes={bytes.Length}");
                }

                return true;
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
                DropTcp();
                if (Time.unscaledTime >= _nextWarnTime)
                {
                    _nextWarnTime = Time.unscaledTime + 2f;
                    Debug.LogWarning($"[PicoTeleop] send failed: {ex.Message}");
                }

                return false;
            }
        }

        public void Dispose()
        {
            DropTcp();
            _udp?.Dispose();
            _udp = null;
        }

        void EnsureTcp()
        {
            if (_tcp != null && _tcp.Connected && _tcpStream != null)
            {
                return;
            }

            DropTcp();
            var client = new TcpClient();
            client.NoDelay = true;
            client.SendTimeout = 200;
            IAsyncResult ar = client.BeginConnect(_endpoint.Address, _endpoint.Port, null, null);
            if (!ar.AsyncWaitHandle.WaitOne(200))
            {
                client.Close();
                throw new TimeoutException("TCP connect timeout");
            }

            client.EndConnect(ar);
            _tcp = client;
            _tcpStream = client.GetStream();
            Debug.Log($"[PicoTeleop] TCP connected {TargetHost}:{TargetPort}");
        }

        void DropTcp()
        {
            try
            {
                _tcpStream?.Close();
            }
            catch (Exception)
            {
            }

            try
            {
                _tcp?.Close();
            }
            catch (Exception)
            {
            }

            _tcpStream = null;
            _tcp = null;
        }

        static IPAddress ParseHost(string host)
        {
            if (IPAddress.TryParse(host, out IPAddress address))
            {
                return address;
            }

            IPHostEntry entry = Dns.GetHostEntry(host);
            if (entry.AddressList.Length == 0)
            {
                throw new InvalidOperationException($"Cannot resolve host: {host}");
            }

            return entry.AddressList[0];
        }
    }
}
