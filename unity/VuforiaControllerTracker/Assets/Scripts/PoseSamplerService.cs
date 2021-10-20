using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using Vuforia;

public class PoseSamplerService : Net.JSONMessageSubscriber
{
    public string hostname = "localhost";
    public UInt16 port = 6810;

    private ModelTargetBehaviour m_modelTarget;
    private Text m_connectionStateText;
    private Text m_trackingStateText;
    private Net.TCPClient m_client = null;
    private Net.Session m_session = null;
    private BlockingRingBuffer<Action> m_networkQueue = new BlockingRingBuffer<Action>(1024 * 1024, true);
    private bool m_okayToSendPose = false;

    [Serializable]
    private struct HelloMessage
    {
        public string message;
    }

    [Serializable]
    public struct VuforiaControllerPoseMessage
    {
        public Matrix4x4 pose_matrix;
    }

    protected override void Awake()
    {
        base.Awake();

        m_modelTarget = GameObject.FindObjectOfType<ModelTargetBehaviour>();
        m_connectionStateText = GameObject.Find("Text_ConnectionState").GetComponentInChildren<Text>();
        m_trackingStateText = GameObject.Find("Text_TrackingState").GetComponentInChildren<Text>();
        Debug.Assert(m_connectionStateText != null);
        Debug.Assert(m_trackingStateText != null);
        m_trackingStateText.text = "Not tracking";
        m_trackingStateText.color = Color.red;

        // Register after everything is wired up because callback fires immediately
        m_modelTarget.RegisterOnTrackableStatusChanged(OnTrackingChanged);
    }

    private void Start()
    {
        m_client = new Net.TCPClient();
        TryConnect();
    }

    private void OnDestroy()
    {
        m_modelTarget.UnregisterOnTrackableStatusChanged(OnTrackingChanged);
    }

    private void OnTrackingChanged(TrackableBehaviour.StatusChangeResult status)
    {
        m_okayToSendPose = false;

        switch (status.NewStatus)
        {
            case TrackableBehaviour.Status.TRACKED:
                m_okayToSendPose = true;
                m_trackingStateText.text = "Tracked";
                m_trackingStateText.color = Color.green;
                break;

            case TrackableBehaviour.Status.LIMITED:
                m_trackingStateText.text = "Limited";
                m_trackingStateText.color = Color.yellow;
                break;

            default:
                m_trackingStateText.text = "Not tracking";
                m_trackingStateText.color = Color.red;
                break;
        }
    }

    private void Update()
    {
        // Send latest pose
        if (m_okayToSendPose)
        {
            VuforiaControllerPoseMessage msg = new VuforiaControllerPoseMessage();
            msg.pose_matrix = m_modelTarget.transform.localToWorldMatrix;
            m_session?.Send(ref msg);
        }

        // Process network events
        while (m_networkQueue.TryDequeue(out Action Closure))
        {
            Closure();
        }
    }

    [Net.Handler()]
    private void OnUnknownMessage(Net.Session session, string json)
    {
        //Debug.LogError("Received unknown message");
    }

    [Net.Handler(typeof(HelloMessage))]
    private void OnHelloMessage(Net.Session session, string json)
    {
        HelloMessage msg = JsonUtility.FromJson<HelloMessage>(json);
        Debug.LogFormat("Received message: {0}", msg.message);
    }

    private void TryConnect()
    {
        MarkDisconnected();
        m_session = null;
        StartCoroutine(ConnectCoroutine());
    }

    private IEnumerator ConnectCoroutine()
    {
        yield return new WaitForSeconds(1);
        Debug.Log("Attempting to connect...");
        m_client.Connect(hostname, port, OnConnected, OnDisconnect, this);
    }

    private void OnConnected(Net.Session session, Exception exception)
    {
        m_networkQueue.Enqueue(() =>
        {
            m_session = session;
            if (session == null)
            {
                Debug.LogError("Connection failed");
                TryConnect();
            }
            else
            {
                MarkConnected();
                Debug.Log("Connection succeeded");
            }
        });
    }

    private void OnDisconnect(Net.Session session, Exception exception)
    {
        Debug.LogErrorFormat("Disconnected from {0}: {1}", session.RemoteEndpoint, exception);
        m_networkQueue.Enqueue(() =>
        {
            TryConnect();
        });
    }

    private void MarkDisconnected()
    {
        m_connectionStateText.text = "Not connected";
        m_connectionStateText.color = Color.red;
    }

    private void MarkConnected()
    {
        m_connectionStateText.text = "Connected";
        m_connectionStateText.color = Color.green;
    }
}
