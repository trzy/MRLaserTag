using System;
using System.Collections;
using System.Linq;
using UnityEngine;

public class NetworkManager : Net.JSONMessageSubscriber
{
    [SerializeField]
    private string m_hostname = "localhost";

    [SerializeField]
    private int m_port = 6810;

    [SerializeField]
    private float m_reconnectDelaySeconds = 1;

    private string m_clientID = Guid.NewGuid().ToString();
    private BlockingRingBuffer<Action> m_networkQueue = new BlockingRingBuffer<Action>(1024 * 1024, true);
    private MessageReceivingBehavior[] m_receivers;
    private Net.Session m_session;
    private float m_lastPingTime = 0;

    public string clientID
    {
        get
        {
            return m_clientID;
        }
    }

    public bool IsConnected
    {
        get
        {
            return m_session != null && m_session.IsConnected;
        }
    }

    public void Send<T>(ref T jsonSerializableObject) where T : struct
    {
        if (m_session != null)
        {
            m_session.Send(ref jsonSerializableObject);
        }
    }

    protected override void Awake()
    {
        base.Awake();
        TryConnect(0);
    }

    private void Start()
    {
        // Find all message receivers
        m_receivers = FindObjectsOfType<MonoBehaviour>().OfType<MessageReceivingBehavior>().ToArray();
        Debug.LogFormat("Found {0} message receivers", m_receivers.Length);
    }

    private void Update()
    {
        // Process network events
        while (m_networkQueue.TryDequeue(out Action networkEvent))
        {
            networkEvent();
        }

        if (m_session != null && Time.time - m_lastPingTime > 5)
        {
            PingMessage msg = new PingMessage(Time.time);
            m_session.Send(ref msg);
            m_lastPingTime = Time.time;
        }
    }

    private void Enqueue(Action networkEvent)
    {
        m_networkQueue.Enqueue(networkEvent);
    }

    private void TryConnect(float delaySeconds)
    {
        StartCoroutine(ConnectCoroutine(delaySeconds));
    }

    private IEnumerator ConnectCoroutine(float delaySeconds)
    {
        if (delaySeconds > 0)
        {
            Debug.LogFormat("Next reconnect attempt in {0} seconds", delaySeconds);
            yield return new WaitForSeconds(delaySeconds);
        }
        new Net.TCPClient().Connect(m_hostname, m_port, OnConnected, OnDisconnected, this);
    }

    private void OnConnected(Net.Session session, Exception e)
    {
        Enqueue(() =>
        {
            m_session = session;

            if (session == null)
            {
                // Connect attempt failed
                Debug.LogErrorFormat("Failed to connect to {0}:{1}", m_hostname, m_port);
                TryConnect(m_reconnectDelaySeconds);
                return;
            }

            Debug.LogFormat("Successfully connected to {0}", session);

            AppState appState = FindObjectOfType<AppState>();
            if (appState != null)
            {
                HelloMessage hello = new HelloMessage(clientID, appState.AppTypeIdentifier, "Hello from Unity!");
                session.Send(ref hello);
            }
        });
    }

    private void OnDisconnected(Net.Session session, Exception e)
    {
        Enqueue(() =>
        {
            TryConnect(m_reconnectDelaySeconds);
        });
    }

    [Net.Handler()]
    private void OnUnknownMessage(Net.Session session, string json)
    {
        Debug.LogErrorFormat("Received unknown message from {0}: {1}", session, json);
    }

    [Net.Handler(typeof(HelloMessage))]
    private void OnHelloMessage(Net.Session session, string json)
    {
        // This is just a test message that contains a printable string
        HelloMessage msg = JsonUtility.FromJson<HelloMessage>(json);
        Debug.LogFormat("Got hello message: ({0}) {1}", msg.app_type, msg.message);
    }

    [Net.Handler(typeof(AprilTagDetectionsMessage))]
    private void OnPoseMessage(Net.Session session, string json)
    {
        Enqueue(() =>
        {
            AprilTagDetectionsMessage msg = JsonUtility.FromJson<AprilTagDetectionsMessage>(json);
            Debug.LogFormat("AprilTagDetectionMessage: {0} tags", msg.detections.Length);
            foreach (var receiver in m_receivers)
            {
                receiver.OnAprilTagDetections(msg.detections);
            }
        });
    }

    [Net.Handler(typeof(ModelBasedRegistrationMessage))]
    private void OnModelBasedRegistrationMessage(Net.Session session, string json)
    {
        Enqueue(() =>
        {
            ModelBasedRegistrationMessage msg = JsonUtility.FromJson<ModelBasedRegistrationMessage>(json);
            Debug.LogFormat("ModelBasedRegistrationMessage: received");
            foreach (var receiver in m_receivers)
            {
                receiver.OnModelBasedRegistration(msg.registration_matrix, msg.oakd_controller_positions);
            }
        });
    }

    [Net.Handler(typeof(BoxesMessage))]
    private void OnBoxesMessage(Net.Session session, string json)
    {
        Enqueue(() =>
        {
            BoxesMessage msg = JsonUtility.FromJson<BoxesMessage>(json);
            Debug.LogFormat("BoxesMessage: received");
            foreach (var receiver in m_receivers)
            {
                receiver.OnBoundingBoxes(msg.boxes);
            }
        });
    }

    [Net.Handler(typeof(HMDAvatarPoseMessage))]
    private void OnHMDAvatarPoseMessage(Net.Session session, string json)
    {
        Enqueue(() =>
        {
            HMDAvatarPoseMessage msg = JsonUtility.FromJson<HMDAvatarPoseMessage>(json);
            foreach (var receiver in m_receivers)
            {
                receiver.OnHMDAvatarPose(msg.id, msg.head_pose, msg.left_hand_pose, msg.right_hand_pose);
            }
        });
    }

    [Net.Handler(typeof(HMDLaserFiredMessage))]
    private void OnHMDLaserFiredMessage(Net.Session session, string json)
    {
        Enqueue(() =>
        {
            HMDLaserFiredMessage msg = JsonUtility.FromJson<HMDLaserFiredMessage>(json);
            foreach (var receiver in m_receivers)
            {
                receiver.OnHMDLaserFired(msg.id, msg.pose, msg.fast_beam_mode);
            }
        });
    }
}
