using System.Linq;
using UnityEngine;

public class LaserEmitter : MessageReceivingBehavior
{
    [SerializeField]
    [Tooltip("Prefab for the laser beam, with forward being the direction of travel.")]
    private LaserBeam m_laserBeamPrefab;

    [SerializeField]
    [Tooltip("True if this laser gun responds to controller input, false if it should process network messages only.")]
    private bool m_playerControlled = true;

    private NetworkManager m_network;

    private void Awake()
    {
        m_network = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is NetworkManager).First() as NetworkManager;
        Debug.Assert(m_network != null);
    }

    private void Update()
    {
        // This pathway is for use on HMDs for firing lasers where the script is attached to Oculus controller anchor point
        if (m_playerControlled && OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger, OVRInput.Controller.RTouch))
        {
            LaserBeam laser = Instantiate(m_laserBeamPrefab, transform.position, transform.rotation) as LaserBeam;
            if (OVRInput.Get(OVRInput.Button.One))
            {
                // When button A is held down, switch to a fast beam mode (for shooting down spaceships)
                laser.fastBeamMode = true;
            }
            HMDLaserFiredMessage msg = new HMDLaserFiredMessage(m_network.clientID, transform.localToWorldMatrix, laser.fastBeamMode);
            m_network.Send(ref msg);
        }
    }

    public override void OnHMDLaserFired(string clientID, Matrix4x4 pose, bool fastBeamMode)
    {
        // This pathway is for use on avatars representing other players and, because the pose is transmitted, can be attached to any singular game object
        if (!m_playerControlled)
        {
            LaserBeam laser = Instantiate(m_laserBeamPrefab, pose.Translation(), pose.Rotation()) as LaserBeam;
            laser.fastBeamMode = fastBeamMode;
        }
    }
}
