using System.Linq;
using UnityEngine;

public class AvatarPoseTransmitter : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Object whose transform is the head pose (i.e., the camera node).")]
    private Transform m_headPoseProvider;

    [SerializeField]
    [Tooltip("Object whose transform is the left hand pose (i.e., the left controller).")]
    private Transform m_leftHandPoseDriver;

    [SerializeField]
    [Tooltip("Object whose transform is the right hand pose (i.e., the right controlller).")]
    private Transform m_rightHandPoseDriver;

    private NetworkManager m_network;
    private int m_frameNumber = 0;

    private void Awake()
    {
        m_network = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is NetworkManager).First() as NetworkManager;
        Debug.Assert(m_network != null);
    }

    private void FixedUpdate()
    {
        m_frameNumber += 1;
        if ((m_frameNumber & 1) == 0)
        {
            // Send every other physics frame (~25Hz)
            return;
        }
        HMDAvatarPoseMessage msg = new HMDAvatarPoseMessage(m_network.clientID, m_headPoseProvider.localToWorldMatrix, m_leftHandPoseDriver.localToWorldMatrix, m_rightHandPoseDriver.localToWorldMatrix);
        m_network.Send(ref msg);
    }
}
