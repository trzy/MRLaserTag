using System.Collections.Generic;
using UnityEngine;

public class Avatars : MessageReceivingBehavior
{
    [SerializeField]
    [Tooltip("Avatar prefab.")]
    private GameObject m_avatarPrefab;

    private Dictionary<string, GameObject> m_avatarByClientID = new Dictionary<string, GameObject>();
    private float m_torsoOffset = 0;    // measured the first time we instantiate an avatar

    public override void OnHMDAvatarPose(string clientID, Matrix4x4 headPose, Matrix4x4 leftHandPose, Matrix4x4 rightHandPose)
    {
        GameObject avatar;
        if (!m_avatarByClientID.TryGetValue(clientID, out avatar))
        {
            avatar = CreateAvatar(clientID);
            m_avatarByClientID.Add(clientID, avatar);
        }

        // Find each part
        Transform head = avatar.transform.FindChildRecursive("Head");
        Transform torso = avatar.transform.FindChildRecursive("Torso");
        Transform leftHand = avatar.transform.FindChildRecursive("LeftHand");
        Transform rightHand = avatar.transform.FindChildRecursive("RightHand");

        // Set poses
        if (head != null)
        {
            head.position = headPose.Translation();
            head.rotation = headPose.Rotation();
        }
        
        if (torso != null)
        {
            torso.position = headPose.Translation() + Vector3.up * m_torsoOffset;
            torso.rotation = Quaternion.LookRotation(headPose.Forward().XZProjected(), Vector3.up);
        }

        if (leftHand != null)
        {
            leftHand.position = leftHandPose.Translation();
            leftHand.rotation = leftHandPose.Rotation();
        }

        if (rightHand != null)
        {
            rightHand.position = rightHandPose.Translation();
            rightHand.rotation = rightHandPose.Rotation();
        }
    }

    private GameObject CreateAvatar(string clientID)
    {
        GameObject avatar = Instantiate(m_avatarPrefab);
        Transform head = avatar.transform.FindChildRecursive("Head");
        Transform torso = avatar.transform.FindChildRecursive("Torso");
        if (head != null && torso != null)
        {
            m_torsoOffset = torso.localPosition.y - head.localPosition.y;
        }
        avatar.name = "Avatar-" + clientID;
        avatar.transform.parent = transform;
        return avatar;
    }
}
