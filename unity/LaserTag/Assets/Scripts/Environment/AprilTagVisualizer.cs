using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class AprilTagVisualizer : MessageReceivingBehavior
{
    [Serializable]
    private struct AprilTagPrefabDefinition
    {
        public int tagID;
        public GameObject prefab;

        [Tooltip("Set this to true to ignore the tag size received from the server and render using the prefab size.")]
        public bool usePrefabScale;
    }

    [SerializeField]
    [Tooltip("AprilTag prefab by tag ID.")]
    private AprilTagPrefabDefinition[] m_aprilTagPrefabs;

    private struct AprilTag
    {
        public GameObject tagObject;
        public Matrix4x4 pose;

        public AprilTag(GameObject tagObject, Matrix4x4 pose)
        {
            this.tagObject = tagObject;
            this.pose = pose;
        }
    }
    
    private List<AprilTag> m_aprilTags = new List<AprilTag>();
    private IOAKDRegistrationProvider m_registration;

    private void Awake()
    {
        m_registration = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is IOAKDRegistrationProvider).First() as IOAKDRegistrationProvider;
        Debug.Assert(m_registration != null);
    }

    private void FixedUpdate()
    {
        // Update poses
        foreach (AprilTag aprilTag in m_aprilTags)
        {
            SetPose(aprilTag.tagObject, aprilTag.pose);
        }
    }

    private AprilTagPrefabDefinition? FindPrefabDefinition(int tagID)
    {
        foreach (var definition in m_aprilTagPrefabs)
        {
            if (definition.tagID == tagID)
            {
                return definition;
            }
        }
        return null;
    }

    private void SetPose(GameObject tagObject, Matrix4x4 pose)
    {
        pose = m_registration.ConvertAprilTagPoseToHMD(pose);
        tagObject.transform.rotation = pose.Rotation();
        tagObject.transform.position = pose.Translation();
    }

    public override void OnAprilTagDetections(AprilTagDetection[] detections)
    {
        // Remove all existing tags
        foreach (var aprilTag in m_aprilTags)
        {
            Destroy(aprilTag.tagObject);
        }
        m_aprilTags.Clear();

        // Create new tag objects
        foreach (AprilTagDetection detection in detections)
        {
            AprilTagPrefabDefinition? prefabDef = FindPrefabDefinition(detection.tag_id);
            if (!prefabDef.HasValue)
            {
                continue;
            }
            float tagSize = detection.tag_size_cm * 1e-2f;
            GameObject tagObject = Instantiate(prefabDef.Value.prefab);
            tagObject.name = string.Format("AprilTag-{0}", detection.tag_id);
            tagObject.transform.parent = transform;
            if (!prefabDef.Value.usePrefabScale)
            {
                // Use size given to us
                tagObject.transform.localScale = new Vector3(tagSize, tagSize, 0.001f);
            }
            SetPose(tagObject, detection.pose_matrix);
            m_aprilTags.Add(new AprilTag(tagObject, detection.pose_matrix));
        }
    }
}