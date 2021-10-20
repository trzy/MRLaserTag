using System.Collections.Generic;
using UnityEngine;

public class ModelTrackingRegistration : MessageReceivingBehavior, IOAKDRegistrationProvider
{
    [SerializeField]
    [Tooltip("Oculus controller transform.")]
    private Transform m_controller;

    [SerializeField]
    [Tooltip("Prefab to place at each controller sample point in HMD space.")]
    private GameObject m_hmdSamplePrefab;

    [SerializeField]
    [Tooltip("Prefab to place at each controller sample point in OAKD space converted back to HMD space with registration matrix.")]
    private GameObject m_oakdSamplePrefab;

    private bool m_registrationSessionActive = false;
    private bool m_registrationFinished = false;
    private List<GameObject> m_calibrationPoints = new List<GameObject>();
    private float m_deleteCalibrationPointsAt = float.PositiveInfinity;
    private Matrix4x4? m_registrationMatrix;
    private NetworkManager m_network;

    private void Awake()
    {
        m_network = GameObject.FindObjectOfType<NetworkManager>();
        Debug.Assert(m_network != null);

        // Initially disabled, must be explicitly enabled
        enabled = false;
    }

    private void Update()
    {
        if (!m_network.IsConnected)
        {
            return;
        }

        if (Time.time >= m_deleteCalibrationPointsAt)
        {
            // Time to destroy the debug visualization
            foreach (var point in m_calibrationPoints)
            {
                Destroy(point);
            }
            m_calibrationPoints.Clear();
        }

        if (m_registrationFinished)
        {
            // We are finished and currently do not support re-calibration
            return;
        }

        if (OVRInput.GetDown(OVRInput.Button.Two))  // right controller button B: begin/end calibration session
        {
            ModelBasedCalibrationControlMessage msg = new ModelBasedCalibrationControlMessage();
            msg.startCalibration = !m_registrationSessionActive;
            m_registrationSessionActive = !m_registrationSessionActive;
            m_registrationFinished = !m_registrationSessionActive;
            m_network.Send(ref msg);
            Debug.Log("Sent calibration control message");

            if (m_registrationFinished)
            {
                // We are finished, remove calibration points in a few seconds
                m_deleteCalibrationPointsAt = Time.time + 10;
            }
        }

        if (OVRInput.GetDown(OVRInput.Button.One))  // right controller button A: send pose if in a calibration session
        {
            if (m_registrationSessionActive)
            {
                HMDControllerPoseMessage msg = new HMDControllerPoseMessage();
                msg.pose_matrix = m_controller.localToWorldMatrix;
                m_network.Send(ref msg);
                Debug.Log("Sent controller pose message");

                // Plot marker at controller position
                if (m_hmdSamplePrefab != null)
                {
                    GameObject marker = Instantiate(m_hmdSamplePrefab);
                    marker.name = "HMD-Point";
                    marker.transform.parent = transform;
                    marker.transform.position = m_controller.position;
                    m_calibrationPoints.Add(marker);
                }
            }
        }
    }

    public override void OnModelBasedRegistration(Matrix4x4 registrationMatrix, Vector3[] oakdControllerPositions)
    {
        m_registrationMatrix = registrationMatrix;

        // Draw OAKD points
        if (m_oakdSamplePrefab != null)
        {
            foreach (Vector3 controllerPosition in oakdControllerPositions)
            {
                Vector4 position = new Vector4(controllerPosition.x, controllerPosition.y, controllerPosition.z, 1);
                Vector4 hmdPosition = registrationMatrix * position;

                // Plot marker at registered controller position
                GameObject marker = Instantiate(m_oakdSamplePrefab);
                marker.name = "OAKD-Point";
                marker.transform.parent = transform;
                marker.transform.position = new Vector3(hmdPosition.x, hmdPosition.y, hmdPosition.z);
                m_calibrationPoints.Add(marker);
            }
        }
    }

    public void StartRegistration()
    {
        enabled = true;
    }

    public bool IsRegistrationFinished
    {
        get
        {
            return m_registrationFinished;
        }
    }

    public Matrix4x4 ConvertAprilTagPoseToHMD(Matrix4x4 pose)
    {
        // Position from cm -> m
        pose.m03 *= 0.01f;
        pose.m13 *= 0.01f;
        pose.m23 *= 0.01f;

        // Transform to HMD space
        return ConvertPoseToHMD(pose);
    }

    public Matrix4x4 ConvertPoseToHMD(Matrix4x4 pose)
    {
        Matrix4x4 registrationMatrix = m_registrationMatrix.HasValue ? m_registrationMatrix.Value : Matrix4x4.identity;
        return registrationMatrix * pose;
    }

    public Matrix4x4 OAKDCameraPose()
    {
        // registrationMatrix * identity = pose of camera in HMD space
        Matrix4x4 registrationMatrix = m_registrationMatrix.HasValue ? m_registrationMatrix.Value : Matrix4x4.identity;
        return registrationMatrix;
    }
}