using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraDolly : MonoBehaviour
{
    public Vector3 targetCameraPosition = new Vector3(-0.03f, 0.6f, 3.3f);
    public Vector3 targetCameraEuler = new Vector3(10.622f, -21.605f, 0f);
    public float transitionSeconds = 5;
    public KeyCode trigger = KeyCode.L;

    private bool m_transition = false;
    private Vector3 m_startCameraPosition;
    private Quaternion m_startCameraRotation;
    private float m_startTime;

    private void Update()
    {
        if (Input.GetKeyDown(trigger))
        {
            m_transition = true;
            m_startCameraPosition = Camera.main.transform.position;
            m_startCameraRotation = Camera.main.transform.rotation;
            m_startTime = Time.time;
        }

        if (m_transition)
        {
            float t = Tweens.CubicEaseInOut((Time.time - m_startTime) / transitionSeconds);
            Camera.main.transform.position = Vector3.Lerp(m_startCameraPosition, targetCameraPosition, t);
            Camera.main.transform.rotation = Quaternion.Slerp(m_startCameraRotation, Quaternion.Euler(targetCameraEuler), t);  
        }
    }
}
