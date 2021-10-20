using System.Collections;
using System.Linq;
using UnityEngine;

public class AppState : MonoBehaviour
{
    public enum ClientType
    {
        HMD,
        Spectator
    }

    [Tooltip("What sort of client this is.")]
    public ClientType clientType = ClientType.HMD;

    private IOAKDRegistrationProvider m_registration;

    public string AppTypeIdentifier
    {
        get
        {
            return clientType.ToString();
        }
    }

    private void Awake()
    {
        m_registration = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is IOAKDRegistrationProvider).First() as IOAKDRegistrationProvider;
        Debug.Assert(m_registration != null);
    }

    private void Start()
    {
        StartCoroutine(RunApp());
    }

    private void FixedUpdate()
    {
        if (clientType == ClientType.Spectator)
        {
            // Position Unity camera at location of OAKD camera
            Camera.main.transform.position = m_registration.OAKDCameraPose().Translation();
            Camera.main.transform.rotation = m_registration.OAKDCameraPose().Rotation();
        }
    }

    private IEnumerator RunApp()
    {
        if (clientType == ClientType.HMD)
        {
            // Perform registration
            m_registration.StartRegistration();
            yield return new WaitUntil(() => m_registration.IsRegistrationFinished);
        }

        // TODO: Would be better if client explicitly queried walls and objecta
        // so that there is no chance of them being sent before registration
    }
}
