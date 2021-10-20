using UnityEngine;

public class LaserBeam : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Speed in m/s in slow-beam mode.")]
    private float m_slowSpeed = 15;

    [SerializeField]
    [Tooltip("Speed in m/s in fast-beam mode.")]
    private float m_fastSpeed = 60;

    [SerializeField]
    [Tooltip("How long beam can travel before it times out and destroys itself.")]
    private float m_timeout = 5;

    [Tooltip("When set, activates fast-beam mode. Must be set before Start() method executes.")]
    public bool fastBeamMode = false;

    private float m_dieAt;
    private Rigidbody m_rb;

    public float Speed
    {
        get
        {
            return fastBeamMode ? m_fastSpeed : m_slowSpeed;
        }
    }
    
    private void Start()
    {
        m_dieAt = Time.time + m_timeout;
        m_rb = GetComponentInChildren<Rigidbody>();
        m_rb.velocity = transform.forward * Speed;
    }

    private void Update()
    {
        if (Time.time > m_dieAt)
        {
            Destroy(gameObject);
            return;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        m_rb.velocity = Vector3.zero;
        m_rb.detectCollisions = false;

        // Hide
        foreach (MeshRenderer renderer in GetComponentsInChildren<MeshRenderer>())
        {
            renderer.enabled = false;
        }

        // Disable collider
        foreach (Collider collider in GetComponentsInChildren<Collider>())
        {
            collider.enabled = false;
        }

        // Trigger particles
        Transform particles = transform.FindChildRecursive("Particles");
        if (particles != null)
        {
            particles.gameObject.SetActive(true);
            m_dieAt = Time.time + 1;
        }
    }
}
