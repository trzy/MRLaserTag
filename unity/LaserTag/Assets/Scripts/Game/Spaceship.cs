using UnityEngine;

public class Spaceship : MonoBehaviour
{
    [SerializeField]
    private GameObject explosionPrefab;

    public float speed = 0;
    public float dieAt = float.PositiveInfinity;

    private Rigidbody m_rb;
    private bool m_dead = false;

    private void Awake()
    {
        m_rb = GetComponentInChildren<Rigidbody>();
    }

    private void Start()
    {
        m_rb.AddForce(transform.forward * speed, ForceMode.VelocityChange);
        m_rb.drag = 0;
    }

    private void FixedUpdate()
    {
        if (transform.position.y <= 0)
        {
            // Must have fallen outside of floor area
            dieAt = Time.time;
            Explode();
        }

        if (Time.time >= dieAt)
        {
            Destroy(gameObject);
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!m_dead && collision.collider.CompareTag("Projectile"))
        {
            // Hit by laser. Enter freefall state.
            m_dead = true;
            m_rb.useGravity = true;
            m_rb.angularVelocity = Random.onUnitSphere * Random.Range(30 * Mathf.Deg2Rad, 10 * 360 * Mathf.Deg2Rad);
            dieAt += 10;    // give us more time to fall

            Transform damageEffects = transform.FindChildRecursive("Damaged");
            if (damageEffects != null)
            {
                damageEffects.gameObject.SetActive(true);
            }
        }
        else if (!collision.collider.CompareTag("Projectile"))
        {
            // Hit something else
            dieAt = Time.time;
            Explode();
        }
    }

    private void Explode()
    {
        if (explosionPrefab != null)
        {
            Instantiate(explosionPrefab, transform.position, Quaternion.identity);
        }
    }
}
