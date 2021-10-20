using System.Linq;
using UnityEngine;

public class SpaceshipEmitter : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Prefabs of ship to use, randomly chosen.")]
    private Spaceship[] m_shipPrefabs;

    [SerializeField]
    private float m_emissionDelayMin = 1;
    
    [SerializeField]
    private float m_emissionDelayMax = 5;

    [SerializeField]
    private int m_maxShips = 8;

    [SerializeField]
    private float m_speedMin = 3;

    [SerializeField]
    private float m_speedMax = 12;

    [SerializeField]
    private float m_altitudeMin = 25;

    [SerializeField]
    private float m_altitudeMax = 45;

    [SerializeField]
    private float m_circleRadius = 200;

    private float m_nextEmissionTime = 0;
    private Spaceship[] m_ships;
    private System.Random m_rnd = new System.Random();

    private void Awake()
    {
         m_ships = new Spaceship[m_maxShips];
    }

    void Update()
    {
        // Emit ships
        float now = Time.time;
        int numActiveShips = m_ships.Sum(ship => ship == null ? 0 : 1);
        if (numActiveShips < m_ships.Length && now >= m_nextEmissionTime)
        {
            EmitShip(now);
            m_nextEmissionTime = now + Random.Range(m_emissionDelayMin, m_emissionDelayMax);
        }

        // Update existing ships
        for (int i = 0; i < m_ships.Length; i++)
        {
            UpdateShip(i, now);
        }
    }

    private void EmitShip(float now)
    {
        // Find first free slot
        int i = 0;
        for (; i < m_ships.Length; i++)
        {
            if (m_ships[i] == null)
            {
                break;
            }
        }

        if (i >= m_ships.Length)
        {
            // No free ships
            return;
        }

        // Compute altitude
        float altitudeStep = (m_altitudeMax - m_altitudeMin) / (float)m_ships.Length;
        float altitude = m_altitudeMin + altitudeStep * (float)i;

        // Select endpoints
        Vector3 startPosition = Vector3.zero;
        Vector3 endPosition = Vector3.zero;
        ThroughCenter(out startPosition, out endPosition, altitude);
        
        // Select speed
        float speed = Random.Range(m_speedMin, m_speedMax);

        // Select a ship
        Spaceship prefab = m_shipPrefabs[m_rnd.Next(m_shipPrefabs.Length)];

        // Create ship
        Vector3 path = endPosition - startPosition;
        Spaceship ship = Instantiate(prefab);
        ship.dieAt = now + Vector3.Distance(endPosition, startPosition) / speed;
        ship.transform.SetPositionAndRotation(startPosition, Quaternion.LookRotation(path));
        ship.speed = speed;
        m_ships[i] = ship;
    }

    // Choose two opposite points on a circle that is centered on (0,0,0)
    private void ThroughCenter(out Vector3 startPosition, out Vector3 endPosition, float altitude)
    {
        float radius = m_circleRadius;
        float angle1 = Random.Range(0.0f, 360.0f);
        float angle2 = angle1 + 180;
        startPosition = new Vector3(radius * Mathf.Cos(angle1 * Mathf.Deg2Rad), altitude, radius * Mathf.Sin(angle1 * Mathf.Deg2Rad));
        endPosition = new Vector3(radius * Mathf.Cos(angle2 * Mathf.Deg2Rad), altitude, radius * Mathf.Sin(angle2 * Mathf.Deg2Rad));
    }

    private void UpdateShip(int idx, float now)
    {
        if (m_ships[idx] == null)
        {
            // Not active
            return;
        }
        
        if (now >= m_ships[idx].dieAt)
        {
            m_ships[idx] = null;
        }
    }
}
