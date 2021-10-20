using System.Collections;
using UnityEngine;

public class AutoDestroy : MonoBehaviour
{
    [SerializeField]
    [Tooltip("Automatically destroy self after this many seconds.")]
    private float m_timeout = 5;

    private IEnumerator Start()
    {
        yield return new WaitForSeconds(m_timeout);
        Destroy(gameObject);
    }
}
