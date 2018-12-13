
using PhysicsAPI;
using System;
using UnityEngine;
using UnityEngine.UI;

public class P2PBreaker : MonoBehaviour
{
    public float m_breakDistance = 0.3f;
    public float m_despawnHeight = -10f;
    private tntArticulationP2PConstraint m_P2PLink;
    private Transform m_transform;
    private Vector3 m_initialPosition;

    // Use this for initialization
    public virtual void Start()
    {
        m_initialPosition = transform.position;
        m_P2PLink = GetComponent<tntArticulationP2PConstraint>();
        m_transform = transform;
    }

    // Update is called once per frame
    void Update()
    {
        if (m_P2PLink == null)
        {
            if (m_transform.position.y < m_despawnHeight)
                Destroy(transform.gameObject);
        } else if (Vector3.Distance(m_initialPosition, m_transform.position) > m_breakDistance)
        {
            Destroy(m_P2PLink.GetComponent<tntArticulationP2PConstraint>());
            m_P2PLink = null;
        }
    }
}
