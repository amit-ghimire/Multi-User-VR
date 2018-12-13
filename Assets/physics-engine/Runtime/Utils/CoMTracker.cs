using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using PhysicsAPI;

public class CoMTracker : MonoBehaviour {

    public ComPositionInRoot m_comPos = null;
    public bool m_trackVerticalMovement = false;

    private Vector3 m_offset = Vector3.zero;

    // Use this for initialization
    void Start ()
    {
        if (m_comPos == null)
            return;
        
        m_offset = transform.position - m_comPos.transform.position;
        if (!m_trackVerticalMovement)
            m_offset.y = 0;
    }
	
	// Update is called once per frame
	void Update () {
        if (m_comPos == null)
            return;

        Vector3 newPos;
        {
            Vector3 com = m_comPos.transform.position;
            newPos = m_offset + com;
        }

        if (!m_trackVerticalMovement)
            newPos.y = transform.position.y;

        transform.position = newPos;
        transform.LookAt(m_comPos.transform.position);
    }
}
