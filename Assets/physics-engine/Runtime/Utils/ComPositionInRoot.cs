using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ComPositionInRoot : MonoBehaviour {
    public tntBase m_articulation = null;
    public bool m_disableVerticalMotion = true;

    // Use this for initialization
    void Start () {
        
    }
    
    // Update is called once per frame
    void Update () {
        if (m_articulation && m_articulation.m_base != IntPtr.Zero)
        {
            Vector3 com = m_articulation.ComputeArticulationCoMPosition();
            if (m_disableVerticalMotion)
                com.y = transform.position.y;
            transform.position = com;
        }
    }
}
