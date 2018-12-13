using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicsAPI;
using System;

public class CharacterAngularMomentum : MonoBehaviour {

    public tntBase m_root;
    public Vector3 m_angularMomentum = Vector3.zero;

	// Use this for initialization
	void Start () {
		
	}

    void OnGUI()
    {
        GUI.Label(new Rect(20, 20, 300, 100),
            String.Format("{0,10:0.000000} {1,10:0.000000} {2,10:0.000000}", 
            m_angularMomentum.x, m_angularMomentum.y, m_angularMomentum.z));
    }
	
	// Update is called once per frame
	void Update () {
        if (m_root && m_root.GetBase() != IntPtr.Zero)
        {
            unsafe
            {
                fixed (Vector3* vec = &m_angularMomentum)
                {
                    TNT.apComputeArticulationAngularMomentum(m_root.GetBase(), vec);
                }
            }
        }
	}
}
