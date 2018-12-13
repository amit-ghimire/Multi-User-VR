using UnityEngine;
using System.Collections;

public class DieTimer : MonoBehaviour
{
    public float SecondsToDie = 10.0f;

    float m_fTimer = 0.0f;

	void Start()
    {
	    m_fTimer = 0.0f;
	}
	
	void Update()
    {
        m_fTimer += Time.deltaTime;

	    if(m_fTimer > SecondsToDie)
        {
            Destroy(gameObject);
        }
	}
}
