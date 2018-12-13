using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using PhysicsAPI;

public class tntFPSObjectShooter : MonoBehaviour
{
    public GameObject        Element      = null;
    public float             InitialSpeed = 20.0f;
    public float             MouseSpeed   = 0.3f;
    public float             Scale        = 1.0f;
    public float             Mass         = 1.0f;
    public float             Life         = 10.0f;
    public bool             EnablePanning = true;
    private Vector3          m_v3MousePosition;

	void Start()
    {
	    m_v3MousePosition = Input.mousePosition;
	}
	
	public void ShootObject()
            {
                GameObject newObject = GameObject.Instantiate(Element) as GameObject;
                newObject.transform.position   = transform.position + transform.forward * 10.0f;
                newObject.transform.localScale = new Vector3(Scale, Scale, Scale);
                //tntRigidBody rb = newObject.AddComponent<tntRigidBody>();
                tntRigidBody rb = newObject.GetComponent<tntRigidBody>();
                rb.position = transform.position;
                rb.rotation = transform.rotation;
                //rb.m_mass = Mass;
                //TNT.apSetRigidBodyMass(rb.GetRigidBody(), rb.m_mass);
                rb.linearVelocity = this.transform.forward * InitialSpeed;
                rb.angularVelocity = Vector3.zero;
                rb.m_initialLinearVelocity = rb.linearVelocity;
                rb.m_initialAngularVelocity = rb.angularVelocity;

                //newObject.rigidbody.mass                 = Mass;
                //newObject.rigidbody.solverIterationCount = 255;
                //newObject.rigidbody.AddForce(this.transform.forward * InitialSpeed, ForceMode.VelocityChange);

                DieTimer dieTimer = newObject.AddComponent<DieTimer>() as DieTimer;
                dieTimer.SecondsToDie = Life;
            }

	void Update()
    {
        if(Element != null)
        {
            if(Input.GetKeyDown(KeyCode.Space))
            {
				ShootObject();
            }
        }

        if(EnablePanning && Input.GetMouseButton(0) && Input.GetMouseButtonDown(0) == false)
        {
            this.transform.Rotate      (-(Input.mousePosition.y - m_v3MousePosition.y) * MouseSpeed, 0.0f, 0.0f);
            this.transform.RotateAround(this.transform.position, Vector3.up, (Input.mousePosition.x - m_v3MousePosition.x) * MouseSpeed);
        }

        m_v3MousePosition = Input.mousePosition;
	}
}
