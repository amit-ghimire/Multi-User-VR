using UnityEngine;
using System.Collections;
using PhysicsAPI;

public class ContactVisualizer : MonoBehaviour, ITntCollisionListener
{
	// Use this for initialization
	void Start () {
        tntRigidBody rigidBody = GetComponent<tntRigidBody>();
        if (rigidBody != null)
        {
            rigidBody.AddListener(this);
        }
        else
        {
            tntBase theBase = GetComponent<tntBase>();
            if (theBase != null)
            {
                theBase.AddListener(this);
            }
        }
    }

    void OnDestroy()
    {

    }

    // Update is called once per frame
    void Update () {
	}

	private void PrintContacts(ExtendedCollision ec)
	{
		for (int i = 0; i < ec.contacts.Length; ++i)
		{
			Debug.Log ("Contact point #" + i + ": position:" + ec.contacts [i].point + " normal:" + ec.contacts [i].normal);
		}
	}

    public void OnTntCollisionEnter(Collision collision) {
		ExtendedCollision ec = (ExtendedCollision)collision;
		//Debug.Log("Collision Enter: go(" + ec.gameObject.name + ") rb(" + ec.rigidbody.name + ")");
		//PrintContacts(ec);
	}

	public void OnTntCollisionStay(Collision collision) {
		ExtendedCollision ec = (ExtendedCollision)collision;
		//Debug.Log("Collision Stay: go(" + ec.gameObject.name + ") rb(" + ec.rigidbody.name + ")");
		//PrintContacts(ec);
#if UNITY_EDITOR
		for (int i = 0; i < ec.contacts.Length; ++i)
		{
			ExtendedContactPoint cp = ec.contacts[i];
			DrawArrow.ForDebug(cp.point, ec.impulse * 2, Color.yellow, 0.02f,20, 1);
		}
#endif
	}

	public void OnTntCollisionExit(Collision collision) {
		ExtendedCollision ec = (ExtendedCollision)collision;
		//Debug.Log("Collision Leave:");
		//PrintContacts(ec);
	}
}