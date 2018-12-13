using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveOnDrag : MonoBehaviour {

    public Vector3 gameObjectScreenPoint;
    public Vector3 mousePreviousPoint;
    public Vector3 mouseCurrentPoint;
    public Vector3 force;
    public Vector3 objectCurrentPosition;
    public Vector3 objecttargetPosition;
    public float topSpeed = 20.0f;

    private void OnMouseDown()
    {
        gameObjectScreenPoint = Camera.main.WorldToScreenPoint(gameObject.transform.position);
        mousePreviousPoint = new Vector3(Input.mousePosition.x, gameObjectScreenPoint.z, Input.mousePosition.y);
    }

    private void OnMouseDrag()
    {
        mouseCurrentPoint = new Vector3(Input.mousePosition.x, gameObjectScreenPoint.z, Input.mousePosition.y);
        force = mouseCurrentPoint - mousePreviousPoint;
        mousePreviousPoint = mouseCurrentPoint;
    }

    private void OnMouseUp()
    {
        if (GetComponent<Rigidbody>().velocity.magnitude > topSpeed)
            force = GetComponent<Rigidbody>().velocity.normalized * topSpeed;
    }

    public void FixedUpdate()
    {
        GetComponent<Rigidbody>().velocity = force;
    }
}
