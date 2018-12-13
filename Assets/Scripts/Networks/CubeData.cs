using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CubeData: Photon.PunBehaviour {

    private Rigidbody rigidBody;
    private PhotonView PhotonView;

    private void Awake()
    {
        rigidBody = GetComponent<Rigidbody>();
        PhotonView = GetComponent<PhotonView>();
    }


    public void OnPhotonSerializeView(PhotonStream stream, PhotonMessageInfo info)
    {
        if (stream.isWriting)
        {
            //Send Data to other Players
            stream.SendNext(transform.position);
            stream.SendNext(transform.rotation);
            stream.SendNext(rigidBody.velocity);
            stream.SendNext(rigidBody.angularVelocity);
        }
        else
        {
            this.transform.position = (Vector3)stream.ReceiveNext();
            this.transform.rotation = (Quaternion)stream.ReceiveNext();
            this.rigidBody.velocity = (Vector3)stream.ReceiveNext();
            this.rigidBody.angularVelocity = (Vector3)stream.ReceiveNext();
        }

    }
}
