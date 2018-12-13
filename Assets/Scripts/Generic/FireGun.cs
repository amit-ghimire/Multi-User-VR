using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FireGun : MonoBehaviour {

    [SerializeField]
    [Tooltip("The bullet prefab to clone")]
    private GameObject bullet;

    [SerializeField]
    [Tooltip("The bullet speed")]
    private float bulletSpeed = 1000f;

    [SerializeField]
    [Tooltip("The bullet life")]
    private float bulletLife = 5f;

    [SerializeField]
    [Tooltip("The bullet spawn position")]
    private GameObject bulletSpawn;

    private VRTK.VRTK_ControllerEvents controllerEvents;
	// Use this for initialization
	void Start () {
        controllerEvents = GetComponent<VRTK.VRTK_ControllerEvents>();
        controllerEvents.TriggerPressed += OnTriggerPressed;
    }

    private void OnTriggerPressed(object sender, VRTK.ControllerInteractionEventArgs e)
    {
        var bulletClone = PhotonNetwork.Instantiate(bullet.name, bulletSpawn.transform.position, bulletSpawn.transform.rotation, 0) as GameObject;
        bulletClone.SetActive(true);
        Rigidbody rb = bulletClone.GetComponent<Rigidbody>();
        rb.AddForce(-bulletSpawn.transform.forward * bulletSpeed);
        Destroy(bulletClone, bulletLife);
    }
}
