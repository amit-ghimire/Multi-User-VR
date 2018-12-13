using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VRTK;

public class SpawnCubes : MonoBehaviour {

    private VRTK_ControllerEvents controllerEvents;
    public GameObject cubePrefab;

    private void Awake()
    {
        controllerEvents = GetComponent<VRTK_ControllerEvents>();
        controllerEvents.TriggerPressed += OnTriggerPressed;
    }
	
    private void OnTriggerPressed(object sender, ControllerInteractionEventArgs e)
    {
        var cube = PhotonNetwork.Instantiate(cubePrefab.name, gameObject.transform.position, gameObject.transform.rotation,0);
    }
}
