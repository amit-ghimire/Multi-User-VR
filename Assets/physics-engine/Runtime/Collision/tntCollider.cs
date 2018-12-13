using UnityEngine;
using System.Collections;

/**
 * @brief Base class for wrapping collider types unavailable in Unity but provided by APE. Meant to be used along with Unity Collider objects.
 */
public abstract class tntCollider : MonoBehaviour
{
	public bool IsTrigger;
}
