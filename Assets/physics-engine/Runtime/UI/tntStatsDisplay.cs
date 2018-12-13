using UnityEngine;
using System.Collections;
using UnityEngine.UI;

public class tntStatsDisplay : MonoBehaviour 
{
	struct MaxInfo {
		public int maxRigidBodies;
		public int maxArticulations;
		public int maxIslands;

		public void UpdateInfo(MaxInfo other) {
			maxRigidBodies = Mathf.Max (other.maxRigidBodies, maxRigidBodies);
			maxArticulations = Mathf.Max (other.maxArticulations, maxArticulations);
			maxIslands = Mathf.Max (other.maxIslands, maxIslands);
		}
	}

	private tntWorld world;
	private bool looking4world = false;

	private Text[] statsText = null;
	private MaxInfo maxInfo = new MaxInfo ();
	private MaxInfo maxActiveInfo = new MaxInfo ();
	PhysicsStatsInfo statsInfo = new PhysicsStatsInfo ();

	void Start()
	{
		statsText = GetComponentsInChildren<Text> ();
		if (statsText.Length < 5) {
			Debug.Log ("not enough text components found in children! found: " + statsText.Length);
			enabled = false;
			return;
		}

		TryToFindWorld ();

		UpdateStatsText (true);
	}

	void TryToFindWorld () {
		if (world != null)
			return;

		world = GameObject.FindObjectOfType<tntWorld> ();
		if (!looking4world && world == null) {
			looking4world = true;
			StartCoroutine (FindWorld ());
		}
	}

	private IEnumerator FindWorld () {
		var wait = new WaitForSeconds (1);
		while (world == null) {
			yield return wait;
			world = FindObjectOfType<tntWorld> ();
		}
		looking4world = false;
	}

	[ContextMenu("Reset Max Info")]
	public void ResetMaxInfo () {
		maxActiveInfo = new MaxInfo ();
		maxInfo = new MaxInfo ();
	}

	public void Toggle () {
		gameObject.SetActive (!gameObject.activeSelf);
	}	

	public void UpdateStatsText (bool force = false) {
		TryToFindWorld ();

		var info = world == null ? new PhysicsStatsInfo() : world.PhysicsStatsInfo;
		var totalInfo = new MaxInfo {
			maxRigidBodies = info.numSleepRigidBodies,
			maxArticulations = info.numSleepArticulations,
			maxIslands = info.numSleepIslands
		};
		var activeInfo = new MaxInfo {
			maxRigidBodies = info.numActiveRigidBodies,
			maxArticulations = info.numActiveArticulations,
			maxIslands = info.numActiveIslands
		};

		maxActiveInfo.UpdateInfo (activeInfo);
		maxInfo.UpdateInfo (totalInfo);

		if (force || 
			info.numActiveRigidBodies != statsInfo.numActiveRigidBodies ||
			info.numSleepRigidBodies != statsInfo.numSleepRigidBodies) {
			statsText[0].text = string.Format (
				"Rigid Bodies: {0}/{1}  {2}/{3}", activeInfo.maxRigidBodies, totalInfo.maxRigidBodies,
				maxActiveInfo.maxRigidBodies, maxInfo.maxRigidBodies);
		}

		if (force ||
		    info.numActiveArticulations != statsInfo.numActiveArticulations ||
		    info.numSleepArticulations != statsInfo.numSleepArticulations) {
			statsText [1].text = string.Format (
				"Articulations: {0}/{1}  {2}/{3}", activeInfo.maxArticulations, totalInfo.maxArticulations,
				maxActiveInfo.maxArticulations, maxInfo.maxArticulations);
		}

		if (force ||
			info.numActiveIslands != statsInfo.numActiveIslands ||
			info.numSleepIslands != statsInfo.numSleepIslands) {
			statsText [2].text = string.Format (
				"Islands: {0}/{1}  {2}/{3}", activeInfo.maxIslands, totalInfo.maxIslands,
				maxActiveInfo.maxIslands, maxInfo.maxIslands);
		}

		if (force ||
		    info.numArticulationConstraints != statsInfo.numArticulationConstraints) {
			statsText [3].text = string.Format ("A Constraints: {0}", info.numArticulationConstraints);
		}

		if (force ||
			info.numRigidBodyConstraints != statsInfo.numRigidBodyConstraints) {
			statsText [4].text = string.Format ("B Constraints: {0}", info.numRigidBodyConstraints);
		}

		statsInfo = info;
	}

	void Update()
	{
		UpdateStatsText ();
	}
}
