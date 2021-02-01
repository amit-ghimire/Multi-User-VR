using UnityEngine;
using UnityEngine.SceneManagement;
using System.IO;

public class PlayerNetwork : MonoBehaviour {

    #region public variables
    public static PlayerNetwork Instance;
    public string PlayerName { get; private set; }
    public GameObject Dummy;
    public GameObject AudioStreamer;
    public GameObject head;
    public GameObject leftHand;
    public GameObject rightHand;
    public bool dummyInstantiate;
    #endregion

    #region private variables
    private PhotonView PhotonView;
    private int PlayersInGame = 0;
    private GameObject gridObject;
    #endregion

    #region Mono Callbacks
    // Use this for initialization
    private void Awake() {
        Instance = this;
        PhotonView = GetComponent<PhotonView>();

        if (PhotonView.isMine)
        {
            PlayerName = PlayerPrefs.GetString("PlayerName");
        }
        SceneManager.sceneLoaded += OnSceneFinishedLoading;
    }
    #endregion

    #region private behaviours
    private void OnSceneFinishedLoading(Scene scene, LoadSceneMode mode)
    {
        if (scene.name == "temple")
        {
            if (PhotonNetwork.isMasterClient)
            {
                MasterLoadedGame();
            }
            else
                NonMasterLoadedGame();
        }
    }

    private void MasterLoadedGame()
    {
        PhotonView.RPC("RPC_LoadedGameScene", PhotonTargets.MasterClient);
        PhotonView.RPC("RPC_LoadGameOthers", PhotonTargets.Others);
    }

    private void NonMasterLoadedGame()
    {
        PhotonView.RPC("RPC_LoadedGameScene", PhotonTargets.MasterClient);
    }
    #endregion

    #region RPCs
    [PunRPC]
    private void RPC_LoadGameOthers()
    {
        PhotonNetwork.LoadLevel(1);
    }

    [PunRPC]
    private void RPC_LoadedGameScene()
    {
        PlayersInGame++;
        if (PlayersInGame == PhotonNetwork.playerList.Length)
        {
            print("All players are in the game scene");
            PhotonView.RPC("RPC_CreatePlayer", PhotonTargets.All);
        }
    }

    [PunRPC]
    private void RPC_CreatePlayer()
    {
        if (dummyInstantiate)
        {
            PhotonNetwork.Instantiate(Dummy.name, HMDManager.Instance.head.transform.position, HMDManager.Instance.head.transform.rotation, 0);
            //PhotonNetwork.Instantiate(AudioStreamer.name, Vector3.zero, Quaternion.identity, 0);
        }
        else
        {
            PhotonNetwork.Instantiate(head.name, HMDManager.Instance.head.transform.position, HMDManager.Instance.head.transform.rotation, 0);
            PhotonNetwork.Instantiate(leftHand.name, HMDManager.Instance.leftHand.transform.position, HMDManager.Instance.leftHand.transform.rotation, 0);
            PhotonNetwork.Instantiate(rightHand.name, HMDManager.Instance.rightHand.transform.position, HMDManager.Instance.leftHand.transform.rotation, 0);
            //PhotonNetwork.Instantiate(AudioStreamer.name, Vector3.zero, Quaternion.identity, 0);
        }
    }
    #endregion
}