using UnityEngine;

/// <summary>
/// Component representing remote audio stream in local scene. Automatically attached to the PUN object which owner's instance has streaming Recorder attached.
/// </summary>
[RequireComponent(typeof (AudioSource))]
[DisallowMultipleComponent]
[AddComponentMenu("Photon Voice/Photon Voice Speaker")]
//[HelpURL("https://doc.photonengine.com/en-us/voice/current/getting-started/voice-for-pun#the__audio_source__prefab")]
public class PhotonVoiceSpeaker : Photon.MonoBehaviour
{
    private ExitGames.Client.Photon.Voice.IAudioOut player;
    private bool started;

    /// <summary>Time when last audio packet was received for the speaker.</summary>
    public long LastRecvTime { get; private set; }

    /// <summary>Is the speaker playing right now.</summary>
    public bool IsPlaying
    {
        get { return this.player.IsPlaying; }
    }

    /// <summary>Smoothed difference between (jittering) stream and (clock-driven) player.</summary>
    public int CurrentBufferLag { get { return this.player.CurrentBufferLag; } }

    /// <summary>Is the speaker linked to the remote voice (info available and streaming is possible).</summary>
    public bool IsVoiceLinked { get { return this.player != null && this.started; } }

    void Awake()
    {
#if !UNITY_EDITOR && UNITY_PS4
        this.player = new PS4AudioOut(() => new AudioStreamPlayer(GetComponent<AudioSource>(), "PUNVoice: PhotonVoiceSpeaker:", PhotonVoiceSettings.Instance.DebugInfo));
#else
        this.player = new AudioStreamPlayer(GetComponent<AudioSource>(), "PUNVoice: PhotonVoiceSpeaker:", PhotonVoiceSettings.Instance.DebugInfo);
#endif
        PhotonVoiceNetwork.LinkSpeakerToRemoteVoice(this);
    }

    // initializes the speaker with remote voice info
    internal void OnVoiceLinked(int frequency, int channels, int frameSamplesPerChannel, int playDelayMs)
    {
        this.player.Start(frequency, channels, frameSamplesPerChannel, playDelayMs);
        started = true;
    }

    internal void OnVoiceUnlinked()
    {
        Cleanup();
    }

    void Update()
    {
        this.player.Service();
    }

    void OnDestroy()
    {
        PhotonVoiceNetwork.UnlinkSpeakerFromRemoteVoice(this);
        Cleanup();
    }

    void OnApplicationQuit()
    {
        Cleanup();
    }

    void Cleanup()
    {
        this.player.Stop();
        started = false;
    }

    internal void OnAudioFrame(float[] frame)
    {
        // Set last time we got something
        this.LastRecvTime = System.DateTime.Now.Ticks;
        
        this.player.OnAudioFrame(frame);
    }
}
