#if !UNITY_EDITOR && UNITY_PS4
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using Photon.Voice;
using Sony.NP;
public class PS4AudioOut : Photon.Voice.IAudioOut
{
    const string libName = "PhotonVoiceAudioOutputPlugin";
    [DllImport(libName)]
    private static extern IntPtr egpvopen(int userID, uint granularity, bool stereo); // open a new audio output port
    [DllImport(libName)]
    private static extern void egpvclose(IntPtr pVoiceAudio); // close the specified port
    [DllImport(libName)]
    private static extern int egpvplay(IntPtr pVoiceAudio, IntPtr pData); // play the specified audio data on the specified port
    [DllImport(libName)]
    private static extern int egpvgetHandle(IntPtr pVoiceAudio); // returns a negative value if egpvopen has failed or the handle that is passed to the Sony APIs to identify the port if egpvopen has been successful
    [DllImport(libName)]
    private static extern bool egpvgetHeadphonesConnected(IntPtr pVoiceAudio);  // returns true, if headphones are plugged in for the specified output port, false otherwise


//    private AudioStreamPlayer player;
    private static IntPtr pPhotonVoiceAudioOutput;
    private static Dictionary<PS4AudioOut, Queue<float>> frameBuf;
    private static readonly object locker = new object();
    public const int FRAME_POOL_CAPACITY = 50;
    public const int GRANULARITY = 256; // The number of samples per channel of the audio data that will be passed to the Sony APIs at one time. The values that can be specified are 256, 512, 768, 1024, 1280, 1536, 1792 and 2048.
    private const int DEST_SAMPLE_RATE = 48000; // 48000 is the only value that is supported by the Sony APIs, so don't change it!
    private Photon.Voice.PrimitiveArrayPool<float> instanceFramePool;
    private static PrimitiveArrayPool<float> staticFramePool;
    private static Thread playThread;
    private static bool playThreadShouldTerminate;
    private int channelCount;
    private IAudioOut speakersOut; // used when headphones not connected

    public PS4AudioOut(Func<IAudioOut> speakersOutFactory)
    {
        this.speakersOut = speakersOutFactory();
    }

    public bool IsPlaying
    {
        get; private set;
    }

    public void Start(int frequency, int channels, int frameSamplesPerChannel, int playDelayMs)
    {
        const int CHANNEL_COUNT = 2; // stereo
        instanceFramePool = new PrimitiveArrayPool<float>(FRAME_POOL_CAPACITY, "PS4AudioOut");
        instanceFramePool.Init(frameSamplesPerChannel * DEST_SAMPLE_RATE / frequency * CHANNEL_COUNT);
        channelCount = channels;
        lock (locker)
        {
            if (frameBuf == null)
            {
                staticFramePool = new PrimitiveArrayPool<float>(FRAME_POOL_CAPACITY, "PS4AudioOut");
                staticFramePool.Init(GRANULARITY * CHANNEL_COUNT);
                int userID = PhotonVoiceSettings.Instance.PS4UserID;
                if (userID == 0)
                {
                    UserProfiles.LocalUsers localUsers = new UserProfiles.LocalUsers();
                    UserProfiles.GetLocalUsers(localUsers);
                    userID = localUsers.LocalUsersIds[0].UserId.Id;
                }
                pPhotonVoiceAudioOutput = egpvopen(userID, GRANULARITY, true);
                playThreadShouldTerminate = false;
                playThread = new Thread(Play);
                playThread.Name = "photon voice audio output thread";
                playThread.IsBackground = true;
                playThread.Start();
                frameBuf = new Dictionary<PS4AudioOut, Queue<float>>();
            }

            frameBuf.Add(this, new Queue<float>());
        }

        this.speakersOut.Start(frequency, channels, frameSamplesPerChannel, playDelayMs);
    }
    public void Stop()
    {
        if (frameBuf == null)
            return;
        lock (locker)
        {
            frameBuf.Remove(this);
            if (frameBuf.Count > 0)
                return;
            frameBuf = null;
        }
        playThreadShouldTerminate = true;
        instanceFramePool = null;
        egpvclose(pPhotonVoiceAudioOutput);
        pPhotonVoiceAudioOutput = IntPtr.Zero;

        this.speakersOut.Stop();
    }

    public void Service()
    {
        // TODO: check if headphones connected?
        this.speakersOut.Service();
    }

    public void OnAudioFrame(float[] frame)
    {
        bool headphonesConnected = egpvgetHeadphonesConnected(pPhotonVoiceAudioOutput);
        if (headphonesConnected)
        {
            float[] frameCopy = instanceFramePool.AcquireOrCreate();
            AudioUtil.Resample(frame, frameCopy, frameCopy.Length / 2 * channelCount, channelCount);
            float[] frameCopy2 = instanceFramePool.AcquireOrCreate();
            AudioUtil.ForceToStereo(frameCopy, frameCopy2, channelCount);
            instanceFramePool.Release(frameCopy);
            lock (locker)
            {
                Queue<float> instanceFrameBuf;
                frameBuf.TryGetValue(this, out instanceFrameBuf);
                if (instanceFrameBuf != null)
                    for (int i = 0; i < frameCopy2.Length; ++i)
                        instanceFrameBuf.Enqueue(frameCopy2[i]);
            }
            instanceFramePool.Release(frameCopy2);
        }
        else
        {
            this.speakersOut.OnAudioFrame(frame);
        }
    }

    internal static void Play()
    {
        while (!playThreadShouldTerminate)
        {
            if (frameBuf != null)
            {
                float[] frameMix = null;
                lock (locker)
                {
                    foreach (var it in frameBuf)
                    {
                        if (it.Value.Count >= GRANULARITY * 2)
                        {
                            if (frameMix == null)
                            {
                                frameMix = staticFramePool.AcquireOrCreate();
                                for (int i = 0; i < GRANULARITY * 2; ++i)
                                    frameMix[i] = it.Value.Dequeue();
                            }
                            else
                                for (int i = 0; i < GRANULARITY * 2; ++i)
                                    frameMix[i] += it.Value.Dequeue();
                            it.Key.IsPlaying = true;
                        }
                    }
                }
                if (frameMix != null)
                {
                    unsafe
                    {
                        fixed (float* pArray = frameMix)
                            egpvplay(pPhotonVoiceAudioOutput, new IntPtr(pArray));
                    }
                    staticFramePool.Release(frameMix);
                    lock (locker)
                        foreach (var it in frameBuf)
                            it.Key.IsPlaying = false;
                }
            }
        }
    }

    public int CurrentBufferLag { get { return 0; } }

}
#endif