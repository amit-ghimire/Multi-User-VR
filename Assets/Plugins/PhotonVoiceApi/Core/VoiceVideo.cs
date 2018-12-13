// -----------------------------------------------------------------------
// <copyright file="VoiceVideo.cs" company="Exit Games GmbH">
//   Photon Voice API Framework for Photon - Copyright (C) 2017 Exit Games GmbH
// </copyright>
// <summary>
//   Photon data streaming support.
// </summary>
// <author>developer@photonengine.com</author>
// ----------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Threading;
#if NETFX_CORE
using Windows.System.Threading;
#endif

namespace ExitGames.Client.Photon.Voice
{
    public class LocalVoiceVideo : LocalVoice
    {
        internal LocalVoiceVideo(VoiceClient voiceClient, IEncoder encoder, byte id, VoiceInfo voiceInfo, int channelId) : base(voiceClient, encoder, id, voiceInfo, channelId)
        {
            if (this.encoder == null)
            {
                this.encoder = VoiceCodec.CreateDefaultEncoder(voiceInfo, this);
            }
        }

        bool imageEncodeThreadStarted;
        Queue<ImageBufferNative> pushImageQueue = new Queue<ImageBufferNative>();
        AutoResetEvent pushImageQueueReady = new AutoResetEvent(false);
        public int PushImageQueueCount { get { return pushImageQueue.Count; } }
        public void PushImageAsync(ImageBufferNative buf)
        {
            if (disposed) return;

            if (!imageEncodeThreadStarted)
            {
                voiceClient.frontend.LogInfo(LogPrefix + ": Starting image encode thread");
#if NETFX_CORE
                ThreadPool.RunAsync((x) =>
                {
                    PushImageAsyncThread();
                });
#else
                var t = new Thread(PushImageAsyncThread);
                t.Name = LogPrefix + " image encode";
                t.Start();
#endif
                imageEncodeThreadStarted = true;
            }

            lock (pushImageQueue)
            {
                pushImageQueue.Enqueue(buf);
            }
            pushImageQueueReady.Set();
        }
        bool exitThread = false;
        private void PushImageAsyncThread()
        {
            try
            {
                while (!exitThread)
                {
                    pushImageQueueReady.WaitOne(); // Wait until data is pushed to the queue or Dispose signals.

                    while (true) // Dequeue and process while the queue is not empty.
                    {
                        if (exitThread) break; // early exit to save few resources

                        ImageBufferNative b = null;
                        lock (pushImageQueue)
                        {
                            if (pushImageQueue.Count > 0)
                            {
                                b = pushImageQueue.Dequeue();
                            }
                        }

                        if (b != null)
                        {
                            PushImage(b.Planes, b.Info.Width, b.Info.Height, b.Info.Stride, b.Info.Format, b.Info.Rotation, b.Info.Flip);
                            b.Release();
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
            catch (Exception e)
            {
                voiceClient.frontend.LogError(LogPrefix + ": Exception in encode thread: " + e);
                throw e;
            }
            finally
            {
                lock (disposeLock)
                {
                    disposed = true;
                }
                lock (pushImageQueue)
                {
                    while (pushImageQueue.Count > 0)
                    {
                        pushImageQueue.Dequeue().Dispose();
                    }
                }

#if NETFX_CORE
                pushImageQueueReady.Dispose();                
#else
                pushImageQueueReady.Close();
#endif

                voiceClient.frontend.LogInfo(LogPrefix + ": Exiting image encode thread");
            }
        }


        public void PushImage(IntPtr[] buf, int width, int height, int[] stride, ImageFormat imageFormat, Rotation rotation = Rotation.Rotate0, Flip flip = Flip.None)
        {
            if (this.voiceClient.frontend.IsChannelJoined(this.channelId) && this.Transmit)
            {
                if (this.encoder is IEncoderNativeImageDirect)
                {
                    lock (disposeLock)
                    {
                        if (!disposed)
                        {
                            foreach (var compressed in ((IEncoderNativeImageDirect)this.encoder).EncodeAndGetOutput(buf, width, height, stride, imageFormat, rotation, flip))
                            {
                                if (compressed.Count != 0)
                                {
                                    sendFrame(compressed);
                                }
                            }
                        }
                    }
                }
                else
                {
                    throw new Exception(LogPrefix + ": PushImage() called on encoder of unsupported type " + (this.encoder == null ? "null" : this.encoder.GetType().ToString()));
                }
            }
        }

        public override void Dispose()
        {
			exitThread = true;
            lock (disposeLock)
            {
                if (!disposed)
                {
                    // objects used for async push disposed in encode thread 'finally'
                    base.Dispose();
                    pushImageQueueReady.Set(); // let worker exit
                }
            }
            base.Dispose();
        }
    }
}