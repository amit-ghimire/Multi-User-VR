using UnityEngine;
using System;

public abstract class tntKernelResource : ScriptableObject
{
    protected abstract void InitKernelObjectFromAsset();                        // this asset contents -> kernel object
    protected abstract void DeinitKernelObject();                               // kernel object clean-up
    protected abstract bool IsKernelObjectInitialized
    {
        get;
    }
    public abstract IntPtr KernelHandle
    {
        get;
    }
    public abstract void ReloadFromKernel();

    protected virtual void Awake()
    {
        Debug.Assert(!IsKernelObjectInitialized);
        InitKernelObjectFromAsset();
    }
    protected virtual void OnEnable()
    {
        if (!IsKernelObjectInitialized)
            InitKernelObjectFromAsset();
    }
    protected virtual void OnDisable()
    {
        Debug.Assert(IsKernelObjectInitialized);
        DeinitKernelObject();
    }
    protected virtual void OnDestroy()
    {
        if (IsKernelObjectInitialized)
            DeinitKernelObject();
    }
    ~tntKernelResource()
    {
        if (IsKernelObjectInitialized)
            DeinitKernelObject();
    }
    // ...
}