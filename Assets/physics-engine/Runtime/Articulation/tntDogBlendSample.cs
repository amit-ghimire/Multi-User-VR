using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is a wrapper script object class for humanoid control params
/// with annotations for its blendspace position
/// </summary>

public class tntDogBlendSample : ScriptableObject {
    public float m_fwdVel;
    public float m_sideVel;
    public float m_turnVel;
    public tntDogControlParams m_paramsScriptObject;

    public tntDogBlendSample Copy() {
        tntDogBlendSample ret = ScriptableObject.CreateInstance<tntDogBlendSample>();
        ret.m_paramsScriptObject = m_paramsScriptObject.Copy();
        ret.m_fwdVel = m_fwdVel;
        ret.m_sideVel = m_sideVel;
        ret.m_turnVel = m_turnVel;
        return ret;
    }
}