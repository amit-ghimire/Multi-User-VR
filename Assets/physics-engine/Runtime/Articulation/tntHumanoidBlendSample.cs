using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// This is a wrapper script object class for humanoid control params
/// with annotations for its blendspace position
/// </summary>

public class tntHumanoidBlendSample : ScriptableObject {
    public float m_fwdVel;
    public float m_sideVel;
    public float m_turnVel;
    public tntHumanoidControlParams m_paramsScriptObject;

    public tntHumanoidBlendSample Copy() {
        tntHumanoidBlendSample ret = ScriptableObject.CreateInstance<tntHumanoidBlendSample>();
        ret.m_paramsScriptObject = m_paramsScriptObject.Copy();
        ret.m_fwdVel = m_fwdVel;
        ret.m_sideVel = m_sideVel;
        ret.m_turnVel = m_turnVel;
        return ret;
    }
}