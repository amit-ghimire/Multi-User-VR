using UnityEngine;
using System.Collections;
using PhysicsAPI;
using System.Collections.Generic;

[System.Serializable]
public class PDParamSet : ScriptableObject
{
    /// <summary>
    /// PDParamSet encapsulates the set of PD parameters of all bones in a humanoid rig
    /// </summary>
    //-------------------Dynamic PD Parameters----------------- 
    // Enums
    public const int PD_CONTROLLED = 0;
    public const int PD_KP = 1;
    public const int PD_KPMOD = 2;
    public const int PD_KD = 3;
    public const int PD_KDMOD = 4;
    public const int PD_MAX_ABS_TORQUE = 5;
    public const int PD_SCALE = 6;
    public const int PD_REL = 9;

    public int m_numLinks;
    public float[] m_pdParams;
    public string[] m_linkNames;

    public PDParamSet Copy ()
    {
        PDParamSet ret = ScriptableObject.CreateInstance<PDParamSet>();
        ret.m_pdParams = new float[m_pdParams.Length];
        ret.m_linkNames = new string[m_linkNames.Length];
        ret.m_numLinks = m_numLinks;
        m_pdParams.CopyTo(ret.m_pdParams, 0);
        m_linkNames.CopyTo(ret.m_linkNames, 0);
        return ret;
    }

    public void UpdatePdParams (int jointIndex, float kp, float kd, float maxTorque, float kpMod = 1, float kdMod = 1) {
        int i = (jointIndex + 1) * (PD_REL + 1);
        m_pdParams[i + PD_KP] = kp;
        m_pdParams[i + PD_KPMOD] = kpMod;
        m_pdParams[i + PD_KD] = kd;
        m_pdParams[i + PD_KDMOD] = kdMod;
        m_pdParams[i + PD_MAX_ABS_TORQUE] = maxTorque;
    }

    public void UpdateScale (int jointIndex, Vector3 scale) {
        int i = jointIndex * (PD_REL + 1);
        m_pdParams[i + PD_SCALE] = scale.x;
        m_pdParams[i + PD_SCALE + 1] = scale.y;
        m_pdParams[i + PD_SCALE + 2] = scale.z;
    }

    public void SetControlled(int jointIndex, bool controlled) {
        int i = jointIndex * (PD_REL + 1);
        m_pdParams[i + PD_CONTROLLED] = controlled ? 1 : 0;
    }

    public void SetRelToCharFrame(int jointIndex, bool rel) {
        int i = jointIndex * (PD_REL + 1);
        m_pdParams[i + PD_REL] = rel ? 1 : 0;
    }

    public void FillPDParamSet(GameObject curSelected)
    {
        tntController controller = curSelected.GetComponentInChildren<tntController>();
        if (curSelected == null)
        {
			Debug.LogError("PDParams not created, Please set the character base");
            return;
        }

        if (controller == null)
        {
			Debug.LogError("PDParams not created. Please set the character base with a tntController");
            return;
        }
        List<tntLink> links = new List<tntLink>(curSelected.GetComponentsInChildren<tntLink>());
        links.Sort((tntLink link1, tntLink link2) => link1.GetIndex() - link2.GetIndex());
        GatherLinkNames(links.ToArray());
        FillParams(links.ToArray());
    }

    void FillParams(tntLink[] links)
    {
        m_pdParams = new float[(links.Length * (PDParamSet.PD_REL + 1))];
        m_numLinks = links.Length;
        int offset = 0;
        for (int i = 0; i < m_numLinks; i++)
        {
            tntChildLink curLink = links[i] as tntChildLink;
            if (curLink != null)
            {
                bool controlled = curLink.m_kp != 0 || curLink.m_kd != 0;
                m_pdParams[offset + PDParamSet.PD_CONTROLLED] = controlled ? 1 : 0;
                m_pdParams[offset + PDParamSet.PD_KP] = curLink.m_kp;
                m_pdParams[offset + PDParamSet.PD_KPMOD] = 1;
                m_pdParams[offset + PDParamSet.PD_KD] = curLink.m_kd;
                m_pdParams[offset + PDParamSet.PD_KDMOD] = 1;
                m_pdParams[offset + PDParamSet.PD_MAX_ABS_TORQUE] = curLink.m_maxPDTorque;
                m_pdParams[offset + PDParamSet.PD_SCALE] = 0;
                m_pdParams[offset + PDParamSet.PD_SCALE + 1] = 0;
                m_pdParams[offset + PDParamSet.PD_SCALE + 2] = 0;
                m_pdParams[offset + PDParamSet.PD_REL] = 0;
            }
            tntBase curBase = links[i] as tntBase;
            tntHumanoidController humanoidController = null;
            PDParams curParams = null;
            if (curBase != null)
            {
                humanoidController = curBase.GetComponentInChildren<tntHumanoidController>();
                if (humanoidController != null)
                {
                    curParams = humanoidController.m_rootPdParams;
                }
            }
            if (humanoidController != null)
            {
                m_pdParams[offset + PDParamSet.PD_CONTROLLED] = curParams.controlled ? 1 : 0;
                m_pdParams[offset + PDParamSet.PD_KP] = curParams.kp;
                m_pdParams[offset + PDParamSet.PD_KPMOD] = 1;
                m_pdParams[offset + PDParamSet.PD_KD] = curParams.kd;
                m_pdParams[offset + PDParamSet.PD_KDMOD] = 1;
                m_pdParams[offset + PDParamSet.PD_MAX_ABS_TORQUE] = curParams.maxAbsTorque;
                m_pdParams[offset + PDParamSet.PD_SCALE] = curParams.scale.x;
                m_pdParams[offset + PDParamSet.PD_SCALE + 1] = curParams.scale.y;
                m_pdParams[offset + PDParamSet.PD_SCALE + 2] = curParams.scale.z;
                m_pdParams[offset + PDParamSet.PD_REL] = curParams.relToCharFrame ? 1 : 0;
            }
            offset += PDParamSet.PD_REL + 1;
        }
    }

    void GatherLinkNames(tntLink[] links)
    {
        m_linkNames = new string[links.Length];
        for (int i = 0; i < links.Length; i++)
        {
            m_linkNames[i] = links[i].name;
        }
    }
}