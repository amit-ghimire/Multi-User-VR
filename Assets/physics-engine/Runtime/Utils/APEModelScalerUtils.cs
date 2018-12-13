using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Utility class containing static functions, can be used in both editor and game mode
public class APEModelScalerUtils : MonoBehaviour
{
    public enum DIR
    {
        Z = 0,
        X = 1
    }

    //If the game object contain tntlink, return the game object otherwise return null. If it is humanoid, also assign corresponding value to the out parameters
    public static GameObject ConfigCheckAPEObject(GameObject _gameObject, out tntLink _lShoulder, out tntLink _rShoulder, out tntLink _lowerback_torso, bool _isHumanoid = true)
    {
        _lShoulder = null;
        _rShoulder = null;
        _lowerback_torso = null;

        if (_gameObject == null)
            return null;

        if (_isHumanoid)
        {
            Component[] children = _gameObject.GetComponentsInChildren<tntLink>();
            foreach (tntLink child in children)
            {
                if (child.name == "lShoulder")
                    _lShoulder = child.gameObject.GetComponent<tntLink>();
                else if (child.name == "rShoulder")
                    _rShoulder = child.gameObject.GetComponent<tntLink>();
                else if (child.name == "lowerback_torso")
                    _lowerback_torso = child.gameObject.GetComponent<tntLink>();
            }
        }

        tntLink link = _gameObject.GetComponentInChildren<tntLink>();
        if (link == null)
            return null;

        return _gameObject;
    }

    public static void ScaleGeometry(GameObject _gameObject,
                                     float _heightScalar, float _wingspanScalar,
                                     tntLink _lShoulder, tntLink _rShoulder, tntLink _lowerback_torso,
                                     float _geometryScalar = 1.0f,
                                     bool _isHumanoid = true,
                                     DIR charFaceDir = DIR.Z)
    {
        if (_gameObject == null)
            return;

        Vector3 totalGeoScalar = Vector3.one;

        Component[] links = _gameObject.GetComponentsInChildren<tntLink>();
        for (int i = 0; i < links.Length; ++i)
        {
            tntLink link = links[i] as tntLink;
            if (!_isHumanoid)
            {
                totalGeoScalar = new Vector3(1, 1, 1);
                totalGeoScalar *= _geometryScalar;
            }
            else
            {
                // Only scale bones in arms and chest according to the "Wing Span" scalar
                bool isArmChild = (link == _lShoulder || link == _rShoulder);
                bool isChest = (link == _lowerback_torso);
                tntChildLink curLink = link.GetComponent<tntChildLink>();
                // recursively check if the shoulder is an ancestor of current link
                while (curLink != null && !isArmChild && !isChest)
                {
                    if (curLink.m_parent == _lShoulder || curLink.m_parent == _rShoulder)
                    {
                        isArmChild = true;
                        break;
                    }
                    curLink = curLink.m_parent as tntChildLink;
                }
                if ((isArmChild || isChest) && charFaceDir == DIR.X && _heightScalar > 0 && _wingspanScalar > 0)
                    totalGeoScalar = new Vector3(_heightScalar, _heightScalar, _wingspanScalar);
                else if ((isArmChild || isChest) && charFaceDir == DIR.Z && _heightScalar > 0 && _wingspanScalar > 0)
                    totalGeoScalar = new Vector3(_wingspanScalar, _heightScalar, _heightScalar);
                else if (_heightScalar > 0)
                    totalGeoScalar = new Vector3(_heightScalar, _heightScalar, _heightScalar);
            }
            link.transform.localScale = Vector3.Scale(link.transform.localScale, totalGeoScalar);
            link.transform.localPosition = Vector3.Scale(link.transform.localPosition, totalGeoScalar);
            if (link as tntBallLink)
            {
                tntBallLink ball = link as tntBallLink;
                ball.PivotA = Vector3.Scale(ball.PivotA, totalGeoScalar);
                ball.AutoFillPivotB();
            }
            else if (link as tntHingeLink)
            {
                tntHingeLink hinge = link as tntHingeLink;
                hinge.PivotA = Vector3.Scale(hinge.PivotA, totalGeoScalar);
                hinge.AutoFillPivotB();
            }
            else if (link as tntUniversalLink)
            {
                tntUniversalLink universal = link as tntUniversalLink;
                universal.PivotA = Vector3.Scale(universal.PivotA, totalGeoScalar);
                universal.AutoFillPivotB();
            }
        }
    }

    public static void ScaleMass(GameObject _gameObject, float _massScalar)
    {
        Component[] links;

        links = _gameObject.GetComponentsInChildren<tntLink>();
        foreach (tntLink link in links)
        {
            link.m_mass *= _massScalar;
            //link.m_moi *= massScalar * totalGeoScalar * totalGeoScalar;
        }

        Component[] bodies;

        bodies = _gameObject.GetComponentsInChildren<tntRigidBody>();
        foreach (tntRigidBody body in bodies)
        {
            body.m_mass *= _massScalar;
            //body.m_moi *= massScalar * totalGeoScalar * totalGeoScalar;
        }
    }

    public static void ScaleMOI(GameObject _gameObject, float _moiScalar)
    {
        Component[] links;

        links = _gameObject.GetComponentsInChildren<tntLink>();
        foreach (tntLink link in links)
        {
            link.m_moi *= _moiScalar;
        }

        Component[] bodies;

        bodies = _gameObject.GetComponentsInChildren<tntRigidBody>();
        foreach (tntRigidBody body in bodies)
        {
            body.m_moi *= _moiScalar;
        }
    }

    public static float GetMassScalar(float l)
    {
        return l * l * l;
    }

    public static float GetMOIScalar(float l)
    {
        return l * l * l * l * l;
    }

    public static float GetStiffnessScalar(float l)
    {
        return l * l;
    }

    public static float GetDamperScalar(float l)
    {
        return (float)Math.Sqrt(l * l * l * l * l);
    }

    public static float GetForceScalar(float l)
    {
        return l * l * l;
    }

    public static float GetTorqueScalar(float l)
    {
        return l * l * l * l;
    }

    public static float GetVelocityScalar(float l)
    {
        return (float)Math.Sqrt(l);
    }

    public static float getUnformScalar(float width, float height)
    {
        // This is an approximation of equivalent 1D uniform scalar based on the scaling formula used
        // in APEModelScalerUtils.ScaleGeometry()
        return (float)System.Math.Pow(width * height * height, (1.0 / 3.0));
    }

    public static void ScaleStrengths(GameObject _gameObject, float geometryScalar)
    {
        float damperScalar = GetDamperScalar(geometryScalar);
        float stiffnessScalar = GetStiffnessScalar(geometryScalar);
        float torqueScalar = GetTorqueScalar(geometryScalar);
        float forceScalar = GetForceScalar(geometryScalar);
        float velocityScalar = GetVelocityScalar(geometryScalar);
        ScaleStrengths(_gameObject, geometryScalar, stiffnessScalar, damperScalar,
                       torqueScalar, forceScalar, velocityScalar);
    }

    public static void ScaleStrengths(GameObject _gameObject,
                                      float geometryScalar,
                                      float stiffnessScalar,
                                      float damperScalar,
                                      float torqueScalar,
                                      float forceScalar,
                                      float velocityScalar)
    {
        Component[] components;
        components = _gameObject.GetComponentsInChildren<tntBipedController>();
        foreach (tntBipedController controller in components)
        {
            for (int i = 0; i < controller.controlParams.Count; ++i)
            {
                PDParams pdParams = controller.controlParams[i];
                pdParams.kd *= damperScalar;
                pdParams.kp *= stiffnessScalar;
                pdParams.maxAbsTorque *= torqueScalar;

            }
            PDParams rootParams = controller.rootControlParams;
            rootParams.kd *= damperScalar;
            rootParams.kp *= stiffnessScalar;
            rootParams.maxAbsTorque *= torqueScalar;

            for (int i = 0; i < controller.states.Count; ++i)
            {
                BipedConState state = controller.states[i];
                for (int j = 0; j < state.sExternalForces.Count; ++j)
                {
                    ExternalForce force = state.sExternalForces[j];
                    force.scale(forceScalar);
                }
                state.minSwingFootForceForContact *= forceScalar;
            }
        }

        components = _gameObject.GetComponentsInChildren<tntTRexController>();
        foreach (tntTRexController controller in components)
        {
            for (int i = 0; i < controller.m_pdParams.Count; ++i)
            {
                PDParams pdParams = controller.m_pdParams[i];
                pdParams.kd *= damperScalar;
                pdParams.kp *= stiffnessScalar;
                pdParams.maxAbsTorque *= torqueScalar;

            }
            PDParams rootParams = controller.m_rootPdParams;
            rootParams.kd *= damperScalar;
            rootParams.kp *= stiffnessScalar;
            rootParams.maxAbsTorque *= torqueScalar;
        }

        components = _gameObject.GetComponentsInChildren<tntDogController>();
        foreach (tntDogController controller in components)
        {
            // TBD: Add support for per-bone Kp/Kd                ControlParams rootParams = controller.m_rootPdParams;
            PDParams rootParams = controller.m_rootPdParams;
            rootParams.kd *= damperScalar;
            rootParams.kp *= stiffnessScalar;
            rootParams.maxAbsTorque *= torqueScalar;
        }

        // TBD: Add support for tntHumanoidController
        //    
        components = _gameObject.GetComponentsInChildren<tntChildLink>();
        foreach (tntChildLink link in components)
        {
            for (int j = 0; j < link.m_dofData.Length; ++j)
            {
                tntDofData dofData = link.m_dofData[j];

                // Assumption: All DOFs of all joints are rotational joints which is
                // true for most of the humanoid models.
                // TBD: Distinguish torque scalar from force scalar

                dofData.m_maxLimitForce *= torqueScalar;
                dofData.m_maxMotorForce *= torqueScalar;
                dofData.m_springStiffness *= stiffnessScalar;
                dofData.m_springDamping *= damperScalar;
                dofData.m_continuousForce *= torqueScalar;
            }
			link.m_kp *= stiffnessScalar;
			link.m_kd *= damperScalar;
			link.m_maxPDTorque *= torqueScalar;
        }

        components = _gameObject.GetComponentsInChildren<tntBase>();
        foreach (tntBase baseLink in components)
        {
            baseLink.m_maxCoordinateVelocity *= velocityScalar;
        }
    }
}
