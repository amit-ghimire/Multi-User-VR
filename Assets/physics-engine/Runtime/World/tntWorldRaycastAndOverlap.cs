using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicsAPI;

public partial class tntWorld : MonoBehaviour
{

    /**
     * Casts a collision-detection ray
     * @param from vector representing ray's start
     * @param to vector representing ray's end
     * @param hitBody will be populated with a hit rigid body's native pointer (if any rigid body is hit)
     * @param hitBase will be populated with a hit multibody base's native pointer (if any multibody is hit)
     * @param hitLink will be populated with hit multibody's link index (if any of them is hit)
     * @param location will be populated with hit location (if anything is hit)
     * @param normal will be populated with surface normal at the hit location (if anything is hit)
     * @remark hit location and normal are provided in world coordinates
     */
    public bool Raycast(Vector3 from, Vector3 to,
                        out IntPtr hitBody, out IntPtr hitBase, out int hitLink,
                        out Vector3 location, out Vector3 normal)
    {
        unsafe
        {
            fixed(IntPtr* hitRigidBody = &hitBody)
            fixed(IntPtr* hitTntBase = &hitBase)
            fixed(int* hitTntLink = &hitLink)
            fixed(Vector3* hitlocation = &location)
            fixed(Vector3* hitNormal = &normal)
            {
                return TNT.apRayCast(dynamicsWorld, &from, &to,
                                     hitRigidBody, hitTntBase, hitTntLink,
                                     hitlocation, hitNormal);
            }
        }
    }

    /**
     * Casts a collision-detection ray
     * @param from vector representing ray's start
     * @param direction vector representing ray's direction
     * @param maxDistance float representing ray's length
     * @param layerMask specifies Unity layers which will be considered during this test
     * @param triggerInteraction specifies whether Unity Triggers will be considered during this test
     * @param hitInfo will be populated with detailes information about the detected hit
     * @return true if there was at least one hit, false otherwise
     * @see http://docs.unity3d.com/ScriptReference/QueryTriggerInteraction.html
     */
    public static bool Raycast(Vector3 from, Vector3 direction, float maxDistance, int layerMask, QueryTriggerInteraction triggerInteraction,
                        out CTntRayHitInfo hitInfo)
    {
        hitInfo = new CTntRayHitInfo();

        Vector3 location = new Vector3();
        Vector3 normal = new Vector3();
        Vector3 baryCentricCoord = new Vector3();
        int worldIndex, triangleIndex;
        bool isRigidBody;
        bool bCollide = false;
        unsafe
        {
            fixed (IntPtr* hitRigidBody = &hitInfo.hitBody)
            fixed (IntPtr* hitTntBase = &hitInfo.hitBase)
            fixed (int* hitTntLink = &hitInfo.hitLinkIndex)
            {
                bCollide = TNT.apRayCast2(m_world.dynamicsWorld, &from, &direction, maxDistance, layerMask,
                                          &location, &normal, &baryCentricCoord, &triangleIndex, &worldIndex, &isRigidBody,
                                          hitRigidBody, hitTntBase, hitTntLink);
            }
        }
        if (!bCollide)
        {
            return false;
        }

        bool hitTrigger;
        switch (triggerInteraction)
        {
            case QueryTriggerInteraction.UseGlobal:
                hitTrigger = Physics.queriesHitTriggers;
                break;
            case QueryTriggerInteraction.Ignore:
                hitTrigger = false;
                break;
            case QueryTriggerInteraction.Collide:
                hitTrigger = true;
                break;
            default:
                hitTrigger = false;
                break;
        }

        GameObject go = m_world.FindGameObject(worldIndex, isRigidBody);
        if (go != null)
        {
            bool isTrigger = false;
            Collider thisCollider = go.GetComponent<Collider>();
            if (thisCollider != null)
            {
                isTrigger = thisCollider.isTrigger;
            }
            else
            {
                tntCollider thisCollider2ndAttempt = go.GetComponent<tntCollider>();
                if (thisCollider2ndAttempt)
                {
                    isTrigger = thisCollider2ndAttempt.IsTrigger;
                }
            }

            if (isTrigger && !hitTrigger)
                return false;

            m_world.fillRaycastHit(go, from, location, normal, baryCentricCoord, triangleIndex, ref hitInfo);
        }

        return bCollide;
    }

    // raycast
    void fillRaycastHit(GameObject go, Vector3 fromPostion, Vector3 hitPosition, Vector3 normal, Vector3 baryCentricCoord, int triangleIndex, ref CTntRayHitInfo hitInfo)
    {
        // set the data fields regardless the mesh collider
        hitInfo.point = hitPosition;
        hitInfo.normal = normal;
        hitInfo.distance = Vector3.Distance(hitPosition, fromPostion);
        hitInfo.gameObject = go;

        MeshCollider mc = go.GetComponent<MeshCollider>();
        if (mc == null)
            return;

        hitInfo.triangleIndex = triangleIndex;
        hitInfo.baryCentricCoord = baryCentricCoord;

        if (triangleIndex >= 0)
        {
            Mesh mesh = mc.sharedMesh;
            if (mesh == null)
                return;

            int v0 = mesh.triangles[triangleIndex * 3];
            int v1 = mesh.triangles[triangleIndex * 3 + 1];
            int v2 = mesh.triangles[triangleIndex * 3 + 2];
            Vector2 uv0 = mesh.uv[v0];
            Vector2 uv1 = mesh.uv[v1];
            Vector2 uv2 = mesh.uv[v2];
            hitInfo.textureCoord = uv0 * baryCentricCoord.x + uv1 * baryCentricCoord.y + uv2 * baryCentricCoord.z;
            if (mesh.uv2 != null && mesh.uv2.Length > 0)
            {
                uv0 = mesh.uv2[v0];
                uv1 = mesh.uv2[v1];
                uv2 = mesh.uv2[v2];
                hitInfo.lightmapCoord = uv0 * baryCentricCoord.x + uv1 * baryCentricCoord.y + uv2 * baryCentricCoord.z;
            }
        }
    }

    /**
     * Performs sphere overlapping test against colliders existing in the dynamic world
     * @param origin sphere origin
     * @param radius sphere radius
     * @param filterLayer specifies Unity layer you want the sphere to be at
     * @param maxNumColliders the max number of colliders you expect to get back
     * @return array of GameObjects overlapping with the test sphere; this array is no larger than maxNumColliders
     */
    public GameObject[] OverlapSphere(Vector3 origin, float radius, int filterLayer, int maxNumColliders)
    {
        int numOverlapping;
        int[] worldIndexBuffer = new int[maxNumColliders];
        unsafe
        {
            fixed (int* buffer = &worldIndexBuffer[0])
            {
                numOverlapping = TNT.apOverlapSphere(GetDynamicWorld(), &origin, radius, filterLayer, maxNumColliders, buffer);
            }
        }
        GameObject[] goArray = new GameObject[numOverlapping];
        for (int i = 0; i < numOverlapping; ++i)
        {
            goArray[i] = FindGameObject((int)(worldIndexBuffer[i] & (~0x80000000u)), (worldIndexBuffer[i] & 0x80000000u) != 0);
        }

        return goArray;
    }


}