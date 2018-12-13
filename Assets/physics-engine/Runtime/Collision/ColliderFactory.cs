using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using PhysicsAPI;

using System;

/**
 * Enumeration of general APE collider types 
 */
public enum tntShapeType
{
    INVALID = -1,
    BasicShape,
    ConvexHull,
    ConcaveMeshStatic,
    ConcaveMeshDynamic,
    CompoundShape,
    HeightMap
};

/**
 * @brief Utility class for creating APE's internal collision shapes
 */
public class ColliderFactory
{
    private static float[,] m_heightMap = null;     // Assumption: Only a single terrain instance per scene
     
    private static List<Vector3> MergeDuplicateVertices(Vector3[] oldVertices, float threshold, int bucketStep)
    {
        List<Vector3> newVertices = new List<Vector3>();
        int[] old2new = new int[oldVertices.Length];
        int newSize = 0;
        
        // Find AABB
        Vector3 min = new Vector3 (float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 max = new Vector3 (float.MinValue, float.MinValue, float.MinValue);
        for (int i = 0; i < oldVertices.Length; i++) {
            if (oldVertices[i].x < min.x) min.x = oldVertices[i].x;
            if (oldVertices[i].y < min.y) min.y = oldVertices[i].y;
            if (oldVertices[i].z < min.z) min.z = oldVertices[i].z;
            if (oldVertices[i].x > max.x) max.x = oldVertices[i].x;
            if (oldVertices[i].y > max.y) max.y = oldVertices[i].y;
            if (oldVertices[i].z > max.z) max.z = oldVertices[i].z;
        }
        
        // Make cubic buckets, each with dimensions "bucketStep"
        int bucketSizeX = Mathf.FloorToInt ((max.x - min.x) / bucketStep) + 1;
        int bucketSizeY = Mathf.FloorToInt ((max.y - min.y) / bucketStep) + 1;
        int bucketSizeZ = Mathf.FloorToInt ((max.z - min.z) / bucketStep) + 1;
        List<int>[,,] buckets = new List<int>[bucketSizeX, bucketSizeY, bucketSizeZ];
        
        // Make new vertices
        for (int i = 0; i < oldVertices.Length; i++) {
            // Determine which bucket it belongs to
            int x = Mathf.FloorToInt ((oldVertices[i].x - min.x) / bucketStep);
            int y = Mathf.FloorToInt ((oldVertices[i].y - min.y) / bucketStep);
            int z = Mathf.FloorToInt ((oldVertices[i].z - min.z) / bucketStep);
            
            // Check to see if it's already been added
            if (buckets[x, y, z] == null)
                buckets[x, y, z] = new List<int> (); // Make buckets lazily
            
            for (int j = 0; j < buckets[x, y, z].Count; j++) {
                Vector3 to = newVertices[buckets[x, y, z][j]] - oldVertices[i];
                if (Vector3.SqrMagnitude (to) < threshold) {
                    old2new[i] = buckets[x, y, z][j];
                    goto skip; // Skip to next old vertex if this one is already there
                }
            }
            
            // Add new vertex
            newVertices.Add(oldVertices[i]);
            buckets[x, y, z].Add(newSize);
            old2new[i] = newSize;
            newSize++;
            
        skip:;
        }
        return newVertices;
    }

    private static IntPtr CreateUnityColliderShape(Collider collider, bool anchored, out tntShapeType shapeType, out Vector3 transformOffset)
    {
        // Automatically disable Unity Collider to avoid churning out PhysX CPU cycles since we are simulating
        // the collision shape within Articulated Physics Engine
        // collider.enabled = false;

        shapeType = tntShapeType.BasicShape;
        transformOffset = Vector3.zero;

        BoxCollider boxCollider = collider as BoxCollider;
        if (boxCollider != null)
        {
            Vector3 boxSize = boxCollider.size;
            return TNT.apNewBoxShape(boxSize.x * 0.5f, boxSize.y * 0.5f, boxSize.z * 0.5f);
        }
        
        CapsuleCollider capsuleCollider = collider as CapsuleCollider;
        if (capsuleCollider != null)
        {
            // Note: Capsule collider doesn't support scaling in engine. So we bake in the scales
            // to the geometric parameters directly.
            Vector3 scaledX = collider.transform.TransformVector(new Vector3(1, 0, 0));
            Vector3 scaledY = collider.transform.TransformVector(new Vector3(0, 1, 0));
            Vector3 scaledZ = collider.transform.TransformVector(new Vector3(0, 0, 1));
            if (capsuleCollider.direction == 0)
            {
                // along X
                float height = capsuleCollider.height * scaledX.magnitude;
                float radius = capsuleCollider.radius * Mathf.Max(scaledY.magnitude, scaledZ.magnitude);
                return TNT.apNewCapsuleShapeX(radius, (height - radius * 2) / 2);
            }
            else if (capsuleCollider.direction == 1)
            {
                // along Y
                float height = capsuleCollider.height * scaledY.magnitude;
                float radius = capsuleCollider.radius * Mathf.Max(scaledX.magnitude, scaledZ.magnitude);
                return TNT.apNewCapsuleShape(radius, (height - radius * 2) / 2);
            }
            else if (capsuleCollider.direction == 2)
            {
                // along Z
                float height = capsuleCollider.height * scaledZ.magnitude;
                float radius = capsuleCollider.radius * Mathf.Max(scaledX.magnitude, scaledY.magnitude);
                return TNT.apNewCapsuleShapeZ(radius, (height - radius * 2) / 2);
            }
        }
        
        SphereCollider sphereCollider = collider as SphereCollider;
        if (sphereCollider != null)
        {
            return TNT.apNewSphereShape(sphereCollider.radius);
        }
        
        IntPtr collisionShape = IntPtr.Zero;

        MeshCollider meshCollider = collider as MeshCollider;
        if (meshCollider != null)
        {

            Mesh mesh = meshCollider.sharedMesh;
            if (mesh == null)
            {
                Debug.LogError(collider.transform.name + " missing sharedMesh from meshCollider");
                return collisionShape;
            }

            Vector3[] vertices = mesh.vertices;
            if (meshCollider.convex)
            {
                collisionShape = TNT.apNewConvexHullShape();
                // List<Vector3> mergedVertices = MergeDuplicateVertices(mesh.vertices, 1e-6f, 10);
                //Debug.Log("Merged mesh collider from " + mesh.vertices.Length + "vertices to " + mergedVertices.Count + " vertices");
                for (int i = 0; i < vertices.Length; ++i)
                {
                    TNT.apAddVertex(collisionShape, vertices[i].x, vertices[i].y, vertices[i].z);
                    //Debug.Log ("Adding vertice :" + mergedVertices[i]);
                }
                shapeType = tntShapeType.ConvexHull;
            } else
            {
                // static concave triangle mesh
                IntPtr triangleMesh = TNT.apNewTriangleMesh();
                int[] triangles = mesh.triangles;
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    Vector3 vertex0 = vertices[triangles[i]];
                    Vector3 vertex1 = vertices[triangles[i + 1]];
                    Vector3 vertex2 = vertices[triangles[i + 2]];
                    unsafe
                    {
                        TNT.apAddTriangle(triangleMesh, &vertex0, &vertex1, &vertex2);
                    }
                }
                collisionShape = TNT.apNewConcaveMeshShape(triangleMesh, anchored);
                shapeType = anchored ? tntShapeType.ConcaveMeshStatic : tntShapeType.ConcaveMeshDynamic;
            }	
        }

        TerrainCollider terrainCollider = collider as TerrainCollider;
        if (terrainCollider != null)
        {
            TerrainData terrainData = terrainCollider.terrainData;
            if (terrainData != null)
            {
                shapeType = tntShapeType.HeightMap;
                int mapWidth = terrainData.heightmapWidth;
                int mapHeight = terrainData.heightmapHeight;
                if (mapWidth != mapHeight || mapWidth != terrainData.heightmapResolution)
                {
                    Debug.LogError("None equal heightmapWidth, heightmapHeight and heightmapResolution of TerrainData are not supported");
                    return collisionShape;
                }
                // Assumption: resolution is the same as (mapWidth, mapHeight)
                m_heightMap = terrainData.GetHeights(0, 0, mapWidth, mapHeight);
                if (m_heightMap != null)
                {
                    float maxHeight = 0;
                    for (int x = 0; x < mapWidth; ++x)
                        for (int y = 0; y < mapHeight; ++y)
                        {
                            m_heightMap[x, y] *= terrainData.size.y;
                            if (m_heightMap[x, y] > maxHeight)
                                maxHeight = m_heightMap[x, y];
                        }
                    unsafe
                    {
                        fixed (float* pHeights = &m_heightMap[0, 0])
                        {
                            //Debug.Log("Add HeihtMap of width:" + mapWidth + " height:" + mapHeight + "Y size =" + terrainData.size.y+ " maxHeight = " + maxHeight);
                                 
                            collisionShape = TNT.apNewHeightMapShape(mapWidth, mapHeight, pHeights, maxHeight);
                        }
                    }
                    transformOffset = new Vector3(mapWidth / 2, maxHeight / 2, mapHeight / 2);
                } else
                {
                    Debug.LogError("Only a single instance of TerrainCollider per scene is supported now.");
                }
            }
        }
 
        return collisionShape;
    }

    private static IntPtr CreateTNTColliderShape(tntCollider collider, out tntShapeType shapeType)
    {
        shapeType = tntShapeType.BasicShape;
        tntConeCollider coneCollider = collider as tntConeCollider;
        if (coneCollider != null)
        {				
            return TNT.apNewConeShape(coneCollider.Radius, coneCollider.Height);
        }
        tntConeColliderX coneColliderX = collider as tntConeColliderX;
        if (coneColliderX != null)
        {               
            return TNT.apNewConeShapeX(coneColliderX.Radius, coneColliderX.Height);
        }
        tntConeColliderZ coneColliderZ = collider as tntConeColliderZ;
        if (coneColliderZ != null)
        {               
            return TNT.apNewConeShapeZ(coneColliderZ.Radius, coneColliderZ.Height);
        }
        tntCylinderCollider cylinderCollider = collider as tntCylinderCollider;
        if (cylinderCollider != null)
        {
            return TNT.apNewCylinderShape(cylinderCollider.Radius, cylinderCollider.Height / 2);
        }
        tntCylinderColliderX cylinderColliderX = collider as tntCylinderColliderX;
        if (cylinderColliderX != null)
        {
            return TNT.apNewCylinderShapeX(cylinderColliderX.Radius, cylinderColliderX.Height / 2);
        }
        tntCylinderColliderZ cylinderColliderZ = collider as tntCylinderColliderZ;
        if (cylinderColliderZ != null)
        {
            return TNT.apNewCylinderShapeZ(cylinderColliderZ.Radius, cylinderColliderZ.Height / 2);
        }
        return IntPtr.Zero;
    }

    private static void SetColliderTotalScale(IntPtr bodyShape, Transform transform)
    {           
        if (bodyShape == IntPtr.Zero)
        {
            Debug.LogError(transform.name + ": Collider factory can't set scale due to invalid body shape");
            return;
        }

        Vector3 scaledX = transform.TransformVector(new Vector3(1, 0, 0));
        Vector3 scaledY = transform.TransformVector(new Vector3(0, 1, 0));
        Vector3 scaledZ = transform.TransformVector(new Vector3(0, 0, 1));
        
        TNT.apSetScaling(bodyShape, scaledX.magnitude, scaledY.magnitude, scaledZ.magnitude);
    }

    private static void SetColliderLocalScale(IntPtr bodyShape, Transform transform)
    {           
        if (bodyShape == IntPtr.Zero)
        {
            Debug.LogError("Collider factory can't set scale due to invalid body shape");
            return;
        }
        TNT.apSetScaling(bodyShape, transform.localScale.x, transform.localScale.y, transform.localScale.z);
    }

    public static tntCompoundCollider FindContainingCompound(Component collider)
    {
        if (!collider || !IsCollider(collider) || !collider.transform.parent)
            return null;

        Component currGO = collider.transform.parent;
        while (currGO != null && currGO.GetComponent<tntCompoundCollider>() == null)
            currGO = currGO.transform.parent;

        return currGO != null ? currGO.GetComponent<tntCompoundCollider>() : null;
    }

    static bool IsCollider(Component comp)
    {
        return
            comp is Collider || comp is tntCollider;
    }

    static tntCompoundCollider GetParentCompound(Component collider)
    {
        if (!collider || !IsCollider(collider))
            return null;

        if (collider.transform.parent == null)
            return null;

        return collider.transform.parent.gameObject.GetComponent<tntCompoundCollider>();
    }

    // a collider is considered a child collider if:
    //  + it's a collider (IsCollider)
    //  + it has a compound collider assigned to its parent 
    static bool IsChildCollider(Component collider)
    {
        return            
            IsCollider(collider) &&
            FindContainingCompound(collider) != null;
    }

    // note: in the current impl, this mass is taken into account only in compound configs (but can be assigned anyway)
    public static bool TrySetColliderAssignedMassAndMoI(Component collider, float mass, Vector3 moi)
    {
        if (!collider || !IsCollider(collider))
            return false;

        tntRigidBody massProvidingRB = collider.GetComponent<tntRigidBody>();
        massProvidingRB = massProvidingRB == null ?
            collider.gameObject.AddComponent<tntRigidBody>() : massProvidingRB;

        massProvidingRB.m_mass = mass;
        massProvidingRB.m_moi = moi;

        return true;
    }

    // note: in the current impl, this mass is taken into account only in compound configs (but can be assigned anyway)
    public static bool TryGetColliderAssignedMassAndMoI(Component collider, out float mass, out Vector3 moi)
    {
        // default values
        mass = 0.0f;
        moi = Vector3.zero;

        if (!collider || !IsCollider(collider))
            return false;

        tntRigidBody massProvidingRB = collider.GetComponent<tntRigidBody>();
        if (massProvidingRB == null)
            return false;               // no mass info but this is a valid case
        if (massProvidingRB.m_mass < 0)
        {
            Debug.LogError("A collider with negative mass on it: " + collider.name);
            return false;
        }

        mass = massProvidingRB.m_mass;
        return true;
    }

    // if there is NO mass/moi assigned to a child collider -> :
    //  if the parent compound has a mass (M) and moi assigned to it then child mass = M / numChildColliders, child moi = vec3.zero
    //  if the parent compound has no mass (M) moi assigned to it -> FAIL
    // if there is a mass/moi assigned to a child collider
    //      if that mass != 0 -> use the mass and moi
    //      if that mass == 0 -> FAIL
    public static bool ComputeChildColliderMass(Component collider, out float mass, out Vector3 moi, int numChildColliders = 0)
    {
        // default values
        mass = 0.0f;
        moi = Vector3.zero;

        if (!collider || !IsChildCollider(collider))
            return false;

        float childMass;
        Vector3 childMoI;
        if (TryGetColliderAssignedMassAndMoI(collider, out childMass, out childMoI))
        {
            if (childMass > 0.0f)
            {
                mass = childMass;
                moi = childMoI;
                return true;
            }

            Debug.LogErrorFormat("{0}: zero compound collider child's mass specified - invalid setting", collider.name);
            return false;            
        }

        tntCompoundCollider parentCompound = GetParentCompound(collider);    // needs to have one since IsChildCollider passed

        float parentMass;
        Vector3 parentMoi;
        if (TryGetColliderAssignedMassAndMoI(parentCompound, out parentMass, out parentMoi))
        {
            numChildColliders = numChildColliders > 0 ?
                numChildColliders : GetNumberOfChildColliders(parentCompound);
            mass = parentMass / numChildColliders;
            moi = Vector3.zero;         // will be auto-computed inside kernel
            return true;
        }

        Debug.LogErrorFormat("{0}: missing both compound child and parent collider mass information - invalid setting", collider.name);
        return false;
    }

    static int GetNumberOfChildColliders(MonoBehaviour behavior)
    {
        if (behavior == null)
            return 0;

        Component mainCollider;
        bool isCompound;
        Component[] childColliders;
        if (ExtractColliderDefinitions(behavior, out mainCollider, out isCompound, out childColliders))
            return isCompound ? childColliders.Length : 0;

        return 0;
    }

    public static bool ComputeChildColliderMasses(
        Component [] childColliders,
        out float[] masses, out float[] inertias)
    {
        masses = null;
        inertias = null;

        if (childColliders == null)
            return false;

        if (childColliders.Length == 0)
            return true;

        int numChildColliders = childColliders.Length;
        masses = new float[numChildColliders];
        inertias = new float[numChildColliders * 3];

        float currChildMass;
        Vector3 currChildMoi;
        for (int i = 0; i < childColliders.Length; ++i)
        {
            Component currChildCollider = childColliders[i];

            if (currChildCollider is tntCompoundCollider)
                continue;

            if (!ComputeChildColliderMass(currChildCollider, out currChildMass, out currChildMoi))
            {
                masses = null;
                inertias = null;
                return false;
            }

            masses[i] = currChildMass;
            inertias[i * 3 + 0] = currChildMoi.x;
            inertias[i * 3 + 1] = currChildMoi.y;
            inertias[i * 3 + 2] = currChildMoi.z;
        }

        return true;
    }

    // returns false if there are no colliders
    public static bool ExtractColliderDefinitions(
        MonoBehaviour behavior,
        out Component mainCollider,
        out bool isCompound,
        out Component [] childColliders)
    {
        mainCollider = null;
        isCompound = false;
        childColliders = null;

        if (behavior == null)
            return false;

        Component[] unityChildColliders = null, tntChildColliders = null, tntNonCompoundChildColliders = null, tntCompoundChildColliders = null;
        int numNonCompoundChildColliders = 0, numChildColliders = 0;

        Collider unityTopMostCollider = behavior.GetComponent<Collider>();
        tntCollider tntTopMostCollider = behavior.GetComponent<tntCollider>();

        if (unityTopMostCollider == null && tntTopMostCollider == null)
            return false;

        tntCompoundCollider topMostCompoundCollider = tntTopMostCollider as tntCompoundCollider;

        if (topMostCompoundCollider)
        {
            unityChildColliders = System.Array.FindAll<Collider>(
                behavior.GetComponentsInChildren<Collider>(),
                c => FindContainingCompound(c) == topMostCompoundCollider
                );
            tntChildColliders = System.Array.FindAll<tntCollider>(
                behavior.GetComponentsInChildren<tntCollider>(),
                c => FindContainingCompound(c) == topMostCompoundCollider
                );

            tntNonCompoundChildColliders = System.Array.FindAll<Component>(
                tntChildColliders,
                comp => !(comp is tntCompoundCollider));
            tntCompoundChildColliders = System.Array.FindAll<Component>(
                tntChildColliders,
                comp => comp is tntCompoundCollider && comp != topMostCompoundCollider);

            numNonCompoundChildColliders = unityChildColliders.Length + tntChildColliders.Length;
            numChildColliders = numNonCompoundChildColliders + tntCompoundChildColliders.Length;
        }

        bool standAloneUnityCollider = false, standAloneTntCollider = false, compoundCollider = false;
        bool forcedStandAloneUnityCollider = false, forcedStandAloneTntCollider = false;

        // regular cases
        compoundCollider = topMostCompoundCollider != null && numChildColliders > 0;        // note: empty compounds are allowed
        standAloneUnityCollider = !compoundCollider && unityTopMostCollider != null && tntTopMostCollider == null && numChildColliders == 0;
        standAloneTntCollider = !compoundCollider && !standAloneUnityCollider && tntTopMostCollider != null && numChildColliders == 0;;
        // irregular (yet handled) cases
        forcedStandAloneUnityCollider = !compoundCollider && unityTopMostCollider != null && (numChildColliders > 0 || tntTopMostCollider != null);
        forcedStandAloneTntCollider = !compoundCollider && tntTopMostCollider != null && (numChildColliders > 0 || unityTopMostCollider != null);

        if (compoundCollider)
        {
            mainCollider = topMostCompoundCollider;
            isCompound = true;

            childColliders = new Component[numNonCompoundChildColliders];
            unityChildColliders.CopyTo(childColliders, 0);
            tntNonCompoundChildColliders.CopyTo(childColliders, unityChildColliders.Length);

            return true;
        }

        if (standAloneUnityCollider || forcedStandAloneUnityCollider)
        {
            mainCollider = unityTopMostCollider;

            if (forcedStandAloneUnityCollider)
                Debug.LogWarning(string.Format("{0}: non-standard collider config: some colliders ignored", behavior.name));
            return true;
        }

        if (standAloneTntCollider || forcedStandAloneTntCollider)
        {
            mainCollider = tntTopMostCollider;

            if (forcedStandAloneTntCollider)
                Debug.LogWarning(string.Format("{0}: non-standard collider config: some colliders ignored", behavior.name));
            return true;
        }

        Debug.LogError(string.Format("{0}: invalid collider config", behavior.name));
        return false;
    }

    /**
     * Creates internal collision shape based on Unity Collider and/or tntCollider objects attached to the passed in MonoBehaviour
     * @param behavior is used to provide collider description (attached Unity Collider and/or tntCollider objects)
     * @param anchored specifies if the requested collider can be assumed to be immobile, true means immobile, false - mobile
     * @param shapeType will be populated by tntShapeType describing the collider created internally by APE
     * @param masses masses will be populated by masses that APE assigned to created child colliders (tntCompoundCollider only, see remarks)
     * @param inertias will be populated by inertias (inertia tensors' diagonals) that APE assigned to created child colliders (tntCompoundCollider only, see remarks)
     * @param transformOffset .
     * @return native pointer to the internal collision shape object
     * @remark If one of collider objects assigned to behavior is tntCompoundCollider then all other colliders attached to it will be treated as child colliders constituting to a single compound shape.
     * @remark If tntCompoundCollider is immobile, masses and inertias for children will not be populated
     * @remark collision shape sharing among multiple bodies is currently not supported
     * @see http://docs.unity3d.com/ScriptReference/Collider.html
     */
    public static IntPtr AddCollider(MonoBehaviour behavior, bool anchored,
                                     out tntShapeType shapeType,
                                     out float[] masses,
                                     out float[] inertias,
                                     out Vector3 transformOffset)
{
        IntPtr bodyShape = IntPtr.Zero;
        shapeType = tntShapeType.INVALID;
        masses = inertias = null;
        transformOffset = Vector3.zero;
        bool hasCapsule = false;

        Component mainCollider;
        bool isCompound;
        Component [] childColliders;
        if (!ExtractColliderDefinitions(behavior, out mainCollider, out isCompound, out childColliders))
        {
            Debug.LogError("Failed to extract collider definitions from " + behavior.name);
            return IntPtr.Zero;
        }

        if (!isCompound)
        {
            if (mainCollider is Collider)
            {
                Collider collider = mainCollider as Collider;
                if (collider as CapsuleCollider)
                    hasCapsule = true;
                bodyShape = CreateUnityColliderShape(collider, anchored, out shapeType, out transformOffset);
            }
            else if (mainCollider is tntCollider)
            {
                tntCollider collider = mainCollider as tntCollider;
                bodyShape = CreateTNTColliderShape(collider, out shapeType);
            }
            else
            {
                Debug.LogError("Neither Unity collider nor tntCollider for " + behavior.name);
                return IntPtr.Zero;           // TODO: should not ever happen, log an issue?
            }
        }
        else
        {
            if (!anchored && !ComputeChildColliderMasses(childColliders, out masses, out inertias))
            {
                masses = null;
                inertias = null;
                shapeType = tntShapeType.CompoundShape;
                Debug.LogError("Failed to compute child collider masses for " + behavior.name);
                return IntPtr.Zero;
            }

            // Compound shape
            bodyShape = TNT.apNewCompoundShape();
            shapeType = tntShapeType.CompoundShape;

            for (int i = 0; i < childColliders.Length; ++i)
            {
                bool childIsCapsule = false;
                Component currChildCollider = childColliders[i];
                IntPtr currChildShape = IntPtr.Zero;
                tntShapeType currChildShapeType;            // not really used anyhow, needed as out param only
                Vector3 currChildTransformOffset;           // not really used anyhow, needed as out param only

                if (currChildCollider is Collider)
                {
                    currChildShape = CreateUnityColliderShape(currChildCollider as Collider, anchored, out currChildShapeType, out currChildTransformOffset);
                    if (currChildCollider as CapsuleCollider)
                        hasCapsule = childIsCapsule = true;
                }
                else if (currChildCollider is tntCollider)
                    currChildShape = CreateTNTColliderShape(currChildCollider as tntCollider, out currChildShapeType);

                if (currChildShape == IntPtr.Zero)
                    continue;           // TODO: should not ever happen, log an issue?

                //if (!childIsCapsule)// for capsule collider, the scale was applied inside CreateUnityColliderShape() 
                    SetColliderLocalScale(currChildShape, currChildCollider.transform);

                unsafe
                {
                    Quaternion relativeRot = currChildCollider.transform.localRotation;
                    Vector3 relativePos = currChildCollider.transform.localPosition;

                    TNT.apAddChildShape(bodyShape, currChildShape, &relativePos, &relativeRot);
                }
            }

            if (hasCapsule) // there is capsule in the child colliders
            {
                Vector3 scaledX = behavior.transform.TransformVector(new Vector3(1, 0, 0));
                Vector3 scaledY = behavior.transform.TransformVector(new Vector3(0, 1, 0));
                Vector3 scaledZ = behavior.transform.TransformVector(new Vector3(0, 0, 1));
				if (Mathf.Abs(scaledX.sqrMagnitude - 1) > 1e-5 ||
					Mathf.Abs(scaledY.sqrMagnitude - 1) > 1e-5 ||
					Mathf.Abs(scaledZ.sqrMagnitude - 1) > 1e-5)
                {
					Debug.LogError("The scale of the compound collider:" + behavior.name +
						" has to be set to (1,1,1) if it has at least one capsule child collider");
                }
            }
        }

        if (!hasCapsule) // Notes: capsule collider doesn't support scaling via this API
            SetColliderTotalScale(bodyShape, behavior.transform);

        return bodyShape;
    }

    /**
     * Deletes the internal collision shape
     * @param shape native pointer to the internal collision shape
     * @param shapeType describes the type of the internal collision shape
     */
    public static void DeleteCollider(IntPtr shape, tntShapeType shapeType)
    {
        switch (shapeType)
        {
            case tntShapeType.ConcaveMeshStatic:
                TNT.apDeleteConcaveMeshShape(shape, true);
                break;
            case tntShapeType.ConcaveMeshDynamic:
                TNT.apDeleteConcaveMeshShape(shape, false);
                break;     
            case tntShapeType.CompoundShape:
                TNT.apDeleteCompoundShape(shape);
                break;
            case tntShapeType.HeightMap:
                TNT.apDeleteShape(shape);
                m_heightMap = null;     // release C# memory allocated for storing the heightmap singleton
                break;
            default:
                TNT.apDeleteShape(shape);
                break;
        }
    }
}
