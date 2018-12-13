using UnityEngine;

public static class MathUtils
{
	public static void RoundToGrid(ref Vector3 vec)
    {
        // Snap to the closest (1/2^17) grid
        float scalar = 131072f;  // 2^17
        vec = vec * scalar;
        uint rounded = (uint)(vec.x + 0.5f);
        if (Mathf.Abs(vec.x - (float)rounded) < 0.125)
            vec.x = (float)rounded;
        rounded = (uint)(vec.y + 0.5f);
        if (Mathf.Abs(vec.y - (float)rounded) < 0.125)
            vec.y = (float)rounded;
        rounded = (uint)(vec.z + 0.5f);
        if (Mathf.Abs(vec.z - (float)rounded) < 0.125)
            vec.z = (float)rounded;
        vec = vec / scalar;
    }

    public static Vector3 TransformPivot(Vector3 pivotA, Transform transA, Transform transB)
    {
        Vector3 pivotB;
        Vector3 pivotAScaled = new Vector3(pivotA.x / transA.localScale.x,
                                           pivotA.y / transA.localScale.y,
                                           pivotA.z / transA.localScale.z);
        pivotB = transB.InverseTransformPoint(transA.TransformPoint(pivotAScaled));
        pivotB.Scale(transB.localScale);
        return pivotB;
    }

    public static Vector3 PointToWorld(Vector3 pointLocal, Transform trans)
    {
        Vector3 point = new Vector3(
            pointLocal.x / trans.localScale.x,
            pointLocal.y / trans.localScale.y,
            pointLocal.z / trans.localScale.z);
        return trans.TransformPoint(point);
    }

    public static Vector3 PointFromWorld(Vector3 pointWorld, Transform trans)
    {
        Vector3 point = trans.InverseTransformPoint(pointWorld);
        point.x = point.x * trans.localScale.x;
        point.y = point.y * trans.localScale.y;
        point.z = point.z * trans.localScale.z;
        return point;
    }

    public static Vector3 FindUpFromForward(Vector3 forward)
    {
        float minDot = Mathf.Abs(Vector3.Dot(forward, Vector3.up));
        Vector3 minDotAxis = Vector3.up;

        float dot = Mathf.Abs(Vector3.Dot(forward, Vector3.right));
        if (dot < minDot)
        {
            minDot = dot;
            minDotAxis = Vector3.right;
        }

        dot = Mathf.Abs(Vector3.Dot(forward, Vector3.forward));
        if (dot < minDot)
        {
            minDot = dot;
            minDotAxis = Vector3.forward;
        }

        Vector3 up = Vector3.Cross(forward, minDotAxis);
        up.Normalize();
        return up;
    }

    public static Vector3 GetMatrix4x4Position(Matrix4x4 m)
    {
        Vector3 posColumn = m.GetColumn(3);
        return new Vector3(posColumn.x, posColumn.y, posColumn.z);
    }

    public static Quaternion GetMatrix4x4Rotation(Matrix4x4 m)
    {
        return m.rotation;
    }
}