using UnityEngine;
using System.Collections;

/// <summary>
/// Utility class for working with planes relative to a camera.
/// </summary>
public static class CameraPlane
{
	private static Vector2 m_dimensionCalibrator;

	public static Vector3 ViewportToCameraPlanePoint (Camera theCamera, float zDepth, Vector2 viewportCord)
	{
		theCamera.ResetAspect();
		float oppositeX = Mathf.Tan(theCamera.fieldOfView / 2 * Mathf.Deg2Rad);
		float oppositeY = oppositeX;

		float xProportion = ((viewportCord.x - .5f)/.5f);
		float yProportion = ((viewportCord.y - .5f)/.5f);

		//Debug.Log ("ViewportCoord:" + viewportCord + " normalized:" + new Vector2 (xProportion, yProportion));
		//Debug.Log ("fieldofView=" + theCamera.fieldOfView + " aspect=" + theCamera.aspect);
		float xOffset = oppositeX * xProportion * zDepth;
		float yOffset = oppositeY * yProportion * zDepth;
		return new Vector3 (xOffset, yOffset, zDepth);
	}

    /// <summary>
    /// Returns world space position at a given viewport coordinate for a given depth.
    /// </summary>
    public static Vector3 ViewportToWorldPlanePoint (Camera theCamera, float zDepth, Vector2 viewportCord)
    {
		Vector3 computedCameraPlanePos = ViewportToCameraPlanePoint(theCamera, zDepth, viewportCord);
		computedCameraPlanePos.x *= m_dimensionCalibrator.x;
		computedCameraPlanePos.y *= m_dimensionCalibrator.y;
		return theCamera.transform.TransformPoint(computedCameraPlanePos);
    }
    
    public static Vector3 ScreenToWorldPlanePoint (Camera camera, float zDepth, Vector3 screenCoord)
    {
        var point = Camera.main.ScreenToViewportPoint(screenCoord);
		return ViewportToWorldPlanePoint(camera, zDepth, point);
    }
    
    /// <summary>
    /// Distance between the camera and a plane parallel to the viewport that passes through a given point.
    /// </summary>
    public static float CameraToPointDepth (Camera cam, Vector3 point)
    {
        Vector3 localPosition = cam.transform.InverseTransformPoint (point);
        return localPosition.z;
    }

	public static void CalibrateDimension(Camera camera, float zDepth, Vector3 screenCoord, Vector3 cameraPlanePos)
	{
		var point = Camera.main.ScreenToViewportPoint(screenCoord);
		Vector3 computedCameraPlanePos = ViewportToCameraPlanePoint(camera, zDepth, point);
		m_dimensionCalibrator.x = cameraPlanePos.x / computedCameraPlanePos.x;
		m_dimensionCalibrator.y = cameraPlanePos.y / computedCameraPlanePos.y;
	}
}