using System;
using UnityEngine;
using PhysicsAPI;

/// <summary>
/// Drag a TNT link with the mouse using a p2p joint.
/// </summary>

public class PanAndDragTntLink : MonoBehaviour
{
	public bool m_dragPhysXBody = false;
    public float force = 600;
    public float damping = 6;
    public float MouseSpeed = 0.3f;
    public float WASDSpeed = 20.0f;

	public GameObject m_fingerPrefab = null;
	private GameObject m_fingerVisualizer = null;

    private bool m_beingTouched = false;
    private Vector3 m_lastMousePosition;

    private Transform jointTrans;
    private float dragDepth;
    
    private tntScriptedP2PConstraint m_mouseConstraint;
    
    private tntWorld m_world = null;

    private Vector3 localHitPointPosition;
#if UNITY_EDITOR
    private Vector3 pointB;
#endif
    private Transform hitObj;

	public bool IsBeingTouched() {
		return m_beingTouched;
	}

	public bool TestArticulationHit(Vector3 screenPosition)
	{
		Ray ray = Camera.main.ScreenPointToRay(screenPosition);
		CTntRayHitInfo hitInfo;
		if (!tntWorld.Raycast (ray.origin, ray.direction, Camera.main.farClipPlane, -1,
			    QueryTriggerInteraction.UseGlobal, out hitInfo))
			return false;
		if (hitInfo.hitBase == IntPtr.Zero)
			return false;
		//Debug.Log("TestArticulationHit: Hit at :" + screenPosition);
		return true;
	}

    public void HandleInputBegin(Vector3 screenPosition)
    {
        Ray ray = Camera.main.ScreenPointToRay(screenPosition);

        StartAPEDrag(screenPosition, ray);

        if (m_dragPhysXBody)
        {
            StartPhysXDrag(ray);
        }
    }

    private void StartPhysXDrag(Ray ray)
    {
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit))
        {
            dragDepth = CameraPlane.CameraToPointDepth(Camera.main, hit.point);
            jointTrans = AttachJoint(hit.rigidbody, hit.point);
            hitObj = hit.transform;
            localHitPointPosition = MathUtils.PointFromWorld(hit.point, hit.transform);
        }
    }

    private void StartAPEDrag(Vector3 screenPosition, Ray ray)
    {
        CTntRayHitInfo hitInfo;

        if (m_world != null &&
            tntWorld.Raycast(ray.origin, ray.direction, Camera.main.farClipPlane, -1, QueryTriggerInteraction.UseGlobal, out hitInfo) &&
            (hitInfo.hitBody != IntPtr.Zero || hitInfo.hitBase != IntPtr.Zero))
        {
            hitObj = hitInfo.gameObject.transform;
            dragDepth = CameraPlane.CameraToPointDepth(Camera.main, hitInfo.point);
            localHitPointPosition = MathUtils.PointFromWorld(hitInfo.point, hitObj);

            InitFingerVisualizer(hitInfo);

            InitConstraint(hitInfo);

            SeedCameraViewportCalibrator(screenPosition, hitInfo);
        }

    }

    private void SeedCameraViewportCalibrator(Vector3 screenPosition, CTntRayHitInfo hitInfo)
    {
        Vector3 cameraPlanePos = Camera.main.transform.InverseTransformPoint(hitInfo.point);
        CameraPlane.CalibrateDimension(Camera.main, dragDepth, screenPosition, cameraPlanePos);
    }

    private void InitConstraint(CTntRayHitInfo hitInfo)
    {
        m_mouseConstraint = hitObj.gameObject.AddComponent<tntScriptedP2PConstraint>();
        m_mouseConstraint.m_maxImpulse = force;

        connectCaughtEntity(hitInfo);

        m_mouseConstraint.m_pivotA = localHitPointPosition;
        m_mouseConstraint.m_pivotB = hitInfo.point;
        
    }
    
    private void connectCaughtEntity(CTntRayHitInfo hitInfo)
    {
        if (hitInfo.hitBody != IntPtr.Zero)
        {
            m_mouseConstraint.m_bodyB = hitObj.GetComponent<tntRigidBody>();
            m_mouseConstraint.m_useBodyB = true;
        }
        else if (hitInfo.hitBase != IntPtr.Zero)
        {
            m_mouseConstraint.m_linkB = hitObj.GetComponent<tntLink>();
        }
    }

    private void InitFingerVisualizer(CTntRayHitInfo hitInfo)
    {
        if (m_fingerPrefab != null)
        {
            m_fingerVisualizer = Instantiate<GameObject>(m_fingerPrefab);
            m_fingerVisualizer.transform.position = hitInfo.point;
        }
    }
    
    public void HandleInput(Vector3 screenPosition)
    {
        Vector3 jointPosition = CameraPlane.ScreenToWorldPlanePoint(Camera.main, dragDepth,
                                    screenPosition);

        if (m_fingerVisualizer != null)
            m_fingerVisualizer.transform.position = jointPosition;

        //Debug.Log ("Input: screenPosition=" + screenPosition + " dragDepth=" + dragDepth + " worldPos=" + jointPosition);
#if UNITY_EDITOR
        pointB = jointPosition;
#endif
        if (jointTrans != null)
            jointTrans.position = jointPosition;

        if (m_mouseConstraint != null)
        {
            m_mouseConstraint.SetPivot(jointPosition);
        }
    }
    
    public void HandleInputEnd(Vector3 screenPosition)
    {
        if (jointTrans != null)
        {
            Destroy(jointTrans.gameObject);
            jointTrans = null;
        }

        if (m_mouseConstraint != null)
        {
            m_mouseConstraint.RemoveConstraint();
            Destroy(m_mouseConstraint);
            m_mouseConstraint = null;
        }
        
		if (m_fingerVisualizer != null) {
			Destroy(m_fingerVisualizer);
			m_fingerVisualizer = null;
		}
    }
    
    Transform AttachJoint(Rigidbody rb, Vector3 attachmentPosition)
    {
        GameObject go = new GameObject ("Attachment Point");
        go.hideFlags = HideFlags.HideInHierarchy; 
        go.transform.position = attachmentPosition;
        
        var newRb = go.AddComponent<Rigidbody> ();
        newRb.isKinematic = true;
        
        var joint = go.AddComponent<ConfigurableJoint> ();
        joint.connectedBody = rb;
        joint.configuredInWorldSpace = true;
        joint.xDrive = NewJointDrive (force, damping);
        joint.yDrive = NewJointDrive (force, damping);
        joint.zDrive = NewJointDrive (force, damping);
        joint.slerpDrive = NewJointDrive (force, damping);
        joint.rotationDriveMode = RotationDriveMode.Slerp;
        
        return go.transform;
    }
    
    private JointDrive NewJointDrive(float force, float damping)
    {
        JointDrive drive = new JointDrive ();
        //drive.mode = JointDriveMode.Position;
        drive.positionSpring = force;
        drive.positionDamper = damping;
        drive.maximumForce = Mathf.Infinity;
        return drive;
    }

    void Awake()
    {
        if (m_world == null)
        {
            m_world = GameObject.FindObjectOfType<tntWorld>(); ;
        }
        m_world.EnsureDynamicWorld();
    }

    void Start()
    {
        m_lastMousePosition = Input.mousePosition;
		m_beingTouched = false;
    }

    void Update()
    {
        if(Input.GetMouseButton(1) && Input.GetMouseButtonDown(1) == false)
        {
			if (WASDSpeed != 0.0f)
			{
				float vertical = 0.0f;
				float horizontal = 0.0f;
				float updown = 0.0f;
				
				vertical = Input.GetAxis("Vertical");
				horizontal = Input.GetAxis("Horizontal");
				if (Input.GetKey(KeyCode.Q))
					updown = -1.0f;
				else if (Input.GetKey(KeyCode.E))
					updown = 1.0f;
				
				if (vertical != 0.0f)
					transform.position += transform.forward * vertical *
						WASDSpeed * Time.deltaTime;
				if (horizontal != 0.0f)
					transform.position += transform.right * horizontal *
						WASDSpeed * Time.deltaTime;
				if (updown != 0.0f)
					transform.position += transform.up * updown *
						WASDSpeed * Time.deltaTime;
			}
            // Right mouse button holding down
            transform.Rotate(-(Input.mousePosition.y - m_lastMousePosition.y) * MouseSpeed,
                                  0.0f,
                                  0.0f);
            transform.RotateAround(this.transform.position, Vector3.up,
                                        (Input.mousePosition.x - m_lastMousePosition.x) * MouseSpeed);
		} else if (Input.GetMouseButtonDown(0) && jointTrans == null && m_mouseConstraint == null)
        {
            // Left mouse button pressed
            HandleInputBegin(Input.mousePosition);
		} else if (jointTrans != null || m_mouseConstraint != null)
        {
			if (Input.GetMouseButton (0)) {            // Left mouse button holding down
				HandleInput (Input.mousePosition);
#if UNITY_EDITOR
				if (hitObj != null) {
					Vector3 pointA = MathUtils.PointToWorld (localHitPointPosition, hitObj);
					DrawArrow.ForDebug(pointA,(pointB-pointA),Color.red,0.02f,20,1);
				}
#endif
			}
            else if (Input.GetMouseButtonUp(0))      // Left mouse button released
                HandleInputEnd(Input.mousePosition);
        }
        
        m_lastMousePosition = Input.mousePosition;
    }
}
