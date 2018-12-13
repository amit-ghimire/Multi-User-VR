using UnityEngine;
using System.Runtime.InteropServices;

public class APEDebugVisuManager : MonoBehaviour {
    [DllImport(PhysicsAPI.TNT.KernalLibraryName)]
    private static extern void apEnableDebugVisualization([MarshalAs(UnmanagedType.I1)]bool enableDebugVisu);
    [DllImport(PhysicsAPI.TNT.KernalLibraryName)]
    private static extern void apShowDebugVisualizationInDedicatedWindow([MarshalAs(UnmanagedType.I1)]bool showDebugVisuWnd);
    [DllImport(PhysicsAPI.TNT.KernalLibraryName)]
    private static extern unsafe int apGetCurrentColliderLinesPoints(float *pLinesPoints, float *pLinesColors);

    [SerializeField][HideInInspector]
    private bool m_DebugVisualizationEmbedded = false;
    public bool DebugVisualizationEmbedded
    {
        get { return m_DebugVisualizationEmbedded; }
        set { m_DebugVisualizationEmbedded = value; }
    }

    [SerializeField][HideInInspector]
    private bool m_DebugVisualizationInDedicatedWindow = false;
#if UNITY_EDITOR_WIN || UNITY_EDITOR_STANDALONE
    public bool DebugVisualizationInDedicatedWindow
    {
        get { return m_DebugVisualizationInDedicatedWindow; }
        set
        {
            if (m_DebugVisualizationInDedicatedWindow != value)
            {
                m_DebugVisualizationInDedicatedWindow = value;
                SetDebugVisualizationInDedicatedWindowToKernel(m_DebugVisualizationInDedicatedWindow);
            }
        }
    }
#endif

    private static void SetDebugVisualizationEnabledToKernel(bool e)
    {
        if (Application.isPlaying)
            apEnableDebugVisualization(e);
    }

#if UNITY_EDITOR_WIN || UNITY_EDITOR_STANDALONE
    private static void SetDebugVisualizationInDedicatedWindowToKernel(bool s)
    {

        if (Application.isPlaying)
            apShowDebugVisualizationInDedicatedWindow(s);
    }
#endif

    void OnEnable()
    {
        SetDebugVisualizationEnabledToKernel(true);
#if UNITY_EDITOR_WIN || UNITY_EDITOR_STANDALONE
        SetDebugVisualizationInDedicatedWindowToKernel(DebugVisualizationInDedicatedWindow);
#endif
    }

    void OnDisable()
    {
        SetDebugVisualizationEnabledToKernel(false);
    }

    float [] m_colliderLinesPointsArray = new float[0];
    float [] m_colliderLinesPointsColorsArray = new float[0];

    static Material lineMaterial;
    static void CreateLineMaterial()
    {
        if (!lineMaterial)
        {
            // Unity has a built-in shader that is useful for drawing
            // simple colored things.
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;
            // Turn on alpha blending
            lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            // Turn backface culling off
            lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
            // Turn off depth test
            lineMaterial.SetInt("_ZTest", 0);
        }
    }

    void OnDrawGizmos()
    {
        if (Event.current.type != EventType.Repaint)
            return;

        DrawLines();
    }

    void OnRenderObject()
    {
        if (Event.current.type != EventType.Repaint)
            return;

        DrawLines();
    }

    void DrawLines()
    {
        if (!DebugVisualizationEmbedded)
            return;

        int currNumLinesPoints = 0;
        unsafe
        {
            currNumLinesPoints = apGetCurrentColliderLinesPoints(null, null);
            if (currNumLinesPoints > m_colliderLinesPointsArray.Length)
                m_colliderLinesPointsArray = new float[currNumLinesPoints];
            if (currNumLinesPoints > m_colliderLinesPointsColorsArray.Length)
                m_colliderLinesPointsColorsArray = new float[currNumLinesPoints];

            if (currNumLinesPoints > 0)
            {
                fixed (float* pLinesPoints = &m_colliderLinesPointsArray[0])
                fixed (float* pLinesPointsColors = &m_colliderLinesPointsColorsArray[0])
                {
                    apGetCurrentColliderLinesPoints(pLinesPoints, pLinesPointsColors);
                }
            }
        }

        if (currNumLinesPoints == 0)
            return;

        CreateLineMaterial();
        lineMaterial.SetPass(0);

        GL.Begin(GL.LINES);        
        for (int i = 0; i < m_colliderLinesPointsArray.Length; i += 6)
        {
            GL.Color(
                new Color(
                    m_colliderLinesPointsColorsArray[i],
                    m_colliderLinesPointsColorsArray[i + 1],
                    m_colliderLinesPointsColorsArray[i + 2]
                    ));

            GL.Vertex(new Vector3(
                m_colliderLinesPointsArray[i],
                m_colliderLinesPointsArray[i + 1],
                m_colliderLinesPointsArray[i + 2]
                ));

            GL.Color(
                new Color(
                    m_colliderLinesPointsColorsArray[i + 3],
                    m_colliderLinesPointsColorsArray[i + 4],
                    m_colliderLinesPointsColorsArray[i + 5]
                    ));

            GL.Vertex(new Vector3(
                m_colliderLinesPointsArray[i + 3],
                m_colliderLinesPointsArray[i + 4],
                m_colliderLinesPointsArray[i + 5]
                ));
        }
        GL.End();
    }
}
