using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(APEDebugVisuManager))]
public class APEDebugVisuManagerInspector : Editor
{
    protected void OnEnable()
    { }

    public override void OnInspectorGUI()
    {
        APEDebugVisuManager targetManager = target as APEDebugVisuManager;
        serializedObject.Update();

        DrawDefaultInspector();

        bool newEnableDebugVisualizationEmbedded = EditorGUILayout.Toggle(new GUIContent("Embedded"), targetManager.DebugVisualizationEmbedded);
        if (newEnableDebugVisualizationEmbedded != targetManager.DebugVisualizationEmbedded)
        {
            targetManager.DebugVisualizationEmbedded = newEnableDebugVisualizationEmbedded;
        }

#if UNITY_EDITOR_WIN
        bool newEnableDebugVisualizationWindow = EditorGUILayout.Toggle(new GUIContent("Dedicated"), targetManager.DebugVisualizationInDedicatedWindow);
        if (newEnableDebugVisualizationWindow != targetManager.DebugVisualizationInDedicatedWindow)
        {
            targetManager.DebugVisualizationInDedicatedWindow = newEnableDebugVisualizationWindow;
        }
#endif
    }

}