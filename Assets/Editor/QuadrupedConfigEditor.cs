using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(QuadrupedConfig))]
public class QuadrupedConfigEditor : Editor
{
    // Add a serialized field for the explanation image
    [SerializeField] private Texture2D explanationImage;

    public override void OnInspectorGUI()
    {
        // Draw the default inspector
        base.OnInspectorGUI();

        // Draw the explanation image
        if (explanationImage != null)
        {
            GUILayout.Label(explanationImage);
        }
    }
}