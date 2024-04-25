using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(QuadrupedActuators))]
public class QuadrupedActuatorsEditor : Editor
{
    private static readonly string[] JointNames = new string[]
    {
        "Front Left Hip Joint",
        "Front Left Upper Joint",
        "Front Left Lower Joint",
        "Front Right Hip Joint",
        "Front Right Upper Joint",
        "Front Right Lower Joint",
        "Rear Left Hip Joint",
        "Rear Left Upper Joint",
        "Rear Left Lower Joint",
        "Rear Right Hip Joint",
        "Rear Right Upper Joint",
        "Rear Right Lower Joint"
    };

    public override void OnInspectorGUI()
    {
        QuadrupedActuators quadrupedActuators = (QuadrupedActuators)target;

        int jointPerLegCount = 3;
        int columnCount = 4;

        float columnWidth = EditorGUIUtility.currentViewWidth / columnCount - 20;

        GUIStyle columnHeaderStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontStyle = FontStyle.Bold };
        GUIStyle jointNameStyle = new GUIStyle(GUI.skin.label) { wordWrap = true };

        // Root Articulation
        EditorGUILayout.LabelField("[Root Articulation]", EditorStyles.boldLabel);
        quadrupedActuators.rootArticulation = (ArticulationBody)EditorGUILayout.ObjectField("Root Articulation", quadrupedActuators.rootArticulation, typeof(ArticulationBody), true);

        // Actuator Parameters
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("[Default Actuator Parameters]", EditorStyles.boldLabel);
        quadrupedActuators.actuatorParams.stiffness = EditorGUILayout.FloatField("Stiffness", quadrupedActuators.actuatorParams.stiffness);
        quadrupedActuators.actuatorParams.damping = EditorGUILayout.FloatField("Damping", quadrupedActuators.actuatorParams.damping);
        quadrupedActuators.actuatorParams.forceLimit = EditorGUILayout.FloatField("Force Limit", quadrupedActuators.actuatorParams.forceLimit);
        quadrupedActuators.actuatorParams.speed = EditorGUILayout.FloatField("Speed", quadrupedActuators.actuatorParams.speed);
        quadrupedActuators.actuatorParams.torque = EditorGUILayout.FloatField("Torque", quadrupedActuators.actuatorParams.torque);
        quadrupedActuators.actuatorParams.acceleration = EditorGUILayout.FloatField("Acceleration", quadrupedActuators.actuatorParams.acceleration);

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("[Joint Setting]", EditorStyles.boldLabel);
        EditorGUILayout.BeginHorizontal();
        GUILayout.Space(15);
        GUILayout.Label("Joint Name", columnHeaderStyle, GUILayout.Width(columnWidth));
        GUILayout.Label("Articulation Body", columnHeaderStyle, GUILayout.Width(columnWidth));
        GUILayout.Label("Invert Rotation Axis", columnHeaderStyle, GUILayout.Width(columnWidth));
        GUILayout.Label("Rotation Offset", columnHeaderStyle, GUILayout.Width(columnWidth));
        EditorGUILayout.EndHorizontal();

        EditorGUI.indentLevel++;

        for (int i = 0; i < quadrupedActuators.joints.Length; i++)
        {
            EditorGUILayout.BeginHorizontal();

            GUILayout.Label(JointNames[i], jointNameStyle, GUILayout.Width(columnWidth));
            quadrupedActuators.joints[i].articulationBody = (ArticulationBody)EditorGUILayout.ObjectField(quadrupedActuators.joints[i].articulationBody, typeof(ArticulationBody), true, GUILayout.Width(columnWidth));
            GUILayout.BeginHorizontal();
            GUILayout.Space(columnWidth / 2 - 10);
            quadrupedActuators.joints[i].invertRotationAxis = EditorGUILayout.Toggle(quadrupedActuators.joints[i].invertRotationAxis, GUILayout.Width(20));
            GUILayout.EndHorizontal();
            quadrupedActuators.joints[i].rotationOffset = EditorGUILayout.IntField("", quadrupedActuators.joints[i].rotationOffset, GUILayout.Width(columnWidth));

            EditorGUILayout.EndHorizontal();

            if ((i + 1) % jointPerLegCount == 0)
            {
                EditorGUILayout.Space();
            }
        }

        EditorGUI.indentLevel--;

        EditorGUILayout.LabelField("[Calibration]", EditorStyles.boldLabel);

        EditorGUILayout.BeginHorizontal();

        if (GUILayout.Button("Reset"))
        {
            quadrupedActuators.ResetButton();
        }

        if (GUILayout.Button("Set to 0"))
        {
            quadrupedActuators.SetTo0();
        }

        if (GUILayout.Button("Set to 30"))
        {
            quadrupedActuators.SetTo30();
        }

        EditorGUILayout.EndHorizontal();
    }
}