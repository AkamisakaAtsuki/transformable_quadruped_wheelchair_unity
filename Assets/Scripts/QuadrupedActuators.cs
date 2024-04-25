using UnityEngine;

[System.Serializable]
public class QuadrupedActuators : MonoBehaviour
{
    [System.Serializable]
    public struct ActuatorParameters
    {
        public float stiffness;
        public float damping;
        public float forceLimit;
        public float speed;
        public float torque;
        public float acceleration;
    }

    /*[System.Serializable]
    public struct PDParameters
    {
        public float P;
        public float D;
    }

    public PDParameters pdParams;*/

    [System.Serializable]
    public struct JointData
    {
        public ArticulationBody articulationBody;
        public bool invertRotationAxis;
        public int rotationOffset;
    }
    public ArticulationBody rootArticulation;
    public ActuatorParameters actuatorParams = new ActuatorParameters();
    public JointData[] joints = new JointData[12];

    public JointData FrontLeftHipJoint => joints[0];
    public JointData FrontLeftUpperJoint => joints[1];
    public JointData FrontLeftLowerJoint => joints[2];
    public JointData FrontRightHipJoint => joints[3];
    public JointData FrontRightUpperJoint => joints[4];
    public JointData FrontRightLowerJoint => joints[5];
    public JointData RearLeftHipJoint => joints[6];
    public JointData RearLeftUpperJoint => joints[7];
    public JointData RearLeftLowerJoint => joints[8];
    public JointData RearRightHipJoint => joints[9];
    public JointData RearRightUpperJoint => joints[10];
    public JointData RearRightLowerJoint => joints[11];

    // 1ステップ前の関節回転角度を保持する配列
    private float[] previousJointAngles;
    private ArticulationBodyReset articulationBodyReset;

    private bool setTo0;
    private bool setTo30;

    private void Start()
    {
        previousJointAngles = new float[joints.Length];
        
        articulationBodyReset = new ArticulationBodyReset();
        articulationBodyReset.InitializeArticulationBodies(rootArticulation.GetComponent<ArticulationBody>());

    }

    private void Update()
    {
        if (setTo0)
        {
            SetAllJointsToTargetDegrees(0f);
        }
        else if (setTo30)
        {
            SetAllJointsToTargetDegrees(30f);
        }
    }
    public float[] GetCurrentJointRotationsInDegrees()
    {
        float[] currentJointRotations = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            float currentRadianRotation = joints[i].articulationBody.jointPosition[0];
            float currentDegreeRotation = Mathf.Rad2Deg * currentRadianRotation;

            // Offset を考慮 (取得時は足す)
            currentJointRotations[i] = currentDegreeRotation + joints[i].rotationOffset;
        }

        return currentJointRotations;
    }

    public void SetJointTargetAngles(float[] targetAngles)
    {
        if ((setTo0 == false) && (setTo30 == false))
        {
            for (int i = 0; i < targetAngles.Length; i++)
            {
                JointData joint = joints[i];
                MoveJoint(
                    joint,
                    targetAngles[i],
                    actuatorParams.stiffness,
                    actuatorParams.damping,
                    actuatorParams.forceLimit
                );
            }
        }
    }

    public float[] GetCurrentNormalizedJointRotationsAndDirections()
    {
        int jointCount = joints.Length;
        float[] jointData = new float[2 * jointCount];

        for (int i = 0; i < jointCount; i++)
        {
            JointData joint = joints[i];
            ArticulationBody articulation = joint.articulationBody;

            float currentRadianRotation = articulation.jointPosition[0];
            float currentDegreeRotation = Mathf.Rad2Deg * currentRadianRotation;

            float normalizedRotation = GetNormalizedValue(currentDegreeRotation,
                                                          articulation.xDrive.lowerLimit,
                                                          articulation.xDrive.upperLimit,
                                                          joint.invertRotationAxis);

            // Store normalized joint rotation
            jointData[i] = normalizedRotation;

            // Store rotation direction (1 for positive, -1 for negative)
            if (normalizedRotation > previousJointAngles[i])
            {
                jointData[jointCount + i] = 1f;
            }
            else
            {
                jointData[jointCount + i] = -1f;
            }

            // Update the previous joint angle for the next iteration
            previousJointAngles[i] = normalizedRotation;
        }

        return jointData;
    }

    private float GetNormalizedValue(float value, float lower, float upper, bool inversion)
    {
        // valueをlowerからupperの範囲で反転するプログラム
        // inversionを指定することで範囲を逆転できる。その場合はvalueをupperからlowerの範囲で反転
        if (inversion == false)   // 値の反転が不必要
        {
            return (value - lower) / (upper - lower);
        }
        else    // 値の反転が必要
        {
            return (upper - value) / (upper - lower);
        }
    }


    public float Map(float value, float start1, float stop1, float start2, float stop2, bool inversion)
    {
        float t = (value - start1) / (stop1 - start1);

        if (inversion)
        {
            return Mathf.Lerp(stop2, start2, t);
        }
        else
        {
            return Mathf.Lerp(start2, stop2, t);
        }
    }

    public void MoveJoint(JointData joint, float target, float stiffness, float damping, float forceLimit)
    {
        ArticulationDrive xDrive = joint.articulationBody.xDrive;

        xDrive.stiffness = stiffness;
        xDrive.damping = damping;
        xDrive.forceLimit = forceLimit;

        if (joint.invertRotationAxis)
        {
            xDrive.target = - target + joint.rotationOffset;
        }
        else
        {
            xDrive.target = target + joint.rotationOffset;
        }
        joint.articulationBody.xDrive = xDrive;
    }

    public void SetAllJointsToTargetDegrees(float targetAngle)
    {
        foreach (JointData joint in joints)
        {
            MoveJoint(
                joint, 
                targetAngle,
                actuatorParams.stiffness,
                actuatorParams.damping,
                actuatorParams.forceLimit
            );
        }
    }

    public void ResetButton()
    {
        rootArticulation.immovable = false;
        articulationBodyReset.ResetArticulationBodies(rootArticulation.gameObject, new Vector3(0f, 0f, 0f));

        setTo0 = false;
        setTo30 = false;
    }
    public void SetTo0()
    {
        ResetButton();
        rootArticulation.immovable = true;
        
        setTo0 = true;
        setTo30 = false;
    }
    public void SetTo30()
    {
        ResetButton();
        rootArticulation.immovable = true;

        setTo0 = false;
        setTo30 = true;
    }
}