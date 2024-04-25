using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

[RequireComponent(typeof(QuadrupedActuators))]

public class QuadrupedIK : MonoBehaviour
{
    public bool DEBUG = false;
    public QuadrupedConfig quadrupedConfig;

    private float l1;
    private float l2;
    private float l3;
    private float l4;

    public GameObject frontLeftLegToe;
    public GameObject frontRightLegToe;
    public GameObject rearLeftLegToe;
    public GameObject rearRightLegToe;

    public enum LegType { frontLeft, frontRight, rearLeftType1, rearRightType1, rearLeftType2, rearRightType2 }; // Unitree Go1を前提としてタイプを設定

    // inspectorタブでは以下の構造体のnameで与えたものが、TrajectoryGeneratorスクリプトのinspectorの該当部分と同じ順番に並び変えること
    [System.Serializable]
    private struct Dof3Legs
    {
        public GameObject startPoint;
        public GameObject firstJoint; 
        public GameObject secondJoint;  
        public GameObject thirdJoint;
        public GameObject endPoint; 
        public LegType legType;
    }

    // 3自由度の脚のロボットにしか対応しない


    private Dof3Legs frontLeftLeg;
    private Dof3Legs frontRightLeg;
    private Dof3Legs rearLeftLeg;
    private Dof3Legs rearRightLeg;

    public LegType frontLeftLegType;
    public LegType frontRightLegType;
    public LegType rearLeftLegType;
    public LegType rearRightLegType;

    private float[] previousJointAngles = new float[12];
    private float thresholdAngleChange = 180.0f; // 直前から大きな角度変化がある場合にその値を採用しないようにするための閾値を設定


    private QuadrupedActuators quadrupedActuators;

    private void Start()
    {
        l1 = quadrupedConfig.l1;
        l2 = quadrupedConfig.l2;
        l3 = quadrupedConfig.l3;
        l4 = quadrupedConfig.l4;

        quadrupedActuators = this.GetComponent<QuadrupedActuators>();
        InitializeLeg(
            ref frontLeftLeg, 
            frontLeftLegType,
            quadrupedActuators.FrontLeftHipJoint.articulationBody.gameObject,
            quadrupedActuators.FrontLeftUpperJoint.articulationBody.gameObject,
            quadrupedActuators.FrontLeftLowerJoint.articulationBody.gameObject,
            frontLeftLegToe
        );
        InitializeLeg(
            ref frontRightLeg, 
            frontRightLegType,
            quadrupedActuators.FrontRightHipJoint.articulationBody.gameObject,
            quadrupedActuators.FrontRightUpperJoint.articulationBody.gameObject,
            quadrupedActuators.FrontRightLowerJoint.articulationBody.gameObject,
            frontRightLegToe
        );
        InitializeLeg(
            ref rearLeftLeg, 
            rearLeftLegType,
            quadrupedActuators.RearLeftHipJoint.articulationBody.gameObject,
            quadrupedActuators.RearLeftUpperJoint.articulationBody.gameObject,
            quadrupedActuators.RearLeftLowerJoint.articulationBody.gameObject,
            rearLeftLegToe
        );
        InitializeLeg(
            ref rearRightLeg, 
            rearRightLegType, 
            quadrupedActuators.RearRightHipJoint.articulationBody.gameObject,
            quadrupedActuators.RearRightUpperJoint.articulationBody.gameObject,
            quadrupedActuators.RearRightLowerJoint.articulationBody.gameObject,
            rearRightLegToe
        );
    }

    public void ikReset()
    {

        for (int i = 0; i < previousJointAngles.Length; i++)
        {
            previousJointAngles[i] = 0.0f;
        }
    }

    private void InitializeLeg(ref Dof3Legs leg, LegType legType, GameObject hipLink, GameObject upperLink, GameObject lowerLink, GameObject toe)
    {
        leg.startPoint = hipLink;
        leg.firstJoint = hipLink;
        leg.secondJoint = upperLink;
        leg.thirdJoint = lowerLink;
        leg.endPoint = toe;
        leg.legType = legType;
    }

    private float leftSign = 1;
    private float forwardSign = 1;
    private float typeSign = 1;

    public float[] SolveQuadrupedIK(Vector3 targetTrajectory, Vector3 trajectoryBaseLocalPosition, LegType legType)
    {
        UpdateSigns(legType);

        float[] jointAngles = new float[3];

        // 1. Check if the target is within reachable range
        float distanceToTarget = Vector3.Distance(targetTrajectory, trajectoryBaseLocalPosition);   // ヒップから見た目標位置
        float maxReach = Mathf.Sqrt(l1 * l1 + l2 * l2 + Mathf.Pow(l3 + l4, 2)); // ヒップから見たエンドポイントの最高長（最短位置を基準。細かいことを言うと、間接角度を考慮すれば最大長は、変化）
        float minReach = Mathf.Sqrt(l1 * l1 + l2 * l2); // 最短距離を近似的に計算。l3とl4は太ももとふくらはぎ部分の長さになるが、しゃがんだときに角度0になると仮定することで、l1とl2のみを考えることにした（大雑把すぎ）
        if (distanceToTarget > maxReach || distanceToTarget < minReach)
        {
            if (DEBUG)
            {
                // Handle error: Target is out of reach
                Debug.Log("目標値に到達することができません");
            }
            return null; // or some default values or throw an exception
        }


        float x2 = Mathf.Pow(targetTrajectory.x - trajectoryBaseLocalPosition.x, 2);
        float y2 = Mathf.Pow(targetTrajectory.y - trajectoryBaseLocalPosition.y, 2);
        float zL = targetTrajectory.z - trajectoryBaseLocalPosition.z - forwardSign * l1;
        float zL2 = Mathf.Pow(zL, 2);

        float cosTheta3 = (x2 + y2 + zL2 - Mathf.Pow(l2, 2) - Mathf.Pow(l3, 2) - Mathf.Pow(l4, 2)) / (2 * l3 * l4);

        /*// 2. Check if cosTheta3 is within valid range
        if (cosTheta3 < -1.0f || cosTheta3 > 1.0f)
        {
            // Handle error: Invalid position
            Debug.Log("CosTheta3の値範囲がおかしいため計算を続けられません");
            return null; // or some default values or throw an exception
        }*/

        float theta3 = -typeSign * Mathf.Acos(cosTheta3);
        float sinTheta3 = Mathf.Sin(theta3);

        // Debug.Log($"zL: {zL}, 分母: {Mathf.Sqrt(x2 + y2 + zL2 - Mathf.Pow(l2, 2))}");
        // Debug.Log($"Mathf.Atan((l4 * sinTheta3) / (l3 + l4 * cosTheta3): {Mathf.Atan((l4 * sinTheta3) / (l3 + l4 * cosTheta3))}");

        float theta2 = -Mathf.Asin(zL / Mathf.Sqrt(x2 + y2 + zL2 - Mathf.Pow(l2, 2))) - Mathf.Atan((l4 * sinTheta3) / (l3 + l4 * cosTheta3));
        float sinTheta2 = Mathf.Sin(theta2);
        float cosTheta2 = Mathf.Cos(theta2);
        // Debug.Log($"theta2: {theta2}");

        float A = l3 * cosTheta2 + l4 * cosTheta2 * cosTheta3 - l4 * sinTheta2 * sinTheta3;
        float theta1 = Mathf.Asin((targetTrajectory.x - trajectoryBaseLocalPosition.x) / Mathf.Sqrt(Mathf.Pow(A, 2) + Mathf.Pow(l2, 2))) + leftSign * Mathf.Atan(l2 / A);

        jointAngles[0] = theta1;
        jointAngles[1] = theta2;
        jointAngles[2] = theta3;

        // 無限大やNaNチェック(その場合はエディタを強制停止)
        if (ContainsInvalidValue(jointAngles))
        {
            if (DEBUG)
            {
                Debug.LogError("One or more joint angles are invalid (Infinity/NaN). Stopping the editor.");
            }
            #if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
            #endif
        }

        return jointAngles;
    }

    bool ContainsInvalidValue(float[] angles)
    {
        foreach (float angle in angles)
        {
            if (float.IsInfinity(angle) || float.IsNaN(angle))
            {
                return true;
            }
        }
        return false;
    }

    private void UpdateSigns(LegType legType)
    {
        switch (legType)
        {
            case LegType.frontLeft:
                leftSign = 1;
                forwardSign = 1;
                typeSign = 1;
                break;
            case LegType.frontRight:
                leftSign = -1;
                forwardSign = 1;
                typeSign = 1;
                break;
            case LegType.rearLeftType1:
                leftSign = 1;
                forwardSign = -1;
                typeSign = 1;
                break;
            case LegType.rearRightType1:
                leftSign = -1;
                forwardSign = -1;
                typeSign = 1;
                break;
            case LegType.rearLeftType2:
                leftSign = 1;
                forwardSign = -1;
                typeSign = -1;
                break;
            case LegType.rearRightType2:
                leftSign = -1;
                forwardSign = -1;
                typeSign = -1;
                break;
        }
    }

    public float[] GetJointAngles(Vector3[] endPointTarget)
    {
        // 目標位置が動作可能範囲外の場合、nullが返されるため、その場合は、直前の値を返すようにする
        float[] jointAngles = new float[12];

        /* float[] frontLeftLegJointAngles = SolveQuadrupedIK(endPointTarget[0], frontLeftLeg.startPoint.transform.localPosition, frontLeftLeg.legType);
        float[] frontRightLegJointAngles = SolveQuadrupedIK(endPointTarget[1], frontRightLeg.startPoint.transform.localPosition, frontRightLeg.legType);
        float[] rearLeftLegJointAngles = SolveQuadrupedIK(endPointTarget[2], rearLeftLeg.startPoint.transform.localPosition, rearLeftLeg.legType);
        float[] rearRightLegJointAngles = SolveQuadrupedIK(endPointTarget[3], rearRightLeg.startPoint.transform.localPosition, rearRightLeg.legType); */

        float[] newFrontLeftLegJointAngles = SolveQuadrupedIK(endPointTarget[0], frontLeftLeg.startPoint.transform.localPosition, frontLeftLeg.legType);
        float[] frontLeftLegJointAngles = CheckAngleChange(newFrontLeftLegJointAngles, 0);

        float[] newFrontRightLegJointAngles = SolveQuadrupedIK(endPointTarget[1], frontRightLeg.startPoint.transform.localPosition, frontRightLeg.legType);
        float[] frontRightLegJointAngles = CheckAngleChange(newFrontRightLegJointAngles, 3);

        float[] newRearLeftLegJointAngles = SolveQuadrupedIK(endPointTarget[2], rearLeftLeg.startPoint.transform.localPosition, rearLeftLeg.legType);
        float[] rearLeftLegJointAngles = CheckAngleChange(newRearLeftLegJointAngles, 6);

        float[] newRearRightLegJointAngles = SolveQuadrupedIK(endPointTarget[3], rearRightLeg.startPoint.transform.localPosition, rearRightLeg.legType);
        float[] rearRightLegJointAngles = CheckAngleChange(newRearRightLegJointAngles, 9);

        previousJointAngles = (float[])jointAngles.Clone();

        // Fill the jointAngles with the new values or the previous ones if they're null
        jointAngles[0] = Mathf.Rad2Deg * frontLeftLegJointAngles[0];
        jointAngles[1] = Mathf.Rad2Deg * frontLeftLegJointAngles[1];
        jointAngles[2] = Mathf.Rad2Deg * frontLeftLegJointAngles[2];
        jointAngles[3] = Mathf.Rad2Deg * frontRightLegJointAngles[0];
        jointAngles[4] = Mathf.Rad2Deg * frontRightLegJointAngles[1];
        jointAngles[5] = Mathf.Rad2Deg * frontRightLegJointAngles[2];
        jointAngles[6] = Mathf.Rad2Deg * rearLeftLegJointAngles[0];
        jointAngles[7] = Mathf.Rad2Deg * rearLeftLegJointAngles[1];
        jointAngles[8] = Mathf.Rad2Deg * rearLeftLegJointAngles[2];
        jointAngles[9] = Mathf.Rad2Deg * rearRightLegJointAngles[0];
        jointAngles[10] = Mathf.Rad2Deg * rearRightLegJointAngles[1];
        jointAngles[11] = Mathf.Rad2Deg * rearRightLegJointAngles[2];

        return jointAngles;
    }

    private float[] CheckAngleChange(float[] newJointAngles, int startIndex)
    {
        if (newJointAngles == null)
        {
            return new float[] { previousJointAngles[startIndex], previousJointAngles[startIndex + 1], previousJointAngles[startIndex + 2] };
        }

        // 大きな変化が起こりそうな場合、予め察知して直前の値にする。
        /*for (int i = 0; i < 3; i++)
        {
            if (Mathf.Abs(Mathf.Rad2Deg * newJointAngles[i] - previousJointAngles[startIndex + i]) > thresholdAngleChange)
            {
                // Joint anglesのログ出力
                return new float[] { previousJointAngles[startIndex], previousJointAngles[startIndex + 1], previousJointAngles[startIndex + 2] };
            }
        }*/

        return newJointAngles;
    }
}
