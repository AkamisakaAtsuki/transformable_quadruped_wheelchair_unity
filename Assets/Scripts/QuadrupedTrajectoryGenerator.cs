using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(QuadrupedIK))]
[RequireComponent(typeof(QuadrupedActuators))]
[RequireComponent(typeof(XboxControllerInput))]
[RequireComponent(typeof(TrajectoryVisualizer))]
public class QuadrupedTrajectoryGenerator : MonoBehaviour
{
    public ArticulationBody rootArticulation;

    [System.Serializable]
    public struct QuadrupedFootTrajectoryGeneratorConfig
    {
        public GameObject trajectoryBase;   // 軌道の中心となるベース位置
        public Vector3 offsetPosition;
        public Vector3 offsetRotation;
        public double tgWidthScale;
        public double tgHeightScale;
        public double frequencyOffset;
    }

    public QuadrupedFootTrajectoryGeneratorConfig frontLeft;
    public QuadrupedFootTrajectoryGeneratorConfig frontRight;
    public QuadrupedFootTrajectoryGeneratorConfig rearLeft;
    public QuadrupedFootTrajectoryGeneratorConfig rearRight;

    [HideInInspector]
    public Quaternion initialFrontLeftBaseRotation;
    [HideInInspector]
    public Quaternion initialFrontRightBaseRotation;
    [HideInInspector]
    public Quaternion initialRearLeftBaseRotation;
    [HideInInspector]
    public Quaternion initialRearRightBaseRotation;

    public enum GeneratorType
    {
        Spline,
        Sine,
        Bezier
    }

    public bool xboxControllerMode = false;

    public GeneratorType generatorType;
    public double baseFrequency = 1;

    public double tgWidth = 0.2;
    public double tgHeight = 0.2;

    [HideInInspector]
    public FootTrajectoryGenerator frontLeftTg;
    [HideInInspector]
    public FootTrajectoryGenerator frontRightTg;
    [HideInInspector]
    public FootTrajectoryGenerator rearLeftTg;
    [HideInInspector]
    public FootTrajectoryGenerator rearRightTg;

    private int numLegs = 4;

    private Vector3[] defaultTrajectoryBasePosition;

    private QuadrupedIK quadrupedIK;
    private QuadrupedActuators quadrupedActuators;
    private XboxControllerInput xboxControllerInput;
    private TrajectoryVisualizer trajectoryVisualizer;

    void Start()
    {

        SetGenerator(ref frontLeftTg, generatorType, baseFrequency, 0);
        SetGenerator(ref frontRightTg, generatorType, baseFrequency, Mathf.PI);
        SetGenerator(ref rearLeftTg, generatorType, baseFrequency, Mathf.PI);
        SetGenerator(ref rearRightTg, generatorType, baseFrequency, 0);

        SetTgParams(ref frontLeft, tgWidth, tgHeight);
        SetTgParams(ref frontRight, tgWidth, tgHeight);
        SetTgParams(ref rearLeft, tgWidth, tgHeight);
        SetTgParams(ref rearRight, tgWidth, tgHeight);

        defaultTrajectoryBasePosition = new Vector3[numLegs];
        defaultTrajectoryBasePosition[0] = frontLeft.trajectoryBase.transform.localPosition;  // 軌道のデフォルトの位置及び回転を記録
        defaultTrajectoryBasePosition[1] = frontRight.trajectoryBase.transform.localPosition;  // 軌道のデフォルトの位置及び回転を記録
        defaultTrajectoryBasePosition[2] = rearLeft.trajectoryBase.transform.localPosition;  // 軌道のデフォルトの位置及び回転を記録
        defaultTrajectoryBasePosition[3] = rearRight.trajectoryBase.transform.localPosition;  // 軌道のデフォルトの位置及び回転を記録
        initialFrontLeftBaseRotation = frontLeft.trajectoryBase.transform.localRotation;  // 軌道のデフォルトの位置及び回転を記録
        initialFrontRightBaseRotation = frontRight.trajectoryBase.transform.localRotation;  // 軌道のデフォルトの位置及び回転を記録
        initialRearLeftBaseRotation = rearLeft.trajectoryBase.transform.localRotation;  // 軌道のデフォルトの位置及び回転を記録
        initialRearRightBaseRotation = rearRight.trajectoryBase.transform.localRotation;  // 軌道のデフォルトの位置及び回転を記録
        setTrajectoryBasePosition();
        setTrajectoryBaseRotation();

        quadrupedIK = this.GetComponent<QuadrupedIK>();
        quadrupedActuators = this.GetComponent<QuadrupedActuators>();
        xboxControllerInput = this.GetComponent<XboxControllerInput>();
        trajectoryVisualizer = GetComponent<TrajectoryVisualizer>();

        // Initialize the lineRenderers list.
        trajectoryVisualizer.lineRenderers = new List<LineRenderer>();

        for (int i = 0; i < numLegs; i++)
        {
            // Create a new game object for each line renderer.
            GameObject lineObj = new GameObject("LineRenderer" + i);
            lineObj.transform.parent = rootArticulation.transform;
            // Add the line renderer component to the new game object.
            LineRenderer lr = lineObj.AddComponent<LineRenderer>();

            // Set up the line renderer. Add any additional customization here.
            lr.startWidth = 0.01f;
            lr.endWidth = 0.01f;

            // Add the line renderer to the list.
            trajectoryVisualizer.lineRenderers.Add(lr);
        }
    }

    public void SetGenerator(ref FootTrajectoryGenerator generator, GeneratorType type, double baseFrequency, double initialPhi)
    {
        generatorType = type;
        switch (generatorType)
        {
            case GeneratorType.Spline:
                generator = new SplineFootTrajectoryGenerator(baseFrequency, initialPhi);
                break;
            case GeneratorType.Sine:
                generator = new SineFootTrajectoryGenerator(baseFrequency, initialPhi);
                break;
            case GeneratorType.Bezier:
                generator = new BezierFootTrajectoryGenerator(baseFrequency, initialPhi);
                break;
            default:
                generator = null;
                break;
        }
    }

    public void SetTgParams(ref QuadrupedFootTrajectoryGeneratorConfig quadrupedFootTrajectoryGeneratorConfig, double tgWidth, double tgHeight)
    {
        quadrupedFootTrajectoryGeneratorConfig.tgWidthScale = tgWidth;
        quadrupedFootTrajectoryGeneratorConfig.tgHeightScale = tgHeight;
    }

    /* void updateTrajectory()
    {
        // 軌道全体の移動や回転の変更
        frontLeft.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[0] + frontLeft.offsetPosition;
        frontLeft.trajectoryBase.transform.localRotation = Quaternion.Euler(frontLeft.offsetRotation);

        frontRight.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[1] + frontRight.offsetPosition;
        frontRight.trajectoryBase.transform.localRotation = Quaternion.Euler(frontRight.offsetRotation);

        rearLeft.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[2] + rearLeft.offsetPosition;
        rearLeft.trajectoryBase.transform.localRotation = Quaternion.Euler(rearLeft.offsetRotation);

        rearRight.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[3] + rearRight.offsetPosition;
        rearRight.trajectoryBase.transform.localRotation = Quaternion.Euler(rearRight.offsetRotation);
    } */

    public void setTrajectoryBasePosition()
    {
        frontLeft.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[0] + frontLeft.offsetPosition;
        frontRight.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[1] + frontRight.offsetPosition;
        rearLeft.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[2] + rearLeft.offsetPosition;
        rearRight.trajectoryBase.transform.localPosition = defaultTrajectoryBasePosition[3] + rearRight.offsetPosition;
    }

    public void setTrajectoryBaseRotation()
    {
        frontLeft.trajectoryBase.transform.localRotation = Quaternion.Euler(frontLeft.offsetRotation);
        frontRight.trajectoryBase.transform.localRotation = Quaternion.Euler(frontRight.offsetRotation);
        rearLeft.trajectoryBase.transform.localRotation = Quaternion.Euler(rearLeft.offsetRotation);
        rearRight.trajectoryBase.transform.localRotation = Quaternion.Euler(rearRight.offsetRotation);
    }

    // 4脚ロボットを仮定
    public Vector3[] getTrotTrajectory(double time, float moveAngle, float moveSpeedScale, float lookDirection, bool stop)
    {
        Vector3[] targetPoint = new Vector3[numLegs];

        (float resultSpeedScaleFrontLeft, float directionAngleFrontLeft) = AddPolarVectors(moveSpeedScale, moveAngle, lookDirection, 45 * Mathf.Deg2Rad); // Front Left
        (float resultSpeedScaleFrontRight, float directionAngleFrontRight) = AddPolarVectors(moveSpeedScale, moveAngle, lookDirection, 135 * Mathf.Deg2Rad); // Front Right
        (float resultSpeedScaleRearLeft, float directionAngleRearLeft) = AddPolarVectors(moveSpeedScale, moveAngle, lookDirection, -45 * Mathf.Deg2Rad); // Rear Left
        (float resultSpeedScaleRearRight, float directionAngleRearRight) = AddPolarVectors(moveSpeedScale, moveAngle, lookDirection, -135 * Mathf.Deg2Rad); // Rear Right

        resultSpeedScaleFrontLeft = Mathf.Clamp(resultSpeedScaleFrontLeft, 0, 1);
        resultSpeedScaleFrontRight = Mathf.Clamp(resultSpeedScaleFrontRight, 0, 1);
        resultSpeedScaleRearLeft = Mathf.Clamp(resultSpeedScaleRearLeft, 0, 1);
        resultSpeedScaleRearRight = Mathf.Clamp(resultSpeedScaleRearRight, 0, 1);

        // 回転させる
        frontLeft.trajectoryBase.transform.localRotation = initialFrontLeftBaseRotation * Quaternion.Euler(frontLeft.offsetRotation) * Quaternion.Euler(0, directionAngleFrontLeft * Mathf.Rad2Deg - 90, 0);
        frontRight.trajectoryBase.transform.localRotation = initialFrontRightBaseRotation * Quaternion.Euler(frontRight.offsetRotation) * Quaternion.Euler(0, directionAngleFrontRight * Mathf.Rad2Deg - 90, 0);
        rearLeft.trajectoryBase.transform.localRotation = initialRearLeftBaseRotation * Quaternion.Euler(rearLeft.offsetRotation) * Quaternion.Euler(0, directionAngleRearLeft * Mathf.Rad2Deg - 90, 0);
        rearRight.trajectoryBase.transform.localRotation = initialRearRightBaseRotation * Quaternion.Euler(rearRight.offsetRotation) * Quaternion.Euler(0, directionAngleRearRight * Mathf.Rad2Deg - 90, 0);

        Vector3 frontLeftendpointTarget = frontLeftTg.ComputeTrajectory(time, frontLeft.frequencyOffset, frontLeft.tgWidthScale * resultSpeedScaleFrontLeft, frontLeft.tgHeightScale, stop);  //.displayTrajectoryObject[tPoint % numOfPoints].GetComponent<Transform>().localPosition;
        Vector3 frontRightendpointTarget = frontRightTg.ComputeTrajectory(time, frontRight.frequencyOffset, frontRight.tgWidthScale * resultSpeedScaleFrontRight, frontRight.tgHeightScale, stop);
        Vector3 rearLeftendpointTarget = rearLeftTg.ComputeTrajectory(time, rearLeft.frequencyOffset, rearLeft.tgWidthScale * resultSpeedScaleRearLeft, rearLeft.tgHeightScale, stop);
        Vector3 rearRightendpointTarget = rearRightTg.ComputeTrajectory(time, rearRight.frequencyOffset, rearRight.tgWidthScale * resultSpeedScaleRearRight, rearRight.tgHeightScale, stop);

        targetPoint[0] = frontLeft.trajectoryBase.transform.localRotation * frontLeftendpointTarget + frontLeft.trajectoryBase.transform.localPosition;
        targetPoint[1] = frontRight.trajectoryBase.transform.localRotation * frontRightendpointTarget + frontRight.trajectoryBase.transform.localPosition;
        targetPoint[2] = rearLeft.trajectoryBase.transform.localRotation * rearLeftendpointTarget + rearLeft.trajectoryBase.transform.localPosition;
        targetPoint[3] = rearRight.trajectoryBase.transform.localRotation * rearRightendpointTarget + rearRight.trajectoryBase.transform.localPosition;

        // 軌道の可視化を更新
        trajectoryVisualizer.UpdateTrajectoryVisualizer(new List<Vector3>(targetPoint));

        return targetPoint;
    }

    public Vector3[] getTrotTrajectoryForward(double time, float joyXaxis, float joyYaxis, float widthScale, bool stop)
    {
        // 直進と回転のTGを作成するだけ
        // ジョイスティックの値でwidthのみを変更
        Vector3[] targetPoint = new Vector3[numLegs];

        Vector3[] straightTrotTargetPoints = getStraightTrot(time, widthScale, false, false);
        Vector3[] rotationTrotTargetPoints = getRotationTrot(time, widthScale, false, false);

        for (int i = 0; i < numLegs; i++)
        {
            float yComponent = straightTrotTargetPoints[i].y;  // Preserve the z-component from straightTrotTargetPoints
            targetPoint[i] = joyYaxis * straightTrotTargetPoints[i] + joyXaxis * rotationTrotTargetPoints[i];
            targetPoint[i].y = yComponent;  // Set the z-component to the preserved value
        }

        targetPoint[0] += frontLeft.trajectoryBase.transform.localPosition;
        targetPoint[1] += frontRight.trajectoryBase.transform.localPosition;
        targetPoint[2] += rearLeft.trajectoryBase.transform.localPosition;
        targetPoint[3] += rearRight.trajectoryBase.transform.localPosition;

        // 軌道の可視化を更新
        trajectoryVisualizer.UpdateTrajectoryVisualizer(new List<Vector3>(targetPoint));

        return targetPoint;
    }

    public Vector3[] getTrotTrajectoryBack(double time, float joyXaxis, float joyYaxis, bool stop)
    {
        // 直進と回転のTGを作成するだけ
        // ジョイスティックの値でwidthのみを変更
        Vector3[] targetPoint = new Vector3[numLegs];

        Vector3[] straightTrotTargetPoints = getStraightTrot(time, -1f, false, false);
        Vector3[] rotationTrotTargetPoints = getRotationTrot(time, -1f, false, false);

        for (int i = 0; i < numLegs; i++)
        {
            float yComponent = straightTrotTargetPoints[i].y;  // Preserve the z-component from straightTrotTargetPoints
            targetPoint[i] = -joyYaxis * straightTrotTargetPoints[i] + joyXaxis * rotationTrotTargetPoints[i];
            targetPoint[i].y = yComponent;  // Set the z-component to the preserved value
        }

        targetPoint[0] += frontLeft.trajectoryBase.transform.localPosition;
        targetPoint[1] += frontRight.trajectoryBase.transform.localPosition;
        targetPoint[2] += rearLeft.trajectoryBase.transform.localPosition;
        targetPoint[3] += rearRight.trajectoryBase.transform.localPosition;

        // 軌道の可視化を更新
        trajectoryVisualizer.UpdateTrajectoryVisualizer(new List<Vector3>(targetPoint));

        return targetPoint;
    }


    public Vector3[] getStraightTrot(double time, float widthScale, bool stop, bool visualize)
    {
        Vector3[] targetPoint = new Vector3[numLegs];

        // 回転させる
        frontLeft.trajectoryBase.transform.localRotation = initialFrontLeftBaseRotation * Quaternion.Euler(frontLeft.offsetRotation) * Quaternion.Euler(0, -90, 0); ;
        frontRight.trajectoryBase.transform.localRotation = initialFrontRightBaseRotation * Quaternion.Euler(frontRight.offsetRotation) * Quaternion.Euler(0, -90, 0); ;
        rearLeft.trajectoryBase.transform.localRotation = initialRearLeftBaseRotation * Quaternion.Euler(rearLeft.offsetRotation) * Quaternion.Euler(0, -90, 0); ;
        rearRight.trajectoryBase.transform.localRotation = initialRearRightBaseRotation * Quaternion.Euler(rearRight.offsetRotation) * Quaternion.Euler(0, -90, 0); ;

        Vector3 frontLeftendpointTarget = frontLeftTg.ComputeTrajectory(time, frontLeft.frequencyOffset, frontLeft.tgWidthScale * widthScale, frontLeft.tgHeightScale, stop);  //.displayTrajectoryObject[tPoint % numOfPoints].GetComponent<Transform>().localPosition;
        Vector3 frontRightendpointTarget = frontRightTg.ComputeTrajectory(time, frontRight.frequencyOffset, frontRight.tgWidthScale * widthScale, frontRight.tgHeightScale, stop);
        Vector3 rearLeftendpointTarget = rearLeftTg.ComputeTrajectory(time, rearLeft.frequencyOffset, rearLeft.tgWidthScale * widthScale, rearLeft.tgHeightScale, stop);
        Vector3 rearRightendpointTarget = rearRightTg.ComputeTrajectory(time, rearRight.frequencyOffset, rearRight.tgWidthScale * widthScale, rearRight.tgHeightScale, stop);

        targetPoint[0] = frontLeft.trajectoryBase.transform.localRotation * frontLeftendpointTarget;
        targetPoint[1] = frontRight.trajectoryBase.transform.localRotation * frontRightendpointTarget;
        targetPoint[2] = rearLeft.trajectoryBase.transform.localRotation * rearLeftendpointTarget;
        targetPoint[3] = rearRight.trajectoryBase.transform.localRotation * rearRightendpointTarget;

        if (visualize)
        {
            // 軌道の可視化を更新
            trajectoryVisualizer.UpdateTrajectoryVisualizer(new List<Vector3>(targetPoint));
        }

        return targetPoint;
    }

    public Vector3[] getRotationTrot(double time, float widthScale, bool stop, bool visualize)
    {
        Vector3[] targetPoint = new Vector3[numLegs];

        // 回転させる
        frontLeft.trajectoryBase.transform.localRotation = initialFrontLeftBaseRotation * Quaternion.Euler(frontLeft.offsetRotation) * Quaternion.Euler(0, -45, 0); ;
        frontRight.trajectoryBase.transform.localRotation = initialFrontRightBaseRotation * Quaternion.Euler(frontRight.offsetRotation) * Quaternion.Euler(0, 45, 0); ;
        rearLeft.trajectoryBase.transform.localRotation = initialRearLeftBaseRotation * Quaternion.Euler(rearLeft.offsetRotation) * Quaternion.Euler(0, -135, 0); ;
        rearRight.trajectoryBase.transform.localRotation = initialRearRightBaseRotation * Quaternion.Euler(rearRight.offsetRotation) * Quaternion.Euler(0, 135, 0); ;

        Vector3 frontLeftendpointTarget = frontLeftTg.ComputeTrajectory(time, frontLeft.frequencyOffset, frontLeft.tgWidthScale * widthScale, frontLeft.tgHeightScale, stop);  //.displayTrajectoryObject[tPoint % numOfPoints].GetComponent<Transform>().localPosition;
        Vector3 frontRightendpointTarget = frontRightTg.ComputeTrajectory(time, frontRight.frequencyOffset, frontRight.tgWidthScale * widthScale, frontRight.tgHeightScale, stop);
        Vector3 rearLeftendpointTarget = rearLeftTg.ComputeTrajectory(time, rearLeft.frequencyOffset, rearLeft.tgWidthScale * widthScale, rearLeft.tgHeightScale, stop);
        Vector3 rearRightendpointTarget = rearRightTg.ComputeTrajectory(time, rearRight.frequencyOffset, rearRight.tgWidthScale * widthScale, rearRight.tgHeightScale, stop);

        targetPoint[0] = frontLeft.trajectoryBase.transform.localRotation * frontLeftendpointTarget;
        targetPoint[1] = frontRight.trajectoryBase.transform.localRotation * frontRightendpointTarget;
        targetPoint[2] = rearLeft.trajectoryBase.transform.localRotation * rearLeftendpointTarget;
        targetPoint[3] = rearRight.trajectoryBase.transform.localRotation * rearRightendpointTarget;

        if (visualize)
        {
            // 軌道の可視化を更新
            trajectoryVisualizer.UpdateTrajectoryVisualizer(new List<Vector3>(targetPoint));
        }

        return targetPoint;
    }

    private (float magnitude, float angle) AddPolarVectors(float moveSpeedScale, float moveAngle, float lookSpeedScale, float lookAngle)
    {
        // 極座標を直交座標に変換
        Vector2 moveVector = new Vector2(moveSpeedScale * Mathf.Cos(moveAngle), moveSpeedScale * Mathf.Sin(moveAngle));
        Vector2 lookVector = new Vector2(lookSpeedScale * Mathf.Cos(lookAngle), lookSpeedScale * Mathf.Sin(lookAngle));

        // 直交座標で足し算
        Vector2 resultVector = moveVector + lookVector;

        // 結果を極座標に変換
        float resultMagnitude = resultVector.magnitude;
        float resultAngle = Mathf.Atan2(resultVector.y, resultVector.x);　　// Unityの座標系の関係上、xとyを逆にしている

        return (resultMagnitude, resultAngle);
    }

    private float[] Linspace(float minValue, float maxValue, int size)
    {
        float[] list = new float[size];

        float delta = (maxValue - minValue) / (size - 1);

        for (int i=0; i<size; i++)
        {
            list[i] = minValue + delta * i;
        }

        return list;
    }
    private void Update()
    {
        // Xboxコントローラの制御
        if (xboxControllerMode)
        {
            float[] leftStick = xboxControllerInput.GetLeftStickPolarCoordinates();
            float[] rightStick = xboxControllerInput.GetRightStick();

            // TGから目標点を抽出する
            Vector3[] tgPosition = getTrotTrajectory(Time.time, leftStick[0], leftStick[1], rightStick[0], false);

            /*// Update LineRenderer positions
            for (int i = 0; i < numLegs; i++)
            {
                Vector3 currentPosition = defaultTrajectoryBasePosition[i];
                Vector3 targetPosition = tgPosition[i];

                LineRenderer lr = trajectoryVisualizer.lineRenderers[i];
                *//*lr.SetPosition(0, currentPosition);
                lr.SetPosition(1, targetPosition);*//*
                Vector3 currentPositionWorld = rootArticulation.transform.TransformPoint(currentPosition);
                Vector3 targetPositionWorld = rootArticulation.transform.TransformPoint(targetPosition);
                lr.SetPosition(0, currentPositionWorld);
                lr.SetPosition(1, targetPositionWorld);
            }*/

            float[] tgAngles = quadrupedIK.GetJointAngles(tgPosition);
            quadrupedActuators.SetJointTargetAngles(tgAngles);
        }
    }
}
