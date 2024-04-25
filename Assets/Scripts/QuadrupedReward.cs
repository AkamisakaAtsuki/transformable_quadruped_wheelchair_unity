using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class QuadrupedReward : MonoBehaviour
{
    [System.Serializable]
    public struct ApproachRewardParameters
    {
        public bool use;
        public Transform targetTransform;
        public Transform robotTransform;
        public float rewardCloser;
        public float rewardSame;
        public float rewardFarther;
        public float reward;
    }
    [System.Serializable]
    public struct TargetTouchReardParameters
    {
        public bool use;
        public Transform targetTransform;
        public Transform robotTransform;
        public float touchReward;
        public float touchThreshold;
        public bool drawConcentricCircles;
        public float reward;
    }
    [System.Serializable]
    public struct BoundingBoxTargetTouchRewardParameters
    {
        public bool use;
        public Transform robotTransform;
        public GameObject boundingBox;
        public float touchReward;
        public float reward;
    }
    [System.Serializable]
    public struct LinearVelocityRewardParameters
    {
        public Transform robotTransform;
        public float maxVelocity;
        public bool drawArrows;
        public float reward;
    }
    [System.Serializable]
    public struct AngularVelocityRewardParameters
    {
        public Transform robotTransform;
        public bool drawArrows;
        public float reward;
    }
    [System.Serializable]
    public struct BaseMotionRewardParams
    {
        public Transform robotTransform;
        public float maxVelocity;
        public bool drawArrows;
        public float reward;
    }
    [System.Serializable]
    public struct FallDownRewardParams
    {
        public Transform robotTransform;
        public float fallDownReward;  
        public float notFallDownReward;
        public bool exitWhenFallDown; 
        public float xMin;
        public float xMax;
        public float yMin;
        public float yMax;
        public float zMin;
        public float zMax;
        public float reward;
    }

    public bool endEpisode = false;
    public bool touchTheGoal = false;
    public bool fallDown = false;
    public ApproachRewardParameters approachRewardParams;
    public TargetTouchReardParameters targetTouchReardParams;
    public BoundingBoxTargetTouchRewardParameters boundingBoxTargetTouchRewardParams;
    public LinearVelocityRewardParameters linearVelocityRewardParams;
    public AngularVelocityRewardParameters angularVelocityRewardParams;
    public BaseMotionRewardParams baseMotionRewardParams;
    public FallDownRewardParams fallDownRewardParams;

    private ApproachReward approachReward;
    private TargetTouchReward targetTouchReward;
    private BoundingBoxTargetTouchReward boundingBoxTargetTouchReward;
    private LinearVelocityReward linearVelocityReward;
    private AngularVelocityReward angularVelocityReward;
    private BaseMotionReward baseMotionReward;
    private FallDownReward fallDownReward;

    private QuadrupedSensors quadrupedSensors;

    // Start is called before the first frame update
    void Start()
    {
        quadrupedSensors = this.GetComponent<QuadrupedSensors>();

        approachReward = new ApproachReward();
        targetTouchReward = new TargetTouchReward();
        boundingBoxTargetTouchReward = new BoundingBoxTargetTouchReward();
        linearVelocityReward = new LinearVelocityReward();
        angularVelocityReward = new AngularVelocityReward();
        baseMotionReward = new BaseMotionReward();
        fallDownReward = new FallDownReward();

        approachReward.Initialize(
            approachRewardParams.targetTransform,
            approachRewardParams.robotTransform,
            approachRewardParams.rewardCloser,
            approachRewardParams.rewardSame,
            approachRewardParams.rewardFarther
        );
        /*targetTouchReward.Initialize(
            targetTouchReardParams.targetTransform,
            targetTouchReardParams.robotTransform,
            targetTouchReardParams.touchReward,
            targetTouchReardParams.touchThreshold,
            targetTouchReardParams.drawConcentricCircles
        );*/
        boundingBoxTargetTouchReward.Initialize(
            boundingBoxTargetTouchRewardParams.robotTransform,
            boundingBoxTargetTouchRewardParams.boundingBox,
            boundingBoxTargetTouchRewardParams.touchReward
        );
        linearVelocityReward.Initialize(
            linearVelocityRewardParams.robotTransform,
            linearVelocityRewardParams.maxVelocity,
            linearVelocityRewardParams.drawArrows
        );
        angularVelocityReward.Initialize(
            angularVelocityRewardParams.robotTransform,    
            angularVelocityRewardParams.drawArrows
        );
        baseMotionReward.Initialize(
            baseMotionRewardParams.robotTransform,
            baseMotionRewardParams.maxVelocity,
            baseMotionRewardParams.drawArrows
        );
        fallDownReward.Initialize(
            fallDownRewardParams.robotTransform,
            fallDownRewardParams.fallDownReward,
            fallDownRewardParams.notFallDownReward,
            fallDownRewardParams.exitWhenFallDown,
            fallDownRewardParams.xMin,
            fallDownRewardParams.xMax,
            fallDownRewardParams.yMin,
            fallDownRewardParams.yMax,
            fallDownRewardParams.zMin,
            fallDownRewardParams.zMax
        );
    }

    // Update is called once per frame
    /*    void Update()
        {
            bool endEpi = false;
            approachRewardParams.reward = approachReward.Calculate();
            targetTouchReardParams.reward = targetTouchReward.Calculate(ref endEpi);
        }*/

    public void CalculateAll(JoyMsg joyMsg)  // Vector2 controllerCommand, float controllerCommandTargetAngularVelocity)
    {
        //new Vector2(leftStick[0], leftStick[1]), rightStick[0]

        Vector3Msg baseVelocityRos = quadrupedSensors.GetVelocity();
        Vector3Msg baseAngularVelocityRos = quadrupedSensors.GetAngularVelocity();

        endEpisode = false;
        touchTheGoal = false;
        fallDown = false;
        approachRewardParams.reward = approachReward.Calculate();
        // targetTouchReardParams.reward = targetTouchReward.Calculate(ref touchTheGoal);
        boundingBoxTargetTouchRewardParams.reward = boundingBoxTargetTouchReward.Calculate(ref touchTheGoal);
        linearVelocityRewardParams.reward = linearVelocityReward.Calculate(joyMsg, baseVelocityRos);
        angularVelocityRewardParams.reward = angularVelocityReward.Calculate(joyMsg, baseAngularVelocityRos, false);
        baseMotionRewardParams.reward = baseMotionReward.Calculate(joyMsg, baseVelocityRos, baseAngularVelocityRos);
        fallDownRewardParams.reward = fallDownReward.Calculate(ref fallDown, false);

        endEpisode = touchTheGoal || fallDown;
    }
}