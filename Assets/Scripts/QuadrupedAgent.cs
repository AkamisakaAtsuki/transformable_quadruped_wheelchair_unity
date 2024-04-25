using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using System.Linq;
using System;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;


[RequireComponent(typeof(QuadrupedTrajectoryGenerator))]
[RequireComponent(typeof(QuadrupedIK))]
[RequireComponent(typeof(XboxControllerInput))]
[RequireComponent(typeof(QuadrupedActuators))]
[RequireComponent(typeof(QuadrupedSensors))]
[RequireComponent(typeof(DecisionRequester))]
[RequireComponent(typeof(QuadrupedReward))]

public class QuadrupedAgent : Agent
{
    public GameObject rootArticulation;
    public GameObject Target;

    // public GameObject leftLidar;
    // public GameObject rightLidar;

    public Transform QuadrupedRobotTransform;
    public GameObject Humanoid;
    public GameObject leftUpLeg;
    public GameObject leftLeg;
    public GameObject rightUpLeg;
    public GameObject rightLeg;
    public float humanoidMaxWeight;
    // public BoxCollider humanoidBoxCollider;
    //public GameObject Stairs;

    public int numGoals;

    public bool tgMode = false;  // TG�ɂ����s
    public bool chVehiclePose = false;   // �ԗ����[�h�֕ϐg
    public bool chTgPose = false; // TG�ɂ����s�̂��߂̃��[�h�֕ϐg
    public bool foldTheChair = false; // �֎q��܂肽���ރ��[�h
    public bool turn;
    [Range(-1000f, 1000f)] public float go;

    [Range(0, 1f)] public float tgWidthScale = 0.5f;


    public float maxHumanWeight = 10f;
    public float maxStairsScale = 0.4f;


    // ゴールの中心座標
    public Vector3 centerPosition = new Vector3(0, 0, 0);
    public float rotationR = 1f;

    // LiDAR���p�̗L��
    public bool useLiDAR;
    public enum JoyMode
    {
        Autonomous,
        Straight
    }
    public JoyMode joyMode = JoyMode.Autonomous;

    public enum WheelChairMode
    {
        None,
        Default,
        UpStair,
        DownStair,
    }
    public WheelChairMode wheelChairMode = WheelChairMode.None;

    public enum EnvType
    {
        StopEnvironment,
        LinearStairClimbingWithNoPassenger,
        LinearStairClimbingWithPassenger,
        LinearStairDescendingWithNoPassenger,
        LinearStairDescendingWithPassenger,
        SpiralStairClimbingWithNoPassenger,
        SpiralStairClimbingWithPassenger,
        SpiralStairDescendingWithNoPassenge,
        SpiralStairDescendingWithPassenger,
    }

    /*public enum SpiralType
    {
        SpiralStaircase0,
        SpiralStaircase1,
        SpiralStaircase2,
        SpiralStaircase3,
        SpiralStaircase4,
        SpiralStaircase5,
        SpiralStaircase6,
        SpiralStaircase7,
        SpiralStaircase8,
        SpiralStaircase9,
    }*/

    public EnvType envType = EnvType.StopEnvironment;
    // public SpiralType spiralType = SpiralType.SpiralStaircase0;

    public TextAsset curriculumFile; // Unity�G�f�B�^����A�^�b�`����CSV�`���̃J���L�������t�@�C��

    public GameObject linearStaircase;
    public GameObject spiralStaircase;

    private int nEpisode = 0;
    private int nGoal = 0;
    private bool goalFlag = false;

    private Vector3 buffRotation;
    private Vector3 buffPosition;

    private Quaternion humanoidRotReset;
    private Vector3 humanoidPosReset;

    private Vector3 robotFaceVector;

    private GeneralQuadrupedController generalQuadrupedController;
    private QuadrupedTrajectoryGenerator quadrupedTrajectoryGenerator;
    private QuadrupedIK quadrupedIK;
    private PoseRuleController poseRuleController;
    private ChairController chairController;
    private QuadrupedActuators quadrupedActuators;
    private QuadrupedSensors quadrupedSensors;
    private QuadrupedReward quadrupedReward;
    private RobotNavigation robotNavigation;
    private SearchOptimalTrajectoryGenerator searchOptimalTrajectoryGenerator;
    // private Lidar leftLidarSensor;
    // private Lidar rightLidarSensor;

    private ArticulationBodyReset articulationBodyReset;
    private XboxControllerInput xboxControllerInput;

    //private Curriculum curriculum;
    // private LinearStaircaseClimbingCurriculum linearStaircaseClimbingCurriculum;
    private LinearStaircaseClimbingCurriculum curriculumManager;
    //private LinearStaircaseDescendingCurriculum linearStaircaseDescendingCurriculum;

    private const float LidarDistanceThreshold = 1.5f;
    private const float RotationNormalizationFactor = 180f;
    private const float PositionAccelerationNormalizationFactor = 5f;
    private const float MaxHumanWeightLimit = 50f;
    private const float MaxStairsScaleLimit = 0.6f;
    private const int NumberOfGoalsToIncreaseDifficulty = 8;

    private ImuMsg imuData;
    private Vector3 direction;
    private Vector3 unitVectorToDirection;

    private Vector3Msg frontLeftForce;
    private Vector3Msg frontRightForce;
    private Vector3Msg rearLeftForce;
    private Vector3Msg rearRightForce;
    private float[] jointRotation;
    private float angle;

    // TG�̃T�[�`�Ŏg�p
    private int currentFrontPitch;
    private int currentRearPitch;
    private float currentFrontX;
    private float currentFrontZ;
    private float currentRearX;
    private float currentRearZ;
    private int numberOfEpisodesPerConfig = 10; // TG���T�[�`����ۂɁA�C�ӂ̐ݒ�̃G�s�\�[�h���s��

    // �J���L�������֘A
    private float riserRatio = 1f;  // �J���L���������g���Ȃ��ꍇ�̓f�t�H���g�l��1�{
    private float weightRatio = 1f;  // �J���L���������g���Ȃ��ꍇ�̓f�t�H���g�l��1�{
    private float goalDistanceFactor = 1f;  // �J���L���������g���Ȃ��ꍇ�̓f�t�H���g�l��1�{
    public bool useCurriculum = false;
    public int startCurriculumNumber = 0;
    private int currentSteps = 0;
    private int totalStepCount = 0;

    private bool firstEpisode = true;

    public int firstStopStepCounts = 0;

    private float episodeStartTime;

    public class JoyData
    {
        public JoyMsg JoyMsg { get; set; }
        public float[] LeftStickPolarCoordinate { get; set; }
        public float[] LeftStick { get; set; }
        public float[] RightStick { get; set; }
    }

    private JoyData joyData;
    private Vector3[] tgPosition;
    private float[] tgAngles;

    // ����I��
    public bool linearStaircaseClimbing = false;
    public bool linearStaircaseDescending = false;
    public bool spiralStaircaseClimbing = false;
    // public bool stopEnvironment = false;
    public bool searchOptimalTrajectory = false;
    private string conf = "";

    public bool DEBUG = false;

    // �����ɃA�N�V�������[�h��ݒ�ł���悤�ɂ���
    public bool useVirtualController = false;   // true -> NavMesh�Ő������ꂽ�o�H�ɉ����悤��TG�̌����𒲐��Afalse -> ���ۂ�xbox�R���g���[������̓��͂��g�p
    public bool usePmtg = false; // ���s������PMTG�������̗p���邩�ǂ���  (true -> pmtg, false -> RL�̏o�͂����̂܂܋r�̖ړI�ʒu�ɂȂ�)
    public bool setTgWidthHeight = false;  // �O���������width��height��ݒ肷��
    public bool setTgPosition = false; // �����w�K��TG�̈ʒu�𒲐߂��邩�ǂ���  (true -> ���߂���, false -> ���߂��Ȃ����f�t�H���g�̂܂�)
    public bool setTgRotation = false; // �����w�K��TG�̌����𒲐߂��邩�ǂ���  (true -> ���߂���, false -> ���߂��Ȃ����f�t�H���gor�R���g���[���ɂ݈̂ˑ�)


    // ���O�̃A�N�V������ۑ�
    // private List<float> lastActions = new List<float>();
    public float[] lastActions;

    // Start is called before the first frame update
    private void Start()
    { 
        quadrupedTrajectoryGenerator = this.GetComponent<QuadrupedTrajectoryGenerator>();
        quadrupedIK = this.GetComponent<QuadrupedIK>();

/*        
        leftLidarSensor = leftLidar.GetComponent<Lidar>();
        rightLidarSensor = rightLidar.GetComponent<Lidar>();
*/
        quadrupedActuators = this.GetComponent<QuadrupedActuators>();
        quadrupedSensors = this.GetComponent<QuadrupedSensors>();
        xboxControllerInput = this.GetComponent<XboxControllerInput>();
        robotNavigation = this.GetComponent<RobotNavigation>();
        searchOptimalTrajectoryGenerator = this.GetComponent<SearchOptimalTrajectoryGenerator>();

        // reset
        articulationBodyReset = new ArticulationBodyReset();
        rootArticulation.GetComponent<ArticulationBody>().immovable = true;
        // 階段の下りにも対応できるように最初だけもう一度、以下を実行する
        articulationBodyReset.InitializeArticulationBodies(rootArticulation.GetComponent<ArticulationBody>());

        // linearStaircaseClimbingCurriculum = new LinearStaircaseClimbingCurriculum();
        if (useCurriculum)
        {
            curriculumManager = new LinearStaircaseClimbingCurriculum();
            curriculumManager.curriculumFile = curriculumFile;
            curriculumManager.InitCurriculum(startCurriculumNumber);
        }
        // linearStaircaseDescendingCurriculum = new LinearStaircaseDescendingCurriculum();

        buffRotation = QuadrupedRobotTransform.localRotation.eulerAngles;
        buffPosition = QuadrupedRobotTransform.localPosition;

        /*humanoidPosReset = Humanoid.transform.position;
        humanoidRotReset = Humanoid.transform.rotation;*/

        quadrupedReward = this.GetComponent<QuadrupedReward>();

        quadrupedTrajectoryGenerator.xboxControllerMode = false;
        
        if (wheelChairMode != WheelChairMode.None)
        {
            chairController = this.GetComponent<ChairController>();
        }

        /*switch (envType)
        {
            case EnvType.StairClimbingWithNoPassenger:
                break;
            case EnvType.StairClimbingWithPassenger:
                break;
            case EnvType.StairDescendingWithNoPassenger:
                break;
            case EnvType.StairDescendingWithPassenger:
                break;
        }*/

        // �ݒ�𔽉f
        /*if (linearStaircaseClimbing && !searchOptimalTrajectory)
        {
            conf = "LinearStaircaseClimbing";
            if (useCurriculum)
            {
                linearStaircaseClimbingCurriculum.curriculumFile = curriculumFile;
                linearStaircaseClimbingCurriculum.InitCurriculum(startCurriculumNumber);
            }
        }
        else if (spiralStaircaseClimbing)
        {
            conf = "SpiralStaircaseClimbing";
        }
        else if (stopEnvironment)
        {
            conf = "stopEnvironment";
        }
        else if (searchOptimalTrajectory && linearStaircaseClimbing)
        {
            conf = "searchOptimalTrajectoryWithLinearStaircase";
        }
        Debug.Log($"Running mode is {conf}");*/

        // �Â��v���O�������K�v�ɂȂ�ꍇ������̂ňȉ����g����悤�ɂ��Ă���
        poseRuleController = this.GetComponent<PoseRuleController>();

        // �A���s�������擾
        BehaviorParameters behaviorParameters = this.GetComponent<BehaviorParameters>();
        int continuousActionSize = behaviorParameters.BrainParameters.ActionSpec.NumContinuousActions;// BrainParameters.VectorActionDescriptions.LongLength;
        lastActions = new float[continuousActionSize];

        // �s����Ԃ̐ݒ�ɉ�����1�X�e�b�v�O�̍s�����L�^����o�b�t�@�̃T�C�Y�𒲐߂���
    }

    public override void OnEpisodeBegin()
    {
        episodeStartTime = Time.time; // 現在の時刻を記録
        currentSteps = 0;
        quadrupedIK.ikReset();
        tgPosition = new Vector3[4];

        nEpisode += 1;
        if (useCurriculum)
        {
            Debug.Log($"nEpisode: {nEpisode}, nGoal: {nGoal}, currentGoalsAchieved: {curriculumManager.currentGoalsAchieved}, GoalFlagHistory: {curriculumManager.GoalFlagHistoryStr}, P={nGoal / nEpisode}, {curriculumManager.textOfCurrentCurriculum()}");
        }
        else
        {
            Debug.Log($"nEpisode: {nEpisode}, nGoal: {nGoal}, P={nGoal / nEpisode}");
        }

        switch (envType)
        {
            case EnvType.StopEnvironment:
                Target.transform.position = new Vector3(-2.5f, 1.5f, 5f);
                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-2.5f, 0.3f, -1.5f));
                break;
            case EnvType.LinearStairClimbingWithNoPassenger:
                // Target.transform.position = new Vector3(Random.Range(-4, -1), 1.5f, Random.Range(4f, 5f));
                Target.transform.position = new Vector3(-2.5f, 1.5f, 7f);

                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-2.5f, 0.3f, -4f));  // ���X�Ay��0.5f������

                currentFrontPitch = 0;
                currentRearPitch = 0;
                currentFrontX = 0f;
                currentFrontZ = 0.15f;
                currentRearX = 0f;
                currentRearZ = 0.15f;
                SetTgParams(currentFrontPitch, currentRearPitch, currentFrontX, currentFrontZ, currentRearX, currentRearZ);
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();

                if (useCurriculum)
                {
                    Dictionary<string, object> curriculumData = curriculumManager.MonitorCurriculum(goalFlag, totalStepCount);
                    goalFlag = false;
                    riserRatio = (float)curriculumData["RiserHeight"];
                    weightRatio = (float)curriculumData["WeightRatio"];
                    goalDistanceFactor = (float)curriculumData["GoalDistanceFactor"];

                    // �K�i�̃X�P�[���ύX
                    Vector3 newScale = linearStaircase.transform.localScale;
                    newScale.y = riserRatio;
                    linearStaircase.transform.localScale = newScale;
                    // �S�[���ʒu�̕ύX
                    Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y, rootArticulation.transform.position.z + (Target.transform.position.z - rootArticulation.transform.position.z) * goalDistanceFactor);
                }

                // 椅子の初期状態
                chairController.setChairArmJoint(-1f);
                chairController.setSlidarJoint(1f);
                chairController.setBackSeatJoint(0f);

                break;
            case EnvType.LinearStairClimbingWithPassenger:
                Target.transform.position = new Vector3(-2.5f, 0, 5f);

                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-2.5f, 0.3f, -2f));  // ���X�Ay��0.5f������

                currentFrontPitch = 0;
                currentRearPitch = 0;
                currentFrontX = 0f;
                currentFrontZ = 0.15f;
                currentRearX = 0f;
                currentRearZ = 0.15f;
                SetTgParams(currentFrontPitch, currentRearPitch, currentFrontX, currentFrontZ, currentRearX, currentRearZ);
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();

                if (useCurriculum)
                {
                    Dictionary<string, object> curriculumData = curriculumManager.MonitorCurriculum(goalFlag, totalStepCount);

                    goalFlag = false;
                    riserRatio = curriculumData.ContainsKey("RiserHeight") ? Convert.ToSingle(curriculumData["RiserHeight"]) : 0.0f;
                    weightRatio = curriculumData.ContainsKey("WeightRatio") ? Convert.ToSingle(curriculumData["WeightRatio"]) : 0.0f;
                    goalDistanceFactor = curriculumData.ContainsKey("GoalDistanceFactor") ? Convert.ToSingle(curriculumData["GoalDistanceFactor"]) : 0.0f;

                    Debug.Log($"riserRatio: {riserRatio}, weightRatio: {weightRatio}, goalDistanceFactor: {goalDistanceFactor}");

                    // �K�i�̃X�P�[���ύX
                    Vector3 newScale = linearStaircase.transform.localScale;
                    newScale.y = riserRatio;
                    linearStaircase.transform.localScale = newScale;
                    Vector3 newGoalScale = Target.transform.localScale;
                    newGoalScale.y = riserRatio;
                    Target.transform.localScale = newGoalScale;

                    // �S�[���ʒu�̕ύX
                    Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y, rootArticulation.transform.position.z + (Target.transform.position.z - rootArticulation.transform.position.z) * goalDistanceFactor);
                    Humanoid.GetComponent<ArticulationBody>().mass = humanoidMaxWeight * weightRatio;
                }

                chairController.resetChair();


                break;
            case EnvType.LinearStairDescendingWithNoPassenger:
                Target.transform.position = new Vector3(-2.25f, 0.5f, -4f);
                // スクリーンショットに基づいた新しい回転値を設定するためのQuaternionを生成
                Quaternion newRotation = Quaternion.Euler(0, 180f, 0);
                articulationBodyReset.UpdateInitialRotation("SittingChairAngle_1", newRotation);
                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-2.5f, 1.5f + linearStaircase.transform.localScale.y, 7f));  // ���X�Ay��0.5f������
                // Humanoid.transform.position = new Vector3(-2.5f, 1.5f + linearStaircase.transform.localScale.y, 7f);

                currentFrontPitch = 0;
                currentRearPitch = 0;
                currentFrontX = 0f;
                currentFrontZ = 0.1f;
                currentRearX = 0f;
                currentRearZ = 0.1f;
                SetTgParams(currentFrontPitch, currentRearPitch, currentFrontX, currentFrontZ, currentRearX, currentRearZ);
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();

                if (useCurriculum)
                {
                    Dictionary<string, object> curriculumData = curriculumManager.MonitorCurriculum(goalFlag, totalStepCount);
                    goalFlag = false;
                    riserRatio = (float)curriculumData["RiserHeight"];
                    weightRatio = (float)curriculumData["WeightRatio"];
                    goalDistanceFactor = (float)curriculumData["GoalDistanceFactor"];

                    // �K�i�̃X�P�[���ύX
                    Vector3 newScale = linearStaircase.transform.localScale;
                    newScale.y = riserRatio;
                    linearStaircase.transform.localScale = newScale;
                    // �S�[���ʒu�̕ύX
                    Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y, rootArticulation.transform.position.z + (Target.transform.position.z - rootArticulation.transform.position.z) * goalDistanceFactor);
                }
                break;
            case EnvType.LinearStairDescendingWithPassenger:
                Target.transform.localScale = new Vector3(1f, 0.02f, 1f);
                Target.transform.position = new Vector3(-2.4f, 0f, -2.5f);

                // 椅子を下りの姿勢に設定
                // スクリーンショットに基づいた新しい回転値を設定するためのQuaternionを生成
                articulationBodyReset.UpdateInitialRotation("SittingChairAngle_1", Quaternion.Euler(0, 180f, 0));
                articulationBodyReset.UpdateInitialRotation("FootLegSupport_1", Quaternion.Euler(-40f, 0, 0));
                articulationBodyReset.UpdateInitialRotation("BackSheet_1", Quaternion.Euler(-45f, 0, 0));
                articulationBodyReset.UpdateInitialRotation("LeftArmSupport_1", Quaternion.Euler(30f, 0, 0));
                articulationBodyReset.UpdateInitialRotation("RightArmSupport_1", Quaternion.Euler(30f, 0, 0));

                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-2.5f, 1.5f * linearStaircase.transform.localScale.y + 0.5f, 4.5f));  // ���X�Ay��0.5f������
                // Humanoid.transform.localPosition = new Vector3(0.198f, -0.929, 0.143f);
                /*Humanoid.transform.rotation = new Quaternion(0, 0, 0, 0);*/
                leftUpLeg.transform.localRotation = Quaternion.Euler(-60f, 6.23f, 176.56f);
                leftLeg.transform.localRotation = Quaternion.Euler(-100f, -11.394f, 13.55f);
                rightUpLeg.transform.localRotation = Quaternion.Euler(-60f, 0.7f, -181.66f);
                rightLeg.transform.localRotation = Quaternion.Euler(-100f, 10.046f, -11.91f);

                currentFrontPitch = 0;
                currentRearPitch = 0;
                currentFrontX = 0f;
                currentFrontZ = 0.15f;
                currentRearX = 0f;
                currentRearZ = 0.15f;
                SetTgParams(currentFrontPitch, currentRearPitch, currentFrontX, currentFrontZ, currentRearX, currentRearZ);
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();

                if (useCurriculum)
                {
                    Dictionary<string, object> curriculumData = curriculumManager.MonitorCurriculum(goalFlag, totalStepCount);
                    goalFlag = false;
                    riserRatio = curriculumData.ContainsKey("RiserHeight") ? Convert.ToSingle(curriculumData["RiserHeight"]) : 0.0f;
                    weightRatio = curriculumData.ContainsKey("WeightRatio") ? Convert.ToSingle(curriculumData["WeightRatio"]) : 0.0f;
                    goalDistanceFactor = curriculumData.ContainsKey("GoalDistanceFactor") ? Convert.ToSingle(curriculumData["GoalDistanceFactor"]) : 0.0f;

                    Vector3 newScale = linearStaircase.transform.localScale;
                    newScale.y = riserRatio;
                    linearStaircase.transform.localScale = newScale;

                    Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y, rootArticulation.transform.position.z + (Target.transform.position.z - rootArticulation.transform.position.z) * goalDistanceFactor);
                    Humanoid.GetComponent<ArticulationBody>().mass = humanoidMaxWeight * weightRatio;
                }

                break;
            case EnvType.SpiralStairClimbingWithPassenger:

                articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(-0.5f, 0.3f, -2f));

                Humanoid.transform.position = new Vector3(-2.725f, -0.25f, -1.5f);
                Humanoid.transform.rotation = new Quaternion(0, 0, 0, 0);

                currentFrontPitch = 0;
                currentRearPitch = 0;
                currentFrontX = 0f;
                currentFrontZ = 0.15f;
                currentRearX = 0f;
                currentRearZ = 0.15f;
                SetTgParams(currentFrontPitch, currentRearPitch, currentFrontX, currentFrontZ, currentRearX, currentRearZ);
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();

                if (useCurriculum)
                {
                    Dictionary<string, object> curriculumData = curriculumManager.MonitorCurriculum(goalFlag, totalStepCount);
                    goalFlag = false;
                    riserRatio = curriculumData.ContainsKey("RiserHeight") ? Convert.ToSingle(curriculumData["RiserHeight"]) : 0.0f;
                    weightRatio = curriculumData.ContainsKey("WeightRatio") ? Convert.ToSingle(curriculumData["WeightRatio"]) : 0.0f;
                    float goalAngle = curriculumData.ContainsKey("Angle") ? Convert.ToSingle(curriculumData["Angle"]) : 0.0f;

                    Target.transform.rotation = Quaternion.Euler(0, goalAngle, 0);
                    Vector3 newPosition = centerPosition + new Vector3(-rotationR*Mathf.Cos(goalAngle * Mathf.Deg2Rad), 2.5f* goalAngle/270f, rotationR * Mathf.Sin(goalAngle * Mathf.Deg2Rad));
                    Target.transform.position = newPosition;

                    // �K�i�̃X�P�[���ύX
                    Vector3 newScale = spiralStaircase.transform.localScale;
                    newScale.y = riserRatio;
                    spiralStaircase.transform.localScale = newScale;

                    Humanoid.GetComponent<ArticulationBody>().mass = humanoidMaxWeight * weightRatio;
                }

                // 椅子の初期状態
                chairController.setChairArmJoint(-1f);
                chairController.setSlidarJoint(1f);
                chairController.setBackSeatJoint(0f);
                chairController.resetChair();

                break;
        }
    }

    public void SetTgParams(int currentFrontPitch, int currentRearPitch, float currentFrontX, float currentFrontZ, float currentRearX, float currentRearZ)
    {
        // �O�r��TG�̉�]
        quadrupedTrajectoryGenerator.frontLeft.offsetRotation = new Vector3Msg(0, currentFrontPitch, 0).From<FLU>();
        quadrupedTrajectoryGenerator.frontRight.offsetRotation = new Vector3Msg(0, currentFrontPitch, 0).From<FLU>();
        // ��r��TG�̉�]
        quadrupedTrajectoryGenerator.rearLeft.offsetRotation = new Vector3Msg(0, currentRearPitch, 0).From<FLU>();
        quadrupedTrajectoryGenerator.rearRight.offsetRotation = new Vector3Msg(0, currentRearPitch, 0).From<FLU>();
        // �O�r��TG�̈ʒu
        quadrupedTrajectoryGenerator.frontLeft.offsetPosition = new Vector3Msg(currentFrontX, 0, currentFrontZ).From<FLU>();
        quadrupedTrajectoryGenerator.frontRight.offsetPosition = new Vector3Msg(currentFrontX, 0, currentFrontZ).From<FLU>();
        // ��r��TG�̈ʒu
        quadrupedTrajectoryGenerator.rearLeft.offsetPosition = new Vector3Msg(currentRearX, 0, currentRearZ).From<FLU>();
        quadrupedTrajectoryGenerator.rearRight.offsetPosition = new Vector3Msg(currentRearX, 0, currentRearZ).From<FLU>();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        totalStepCount += 1;   // �X�e�b�v���E���g��1���₷
        currentSteps++; // �X�e�b�v�����C���N�������g


        int numValues = 0; // �ϑ��̏�񐔂��J�E���g

        if (useVirtualController)
        {
            switch (joyMode)
            {
                case JoyMode.Autonomous:
                    joyData = getJoyData(2, DEBUG);  // NavMesh���烍�{�b�g�̐i�s�����𐧌�
                    break;
                case JoyMode.Straight:
                    joyData = getJoyData(3, DEBUG);
                    break;
            }
        }
        else
        {
            joyData = getJoyData(0, DEBUG);  // ���ۂ�xbox���烍�{�b�g�̐i�s�����𐧌�
        }

        if (useLiDAR)
        {
            quadrupedSensors.lidar1.Scan();
            quadrupedSensors.lidar2.Scan();
            
            for (int incr = 0; incr < quadrupedSensors.lidar1.numberOfIncrements; incr++)
            {
                for (int layer = 0; layer < quadrupedSensors.lidar1.numberOfLayers; layer++)
                {
                    int indx = layer + incr * quadrupedSensors.lidar1.numberOfLayers;

                 /*   if (indx % 2 == 0 && layer == 0)
                    {*/
                        if (incr > quadrupedSensors.lidar1.numberOfIncrements * 9 / 16 && incr < quadrupedSensors.lidar1.numberOfIncrements * 15 / 16)
                        {
                            if (quadrupedSensors.lidar1.distances[indx] < 3.5)   // 1.5m���Z�����
                            {
                                AddObservationWithNanCheck(sensor, quadrupedSensors.lidar1.distances[indx]);
                                numValues += 1;
                                // sensor.AddObservation(quadrupedSensors.lidar1.distances[indx]);
                            }
                            else    // �������
                            {
                                sensor.AddObservation(3.5f);
                                numValues += 1;
                            }

                            if (quadrupedSensors.lidar2.distances[indx] < 3.5)   // 1.5m���Z�����
                            {
                                AddObservationWithNanCheck(sensor, quadrupedSensors.lidar2.distances[indx]);
                                numValues += 1;
                                // sensor.AddObservation(quadrupedSensors.lidar2.distances[indx]);
                            }
                            else    // �������
                            {
                                sensor.AddObservation(3.5f);
                                numValues += 1;
                            }

                            // �����ɕ`�悵�Ă݂悤
                            /*   i += 2;
                               Debug.Log(rightLidarSensor.distances[indx]);
                               Debug.Log(leftLidarSensor.distances[indx]);*/
                        }
                    /*}*/
                }
            }
        }

        AddObservationWithNanCheck(sensor, joyData.LeftStick[0]); 
        AddObservationWithNanCheck(sensor, joyData.LeftStick[1]); 
        AddObservationWithNanCheck(sensor, joyData.RightStick[0]); 
        AddObservationWithNanCheck(sensor, joyData.RightStick[1]);
        numValues += 4;

        imuData = quadrupedSensors.GetImuData(); 
        AddObservationWithNanCheck(sensor, (float)imuData.linear_acceleration.x);
        AddObservationWithNanCheck(sensor, (float)imuData.linear_acceleration.y);
        AddObservationWithNanCheck(sensor, (float)imuData.linear_acceleration.z);
        numValues += 3;

        AddObservationWithNanCheck(sensor, (float)imuData.angular_velocity.x);
        AddObservationWithNanCheck(sensor, (float)imuData.angular_velocity.y);
        AddObservationWithNanCheck(sensor, (float)imuData.angular_velocity.z);
        numValues += 3;

        // 速度と各加速度の取得
        var currentVelocity = quadrupedSensors.GetVelocity();
        var currentAngularAcceleration = quadrupedSensors.GetAngularAcceleration();
        AddObservationWithNanCheck(sensor, (float)currentVelocity.x);
        AddObservationWithNanCheck(sensor, (float)currentVelocity.y);
        AddObservationWithNanCheck(sensor, (float)currentVelocity.z);
        AddObservationWithNanCheck(sensor, (float)currentAngularAcceleration.x);
        AddObservationWithNanCheck(sensor, (float)currentAngularAcceleration.y);
        AddObservationWithNanCheck(sensor, (float)currentAngularAcceleration.z);
        numValues += 6;


        var rollPitchYaw = quadrupedSensors.GetRollPitchYaw();
        // Debug.Log($"Roll: {rollPitchYaw.x}, Pitch: {rollPitchYaw.y}, Yaw: {rollPitchYaw.z}");
        AddObservationWithNanCheck(sensor, (float)rollPitchYaw.x);
        AddObservationWithNanCheck(sensor, (float)rollPitchYaw.y);
        AddObservationWithNanCheck(sensor, (float)rollPitchYaw.z);
        numValues += 3;

        // Front Left Force
        frontLeftForce = quadrupedSensors.GetFrontLeftForce();
        AddObservationWithNanCheck(sensor, (float)frontLeftForce.x, 300f);
        AddObservationWithNanCheck(sensor, (float)frontLeftForce.y, 300f);
        AddObservationWithNanCheck(sensor, (float)frontLeftForce.z, 300f);
        numValues += 3;

        // Front Right Force
        frontRightForce = quadrupedSensors.GetFrontRightForce();
        AddObservationWithNanCheck(sensor, (float)frontRightForce.x, 300f);
        AddObservationWithNanCheck(sensor, (float)frontRightForce.y, 300f);
        AddObservationWithNanCheck(sensor, (float)frontRightForce.z, 300f);
        numValues += 3;

        // Rear Left Force
        rearLeftForce = quadrupedSensors.GetRearLeftForce();
        AddObservationWithNanCheck(sensor, (float)rearLeftForce.x, 300f);
        AddObservationWithNanCheck(sensor, (float)rearLeftForce.y, 300f);
        AddObservationWithNanCheck(sensor, (float)rearLeftForce.z, 300f);
        numValues += 3;

        // Rear Right Force
        rearRightForce = quadrupedSensors.GetRearRightForce();
        AddObservationWithNanCheck(sensor, (float)rearRightForce.x, 300f);
        AddObservationWithNanCheck(sensor, (float)rearRightForce.y, 300f);
        AddObservationWithNanCheck(sensor, (float)rearRightForce.z, 300f);
        numValues += 3;

        // TG phase (0, 1, 2)
        AddOneHotEncoding(sensor, quadrupedTrajectoryGenerator.frontLeftTg.tgPhase, 3);
        AddOneHotEncoding(sensor, quadrupedTrajectoryGenerator.frontRightTg.tgPhase, 3);
        AddOneHotEncoding(sensor, quadrupedTrajectoryGenerator.rearLeftTg.tgPhase, 3);
        AddOneHotEncoding(sensor, quadrupedTrajectoryGenerator.rearRightTg.tgPhase, 3);
        numValues += 4 * 3;  // 3 possible phases for 4 legs
        // Debug.Log($"{quadrupedTrajectoryGenerator.frontLeftTg.tgPhase}, {quadrupedTrajectoryGenerator.frontRightTg.tgPhase}, {quadrupedTrajectoryGenerator.rearLeftTg.tgPhase}, {quadrupedTrajectoryGenerator.rearRightTg.tgPhase}");

        // TG frequency
        sensor.AddObservation((float)quadrupedTrajectoryGenerator.frontLeftTg.frequency);
        sensor.AddObservation((float)quadrupedTrajectoryGenerator.frontRightTg.frequency);
        sensor.AddObservation((float)quadrupedTrajectoryGenerator.rearLeftTg.frequency);
        sensor.AddObservation((float)quadrupedTrajectoryGenerator.rearRightTg.frequency);
        numValues += 4;
        // Debug.Log($"{quadrupedTrajectoryGenerator.frontLeftTg.frequency}, {quadrupedTrajectoryGenerator.frontRightTg.frequency}, {quadrupedTrajectoryGenerator.rearLeftTg.frequency}, {quadrupedTrajectoryGenerator.rearRightTg.frequency}");

        // ���O�̕���̍s���o�͂��L�^
        // �ȑO�̃A�N�V�������ϑ��Ƃ��Ēǉ�
        foreach (var action in lastActions)
        {
            sensor.AddObservation(action);
            numValues += 1;
        }

        // ����ɂ�钲�߂��{���ꂽ��̃A�N�V����
        foreach (var tgPos in tgPosition)
        {
            AddObservationWithNanCheck(sensor, tgPos);
            numValues += 3;
            // Debug.Log($"tgPos.x:{tgPos.x}, tgPos.y:{tgPos.y}, tgPos.z:{tgPos.z}");
        }

        // Foot position residual

        jointRotation = quadrupedActuators.GetCurrentNormalizedJointRotationsAndDirections();
        for (int ind = 0; ind < 12; ind++)
        {
            AddObservationWithNanCheck(sensor, jointRotation[ind]);
            numValues += 1;
            // sensor.AddObservation(jointRotation[ind]);
        }

        if (DEBUG)
        {
            Debug.Log($"[DEBUG INFO] Observation Size: {numValues}");
        }
    }

    private void AddOneHotEncoding(VectorSensor sensor, int value, int range)
    {
        for (int i = 0; i < range; i++)
        {
            if (i == value)
            {
                sensor.AddObservation(1);
            }
            else
            {
                sensor.AddObservation(0);
            }
        }
    }

    // NaN���`�F�b�N����w���p�[�֐�
    void AddObservationWithNanCheck(VectorSensor sensor, float value, float maxVal = 1f)
    {
        if (float.IsNaN(value))
        {
            Debug.LogError("NaN value detected!");
        }
        sensor.AddObservation(value / maxVal);
    }

    // Vector3�p�̃o�[�W���������l�ɍ쐬�\
    void AddObservationWithNanCheck(VectorSensor sensor, Vector3 value, float maxVal = 1f)
    {
        if (float.IsNaN(value.x) || float.IsNaN(value.y) || float.IsNaN(value.z))
        {
            Debug.LogError("NaN value detected in Vector3!");
        }
        sensor.AddObservation(value / maxVal);
    }

    float MapAngleToLimitedRange(float angle)
    {
        // 0 - 360の範囲を正規化
        angle = angle % 360;
        // 角度が0 - 45の範囲にあるか、または315 - 360（実際には0）の範囲にある場合はそのまま使用
        if (angle <= 45)
        {
            return angle;
        }
        // 角度が315 - 360の範囲にある場合は、角度から360を引いて負の値に変換
        else if (angle >= 315)
        {
            return angle - 360;
        }
        // 角度が45 - 180の範囲にある場合は、45に丸める
        else if (angle <= 180)
        {
            return 45;
        }
        // 角度が180 - 315の範囲にある場合は、-45に丸める
        else
        {
            return -45;
        }
    }

    float MapAngleToMinusOneToOne(float angle)
    {
        // 角度を-45から45の範囲にマッピング
        angle = MapAngleToLimitedRange(angle);
        // 結果を-1から1にスケーリング
        return angle / 45f;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // �ȉ���if���̓G�s�\�[�h�J�n���̔�ђ��˖h�~
        PreventBouncingAtEpisodeStart(currentSteps, firstStopStepCounts);

        // �V�����A�N�V���������X�g�ɒǉ�
        for (int i = 0; i < actions.ContinuousActions.Length; i++)
        {
            lastActions[i] = actions.ContinuousActions[i];
        }


        int indx = 0;
        if (usePmtg)
        {
            // PMTG���g�p���郂�[�h
            // Width��Height��ݒ肷��ꍇ
            if (setTgWidthHeight)
            {
                // quadrupedTrajectoryGenerator.frontLeft
                // TgWidth: 0~0.5
                // TgHeight: 0~0.4
                quadrupedTrajectoryGenerator.SetTgParams(
                    ref quadrupedTrajectoryGenerator.frontLeft, 
                    (actions.ContinuousActions[indx++] + 1) * 0.5 / 2,
                    (actions.ContinuousActions[indx++] + 1) * 0.2
                );
                quadrupedTrajectoryGenerator.SetTgParams(
                    ref quadrupedTrajectoryGenerator.frontRight,
                    (actions.ContinuousActions[indx++] + 1) * 0.5 / 2,
                    (actions.ContinuousActions[indx++] + 1) * 0.2
                );
                quadrupedTrajectoryGenerator.SetTgParams(
                    ref quadrupedTrajectoryGenerator.rearLeft,
                    (actions.ContinuousActions[indx++] + 1) * 0.5 / 2,
                    (actions.ContinuousActions[indx++] + 1) * 0.2
                );
                quadrupedTrajectoryGenerator.SetTgParams(
                    ref quadrupedTrajectoryGenerator.rearRight,
                    (actions.ContinuousActions[indx++] + 1) * 0.5 / 2,
                    (actions.ContinuousActions[indx++] + 1) * 0.2
                );
            }

            // TG�̈ʒu��ݒ肷��ꍇ
            if (setTgPosition)
            {
                // 胴体の傾きに応じて柔軟に変更する機能を付与する
                // 前後の傾きを取得
                float originalValueX = rootArticulation.transform.localRotation.eulerAngles.x;
                float valueX = MapAngleToMinusOneToOne(MapAngleToLimitedRange(originalValueX));
                // 左右の傾きを取得
                float originalValueZ = rootArticulation.transform.localRotation.eulerAngles.z;
                float valueZ = MapAngleToMinusOneToOne(MapAngleToLimitedRange(originalValueZ));

                // 階段の傾きに応じて調節する方法

                // Debug.Log($"originalValueX: {valueX}, originalValueZ: {valueZ}");

                quadrupedTrajectoryGenerator.frontLeft.offsetPosition = new Vector3(-valueZ * 0.05f, currentFrontZ - valueX * 0.05f - valueZ * 0.05f, valueX * 0.1f);
                quadrupedTrajectoryGenerator.frontRight.offsetPosition = new Vector3(-valueZ * 0.05f, currentFrontZ - valueX * 0.05f + valueZ * 0.05f, valueX * 0.1f);
                quadrupedTrajectoryGenerator.rearLeft.offsetPosition = new Vector3(-valueZ * 0.05f, currentRearZ + valueX * 0.0f - valueZ * 0.05f, valueX * 0.1f);
                quadrupedTrajectoryGenerator.rearRight.offsetPosition = new Vector3(-valueZ * 0.05f, currentRearZ + valueX * 0.0f + valueZ * 0.05f, valueX * 0.1f);

                quadrupedTrajectoryGenerator.setTrajectoryBasePosition();
            }

            if (setTgRotation) 
            {
                float originalValueX = rootArticulation.transform.localRotation.eulerAngles.x;
                // �O�r��TG�̉�]
                quadrupedTrajectoryGenerator.frontLeft.offsetRotation = new Vector3Msg(0, originalValueX, 0).From<FLU>();
                quadrupedTrajectoryGenerator.frontRight.offsetRotation = new Vector3Msg(0, originalValueX, 0).From<FLU>();
                // ��r��TG�̉�]
                quadrupedTrajectoryGenerator.rearLeft.offsetRotation = new Vector3Msg(0, originalValueX, 0).From<FLU>();
                quadrupedTrajectoryGenerator.rearRight.offsetRotation = new Vector3Msg(0, originalValueX, 0).From<FLU>();
                // ��]�̕ύX�̓K�p
                quadrupedTrajectoryGenerator.setTrajectoryBaseRotation();
            }

            // tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectory(Time.time - episodeStartTime, joyData.LeftStickPolarCoordinate[0], joyData.LeftStickPolarCoordinate[1], joyData.RightStick[0], false);
            switch (envType)
            {
                case EnvType.LinearStairClimbingWithNoPassenger:
                    tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectoryForward(Time.time - episodeStartTime, joyData.LeftStick[0] * 0.5f, joyData.LeftStick[1], tgWidthScale, false);
                    break;
                case EnvType.LinearStairClimbingWithPassenger:
                    tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectoryForward(Time.time - episodeStartTime, joyData.LeftStick[0] * 0.5f, joyData.LeftStick[1], tgWidthScale, false);
                    break;
                case EnvType.LinearStairDescendingWithNoPassenger:
                    tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectoryBack(Time.time - episodeStartTime, joyData.LeftStick[0] * 0.5f, joyData.LeftStick[1], false);
                    break;
                case EnvType.LinearStairDescendingWithPassenger:
                    tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectoryBack(Time.time - episodeStartTime, joyData.LeftStick[0] * 0.5f, joyData.LeftStick[1], false);
                    break;
                case EnvType.SpiralStairClimbingWithPassenger:
                    tgPosition = quadrupedTrajectoryGenerator.getTrotTrajectoryForward(Time.time - episodeStartTime, joyData.LeftStick[0] * 0.5f, joyData.LeftStick[1], tgWidthScale, false);
                    break;
            }

            // TG���狁�߂�ꂽ�e����̍��W�����������
            tgPosition[0].x = tgPosition[0].x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[0].y = tgPosition[0].y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[0].z = tgPosition[0].z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].x = tgPosition[1].x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].y = tgPosition[1].y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].z = tgPosition[1].z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].x = tgPosition[2].x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].y = tgPosition[2].y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].z = tgPosition[2].z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].x = tgPosition[3].x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].y = tgPosition[3].y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].z = tgPosition[3].z + actions.ContinuousActions[indx++] * 0.1f;
        }
        else
        {
            // PMTG���g�p���Ȃ����[�h
            tgPosition[0].x = quadrupedTrajectoryGenerator.frontLeft.trajectoryBase.transform.localPosition.x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[0].y = quadrupedTrajectoryGenerator.frontLeft.trajectoryBase.transform.localPosition.y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[0].z = quadrupedTrajectoryGenerator.frontLeft.trajectoryBase.transform.localPosition.z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].x = quadrupedTrajectoryGenerator.frontRight.trajectoryBase.transform.localPosition.x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].y = quadrupedTrajectoryGenerator.frontRight.trajectoryBase.transform.localPosition.y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[1].z = quadrupedTrajectoryGenerator.frontRight.trajectoryBase.transform.localPosition.z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].x = quadrupedTrajectoryGenerator.rearLeft.trajectoryBase.transform.localPosition.x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].y = quadrupedTrajectoryGenerator.rearLeft.trajectoryBase.transform.localPosition.y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[2].z = quadrupedTrajectoryGenerator.rearLeft.trajectoryBase.transform.localPosition.z + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].x = quadrupedTrajectoryGenerator.rearRight.trajectoryBase.transform.localPosition.x + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].y = quadrupedTrajectoryGenerator.rearRight.trajectoryBase.transform.localPosition.y + actions.ContinuousActions[indx++] * 0.1f;
            tgPosition[3].z = quadrupedTrajectoryGenerator.rearRight.trajectoryBase.transform.localPosition.z + actions.ContinuousActions[indx++] * 0.1f;
        }

        switch (wheelChairMode)
        {
            case WheelChairMode.None:
                break;
            case WheelChairMode.Default:
                // デフォルトモードのアクション
                chairController.setChairArmJoint(-1f);
                chairController.setSlidarJoint(1f);
                chairController.setBackSeatJoint(0f);
                break;
            case WheelChairMode.UpStair:
                float originalValue = rootArticulation.transform.localRotation.eulerAngles.x;
                float newValue = 0;
                if (originalValue >= 315 && originalValue <= 360)
                {
                    newValue = -2 * (originalValue - 315) / (360 - 315) + 1;
                }
                else if (originalValue >= 270 && originalValue < 315)
                {
                    newValue = 1f;
                }
                else if (originalValue > 0 && originalValue < 90)
                {
                    newValue = -1f;
                }
                chairController.resetChair(); // 椅子の他の部位が変化しないように、先に全体のリセットをかけておく
                // Debug.Log($"rootArticulation: {rootArticulation.transform.localRotation.eulerAngles.x}");
                // Debug.Log($"rootArticulation: {newValue}");
                chairController.setChairArmJoint(newValue);
                // chairController.setSlidarJoint(-newValue*0.5f + 0.5f); // 値の範囲を0~1に
                chairController.setBackSeatJoint(0f);
                break;
            case WheelChairMode.DownStair:
                chairController.setDownMode(); // 椅子の他の部位が変化しないように、先に全体のリセットをかけておく
                if (currentSteps > firstStopStepCounts)
                {
                    originalValue = rootArticulation.transform.localRotation.eulerAngles.x;
                    newValue = 0;
                    if (originalValue >= 315 && originalValue <= 360)
                    {
                        newValue = -2 * (originalValue - 315) / (360 - 315) + 1;
                    }
                    else if (originalValue >= 270 && originalValue < 315)
                    {
                        newValue = 1f;
                    }
                    else if (originalValue > 0 && originalValue < 90)
                    {
                        newValue = -1f;
                    }

                    // Debug.Log($"rootArticulation: {rootArticulation.transform.localRotation.eulerAngles.x}");
                    // Debug.Log($"rootArticulation: {newValue}");
                    chairController.setChairArmJoint(newValue);
                    chairController.setSlidarJoint(-newValue);
                }
                break;
        }


        tgAngles = quadrupedIK.GetJointAngles(tgPosition);
        if (tgMode)
        {
            quadrupedActuators.SetJointTargetAngles(tgAngles);
            quadrupedReward.CalculateAll(joyData.JoyMsg);
            // SetReward(quadrupedReward.approachRewardParams.reward + quadrupedReward.targetTouchReardParams.reward);
            // SetReward(quadrupedReward.targetTouchReardParams.reward);
            // SetReward(quadrupedReward.linearVelocityRewardParams.reward);
            // SetReward(quadrupedReward.angularVelocityRewardParams.reward);
            // SetReward(quadrupedReward.baseMotionRewardParams.reward);
            // SetReward(quadrupedReward.targetTouchReardParams.reward + quadrupedReward.linearVelocityRewardParams.reward + quadrupedReward.angularVelocityRewardParams.reward + quadrupedReward.baseMotionRewardParams.reward);
            /*SetReward(
                quadrupedReward.targetTouchReardParams.reward + 
                quadrupedReward.linearVelocityRewardParams.reward +
                quadrupedReward.angularVelocityRewardParams.reward + 
                quadrupedReward.baseMotionRewardParams.reward + 
                quadrupedReward.fallDownRewardParams.reward
            );*/
            SetReward(
                quadrupedReward.boundingBoxTargetTouchRewardParams.reward +
                quadrupedReward.linearVelocityRewardParams.reward +
                quadrupedReward.angularVelocityRewardParams.reward +
                quadrupedReward.baseMotionRewardParams.reward +
                quadrupedReward.fallDownRewardParams.reward
            );
            if (quadrupedReward.touchTheGoal)
            {
                goalFlag = true;
                nGoal += 1;
                EndEpisode();
            }
            else if (quadrupedReward.fallDown)
            {
                EndEpisode();
            }
        }
        else if (chVehiclePose)
        {
            // �ԗ����[�h�֕ϐg
            poseRuleController.VehicleMode();

            // ���i�Ɖ�]��
            poseRuleController.AwcMode(go, turn);
            SetReward(0f);
        }
        else if (chTgPose)
        {
            poseRuleController.TgMode();
            SetReward(0f);
        }
        else if (foldTheChair)
        {
            poseRuleController.VehicleMode();
            chairController.setFoldingTheChair();
        }
    }

    public void PreventBouncingAtEpisodeStart(int currentSteps, int stopSteps)
    {
        if (currentSteps <= stopSteps)
        {
            rootArticulation.GetComponent<ArticulationBody>().immovable = true;
        }
        else
        {
            rootArticulation.GetComponent<ArticulationBody>().immovable = false;
        }
    }

    JoyData getJoyData(int mode = 1, bool DEBUG = false)  // mode=0�̂Ƃ�xbox�̒l���擾�A1�̂Ƃ��^���I�ȃR���g���[���̏o�͂𐶐�
    {
        JoyMsg joyMsg = new JoyMsg();
        joyMsg.buttons = new int[11];
        joyMsg.axes = new float[8];

        float[] leftStickPolarCoordinate = new float[2];
        float[] leftStick = new float[2];
        float[] rightStick = new float[2];

        if (mode == 0)
        {
            joyMsg = xboxControllerInput.GetJoyMsg();
            // xbox�̏o�͂����̂܂܎g�p

            leftStickPolarCoordinate = xboxControllerInput.GetLeftStickPolarCoordinates();
            leftStick = xboxControllerInput.GetLeftStick();
            rightStick = xboxControllerInput.GetRightStick();
        }
        else if (mode == 1)
        {
            // �S�[���ʒu����t�Z�����l���R���g���[���̒l�Ƃ���ꍇ
            // ���{�b�g�̊�̌����ƃS�[���̌����̊p�x�����߂�

            // ���z�I�ȃW���C�X�e�B�b�N�̑����M�����쐬
            robotFaceVector = quadrupedTrajectoryGenerator.frontLeft.trajectoryBase.transform.position - quadrupedTrajectoryGenerator.rearLeft.trajectoryBase.transform.position;
            leftStickPolarCoordinate[0] = Mathf.Atan2(Target.transform.position.x - QuadrupedRobotTransform.position.x, Target.transform.position.z - QuadrupedRobotTransform.position.z) - Mathf.Atan2(robotFaceVector.x, robotFaceVector.z);
            leftStickPolarCoordinate[1] = 0.75f;
            // �f�J���g���W�\����
            leftStick[1] = leftStickPolarCoordinate[1] * Mathf.Cos(leftStickPolarCoordinate[0]);  // Unity��ROS�̍��W�n�͈قȂ�̂�ROS�ɂ�����y��cos�ŋ��߂Ă���_�ɒ���
            leftStick[0] = leftStickPolarCoordinate[1] * Mathf.Sin(leftStickPolarCoordinate[0]);  //
            // �E�ɂ��Ă�
            if (leftStickPolarCoordinate[0] >= -0.5f || leftStickPolarCoordinate[0] <= 0.5f)
            {
                rightStick[0] = leftStickPolarCoordinate[0];
            }
            else if (leftStickPolarCoordinate[0] > 0.5f)
            {
                rightStick[0] = 0.5f;
            }
            else if (leftStickPolarCoordinate[0] < -0.5f)
            {
                rightStick[0] = -0.5f;
            }

            joyMsg.axes[0] = -leftStick[0];
            joyMsg.axes[1] = leftStick[1];
            joyMsg.axes[2] = -rightStick[0];
            joyMsg.axes[3] = rightStick[1];
        }
        else if (mode == 2)
        {
            // NavMesh����̈ړ���������p����
            Vector3 directionCmd = robotNavigation.direction;
            // ���z�I�ȃW���C�X�e�B�b�N�̑����M�����쐬

            robotFaceVector = quadrupedTrajectoryGenerator.frontLeft.trajectoryBase.transform.position - quadrupedTrajectoryGenerator.rearLeft.trajectoryBase.transform.position;
            leftStickPolarCoordinate[0] = Mathf.Atan2(directionCmd.x, directionCmd.z) - Mathf.Atan2(robotFaceVector.x, robotFaceVector.z);
            leftStickPolarCoordinate[1] = 0.75f;
            // �f�J���g���W�\����
            leftStick[1] = leftStickPolarCoordinate[1] * Mathf.Cos(leftStickPolarCoordinate[0]);  // Unity��ROS�̍��W�n�͈قȂ�̂�ROS�ɂ�����y��cos�ŋ��߂Ă���_�ɒ���
            leftStick[0] = leftStickPolarCoordinate[1] * Mathf.Sin(leftStickPolarCoordinate[0]);  //
            // �E�ɂ��Ă�
            if (leftStickPolarCoordinate[0] >= -0.5f || leftStickPolarCoordinate[0] <= 0.5f)
            {
                rightStick[0] = leftStickPolarCoordinate[0];
            }
            else if (leftStickPolarCoordinate[0] > 0.5f)
            {
                rightStick[0] = 0.5f;
            }
            else if (leftStickPolarCoordinate[0] < -0.5f)
            {
                rightStick[0] = -0.5f;
            }

            joyMsg.axes[0] = -leftStick[0];
            joyMsg.axes[1] = leftStick[1];
            joyMsg.axes[2] = -rightStick[0];
            joyMsg.axes[3] = rightStick[1];
        }
        else if (mode == 3)
        {
            // ���z�I�ȃW���C�X�e�B�b�N�̒��i�݂̂̑���M�����쐬

            leftStickPolarCoordinate[0] = 0f;
            leftStickPolarCoordinate[1] = 0.75f;
            // �f�J���g���W�\����
            leftStick[1] = leftStickPolarCoordinate[1] * Mathf.Cos(leftStickPolarCoordinate[0]);  // Unity��ROS�̍��W�n�͈قȂ�̂�ROS�ɂ�����y��cos�ŋ��߂Ă���_�ɒ���
            leftStick[0] = leftStickPolarCoordinate[1] * Mathf.Sin(leftStickPolarCoordinate[0]);  //
            // �E�ɂ��Ă�
            if (leftStickPolarCoordinate[0] >= -0.5f || leftStickPolarCoordinate[0] <= 0.5f)
            {
                rightStick[0] = leftStickPolarCoordinate[0];
            }
            else if (leftStickPolarCoordinate[0] > 0.5f)
            {
                rightStick[0] = 0.5f;
            }
            else if (leftStickPolarCoordinate[0] < -0.5f)
            {
                rightStick[0] = -0.5f;
            }

            joyMsg.axes[0] = -leftStick[0];
            joyMsg.axes[1] = leftStick[1];
            joyMsg.axes[2] = -rightStick[0];
            joyMsg.axes[3] = rightStick[1];
        }

        if (DEBUG)
        {
            Debug.Log($"leftStickPolarCoordinate: [{leftStickPolarCoordinate[0]}, {leftStickPolarCoordinate[1]}]");
            Debug.Log($"leftStick: [{leftStick[0]}, {leftStick[1]}]");
            Debug.Log($"rightStick: [{rightStick[0]}, {rightStick[1]}]");
        }

        return new JoyData { JoyMsg = joyMsg, LeftStickPolarCoordinate = leftStickPolarCoordinate, LeftStick = leftStick, RightStick = rightStick };
    }
}