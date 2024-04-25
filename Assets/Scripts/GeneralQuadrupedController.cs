using System.Linq;
using UnityEngine;

public class GeneralQuadrupedController : MonoBehaviour
{
    [System.Serializable]
    public struct controllerParam
    {
        public float stiffness;
        public float damping;
        public float forceLimit;
        public float speed;// = 5f; // Units: degree/s
        public float torque;// = 100f; // Units: Nm or N
        public float acceleration;// = 5f;// Units: m/s^2 / degree/s^2
    }

    [System.Serializable]
    public struct movableJoint
    {
        public GameObject link;
        public bool invertAxis;
        public int offset;
    }

    [System.Serializable]
    public struct fixedJoint
    {
        public GameObject link;
    }

    [System.Serializable]
    public struct quadrupedActuators
    {
        public movableJoint frontLeftHip;
        public movableJoint frontLeftUpper;
        public movableJoint frontLeftLower;
        public movableJoint frontRightHip;
        public movableJoint frontRightUpper;
        public movableJoint frontRightLower;
        public movableJoint rearLeftHip;
        public movableJoint rearLeftUpper;
        public movableJoint rearLeftLower;
        public movableJoint rearRightHip;
        public movableJoint rearRightUpper;
        public movableJoint rearRightLower;
    }

    [System.Serializable]
    public struct quadrupedToes
    {
        public GameObject frontLeftToe;
        public GameObject frontRightToe;
        public GameObject rearLeftToe;
        public GameObject rearRightToe;
    }

    public GameObject rootArticulation;

    [Header("4脚のアクチュエータ Param")]
    public controllerParam quadrupedJointParam;

    [Header("その他のアクチュエータ Param")]
    public controllerParam otherJointParam;

    [Header("4脚のアクチュエータ")]
    public quadrupedActuators quadrupedJoints; // 各脚3自由度で固定

    [Header("その他のアクチュエータ")]
    public movableJoint[] otherJoints; // 可変

    [Header("位置固定ジョイント：姿勢を初期化するために必要")]
    public fixedJoint[] fJoints;

    [Header("足先（エンドエフェクタ）")]
    public quadrupedToes toes;


    // キャリブレーション機能
    public bool calibration;
    public bool positiveDirection;   // これにチェックが入っていたら全てのサーボに角度30度を送信
    public bool reset;

    private movableJoint[] movableArticulations;


    private float[] jointValueBuffer;

    private ArticulationBodyReset articulationBodyReset;

    private void Start()
    {
        movableArticulations = new movableJoint[12 + otherJoints.Length];
      
        articulationBodyReset = new ArticulationBodyReset();
        articulationBodyReset.InitializeArticulationBodies(rootArticulation.GetComponent<ArticulationBody>());

        StoreMovableArticulations();


        // 可動ジョイントの値
        jointValueBuffer = new float[12 + otherJoints.Length];

        

        for (int i = 0; i < jointValueBuffer.Length; i++)
        {
            jointValueBuffer[i] = 0f;
        }
    }

    private void StoreMovableArticulations()
    {
        movableArticulations[0] = quadrupedJoints.frontLeftHip;
        movableArticulations[1] = quadrupedJoints.frontLeftUpper;
        movableArticulations[2] = quadrupedJoints.frontLeftLower;
        movableArticulations[3] = quadrupedJoints.frontRightHip;
        movableArticulations[4] = quadrupedJoints.frontRightUpper;
        movableArticulations[5] = quadrupedJoints.frontRightLower;
        movableArticulations[6] = quadrupedJoints.rearLeftHip;
        movableArticulations[7] = quadrupedJoints.rearLeftUpper;
        movableArticulations[8] = quadrupedJoints.rearLeftLower;
        movableArticulations[9] = quadrupedJoints.rearRightHip;
        movableArticulations[10] = quadrupedJoints.rearRightUpper;
        movableArticulations[11] = quadrupedJoints.rearRightLower;
        for (int i = 0; i < otherJoints.Length; i++)
        {
            movableArticulations[12 + i] = otherJoints[i];
        }

        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
            if (i < 12)
            {
                aDrive.stiffness = quadrupedJointParam.stiffness;
                aDrive.damping = quadrupedJointParam.damping;
                aDrive.forceLimit = quadrupedJointParam.forceLimit;
                //aDrive.targetVelocity = quadrupedJointParam.speed;
            }
            else
            {
                aDrive.stiffness = otherJointParam.stiffness;
                aDrive.damping = otherJointParam.damping;
                aDrive.forceLimit = otherJointParam.forceLimit;
                //aDrive.targetVelocity = otherJointParam.speed;
            }
            
            movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
        }
    } 

    // 現在の角度(度数法)を取得
    public float[] GetCurrentJointRotations()
    {
        float[] list = new float[movableArticulations.Length];
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            float currentRadRotation = joint.link.GetComponent<ArticulationBody>().jointPosition[0];

            // offsetを考慮(取得時は足す)
            list[i] = Mathf.Rad2Deg * currentRadRotation + joint.offset;
        }
        return list;
    }
    /*chatgpt*/
    // 正規化したジョイント角度と回転方向返す
    public float[] GetCurrentNormalizedJointRotationsAndDirections()
    {
        float[] list = new float[2 * movableArticulations.Length];
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            ArticulationBody articulation = joint.link.GetComponent<ArticulationBody>();

            float currentRadRotation = articulation.jointPosition[0];

            // offsetを考慮してもしなくても、正規化すればおなじ
            float currentDegRotation = Mathf.Rad2Deg * currentRadRotation;
            float normalizedRotation = GetNormalizedValue(currentDegRotation,
                                                          articulation.xDrive.lowerLimit,
                                                          articulation.xDrive.upperLimit,
                                                          joint.invertAxis);
            //float normalizedRotation = (currentDegRotation - articulation.xDrive.lowerLimit)
            //                           / (articulation.xDrive.upperLimit - articulation.xDrive.lowerLimit);
            list[i] = normalizedRotation;

            if (normalizedRotation > jointValueBuffer[i])
            {
                list[movableArticulations.Length + i] = 1f;
            }
            else
            {
                list[movableArticulations.Length + i] = -1f;
            }
            jointValueBuffer[i] = normalizedRotation;
        }
        return list;
    }

    // revoluteのjointの設定を行う
    public void SetXDriveParams(int[] stiffness, int[] damping, int[] forceLimit, float[] target, float[] targetVelocity)
    {
        // ※　これにはキャリブレーション機能はまだ搭載していない

        // targetは-1から1の範囲で表されたもの。以下で適切な範囲にマッピングしてから適用
        // targetVelocityは-1から1の範囲で表されたもの。以下で0から360(360度/s)の範囲にマッピングしてから適用
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;

            // offsetを考慮
            aDrive.target = Map(target[i], -1, 1, aDrive.lowerLimit, aDrive.upperLimit, joint.invertAxis) + joint.offset;
            aDrive.targetVelocity = Map(targetVelocity[i], -1, 1, 0, 360, false); // 速度はマイナスにならないのでinversion=false
            movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
        }
    }

    public void SetXDriveParamsRawTarget(float[] target, float[] targetVelocity)
    {
        if (calibration == false)
        {
            // targetは-1から1の範囲で表されたもの。以下で適切な範囲にマッピングしてから適用
            // targetVelocityは-1から1の範囲で表されたもの。以下で0から360(360度/s)の範囲にマッピングしてから適用
            for (int i = 0; i < target.Length; i++)
            {
                movableJoint joint = movableArticulations[i];
                ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
                /*aDrive.stiffness = stiffness[i];
                aDrive.damping = damping[i];
                aDrive.forceLimit = forceLimit[i];*/
                // offsetを考慮
                if (joint.invertAxis)
                {
                    aDrive.target = -target[i] + joint.offset;
                }
                else
                {
                    aDrive.target = target[i] + joint.offset;
                }
                //aDrive.target = Map(target[i], aDrive.lowerLimit, aDrive.upperLimit, aDrive.lowerLimit, aDrive.upperLimit, joint.invertAxis) + joint.offset;
                aDrive.targetVelocity = Map(targetVelocity[i], -1, 1, 0, 360, false); // 速度はマイナスにならないのでinversion=false
                movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
            }
        }
        else if (calibration == true && positiveDirection == false)
        {
            for (int i = 0; i < movableArticulations.Length; i++)
            {
                movableJoint joint = movableArticulations[i];
                ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
                /*aDrive.stiffness = stiffness[i];
                aDrive.damping = damping[i];
                aDrive.forceLimit = forceLimit[i];*/
                aDrive.target = joint.offset;
                movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
            }
        }
        else
        {
            for (int i = 0; i < movableArticulations.Length; i++)
            {
                // 全ての関節を30度にする
                movableJoint joint = movableArticulations[i];
                ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
                /*aDrive.stiffness = stiffness[i];
                aDrive.damping = damping[i];
                aDrive.forceLimit = forceLimit[i];*/
                if (joint.invertAxis)
                {
                    aDrive.target = -30 + joint.offset;
                }
                else
                {
                    aDrive.target = 30 + joint.offset;
                }
                movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
            }
        }
    }
   
    public float Map(float value, float start1, float stop1, float start2, float stop2, bool inversion)
    {
        // valueを[start1, stop1]から[start2, stop2]へマップする関数
        // inversionを指定することで範囲を逆転できる。
        // その場合はvalueを[start1, stop1]から[stop2, start2]へマップする
        if (inversion == false)   // 値の反転が不必要
        {
            return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
        }
        else    // 値の反転が必要
        {
            return stop2 + (start2 - stop2) * ((value - start1) / (stop1 - start1));
        }
    }

   /* public float Map(float value, float start1, float stop1, float start2, float stop2, bool inversion)
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
        Debug.Log("MAp");
    }*/


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

    public void MoveJoint(movableJoint joint, float target)
    {
        // targetは-1から1に正規化されたもの
        ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
 
        if (joint.invertAxis)
        {
            aDrive.target = -target + joint.offset;
        }
        else
        {
            aDrive.target = target + joint.offset;
        }
        joint.link.GetComponent<ArticulationBody>().xDrive = aDrive;
    }

    public float GetMovableJointPosition(movableJoint joint)
    {
        float currentPosition = joint.link.GetComponent<ArticulationBody>().jointPosition[0];
        
        switch (joint.link.GetComponent<ArticulationBody>().jointType.ToString())
        {
            case "RevoluteJoint":
                // 度数法へ変換＆offsetを考慮(取得時は足す)
                currentPosition = Mathf.Rad2Deg * currentPosition + joint.offset;
                break;
            case "PrismaticJoint":
                // Debug.Log("Pris");
                break;
        }


        if (joint.invertAxis)
        {
            return -currentPosition;
        }
        else
        {
            return currentPosition;
        }
    }
    /*chatgpt*/
    public void startMotion(movableJoint joint, float delta, float speed, float target)
    {
        float currentPos = GetMovableJointPosition(joint);
        float _delta = target - currentPos;

        if (Mathf.Abs(_delta) > delta)
        {
            MoveJoint(joint, currentPos + _delta * speed);
        }
    }

    private void Update()
    {
        if (calibration)
        {
            // int[] stf = new int[movableArticulations.Length]; // [10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000];
            // int[] dmp = new int[movableArticulations.Length]; // [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100];
            // int[] fcl = new int[movableArticulations.Length]; // [1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000];
            float[] trg = new float[movableArticulations.Length]; //[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            float[] trgvel = new float[movableArticulations.Length]; // [100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100];
            for (int i = 0; i < movableArticulations.Length; i++)
            {
                // stf[i] = 10000;
                // dmp[i] = 100;
                // fcl[i] = 1000;
                trgvel[i] = 100;
            }
            if (positiveDirection)
            {
                for (int i = 0; i < movableArticulations.Length; i++)
                {
                    trg[i] = 30;
                }
            }
            else
            {
                for (int i = 0; i < movableArticulations.Length; i++)
                {
                    trg[i] = 0;
                }
            }

            SetXDriveParamsRawTarget(trg, trgvel);
        }
        if (reset)
        {
            articulationBodyReset.ResetArticulationBodies(rootArticulation, new Vector3(0f, 0f, 0f));
        }
    }
}