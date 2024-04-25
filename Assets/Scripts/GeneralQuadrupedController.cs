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

    [Header("4�r�̃A�N�`���G�[�^ Param")]
    public controllerParam quadrupedJointParam;

    [Header("���̑��̃A�N�`���G�[�^ Param")]
    public controllerParam otherJointParam;

    [Header("4�r�̃A�N�`���G�[�^")]
    public quadrupedActuators quadrupedJoints; // �e�r3���R�x�ŌŒ�

    [Header("���̑��̃A�N�`���G�[�^")]
    public movableJoint[] otherJoints; // ��

    [Header("�ʒu�Œ�W���C���g�F�p�������������邽�߂ɕK�v")]
    public fixedJoint[] fJoints;

    [Header("����i�G���h�G�t�F�N�^�j")]
    public quadrupedToes toes;


    // �L�����u���[�V�����@�\
    public bool calibration;
    public bool positiveDirection;   // ����Ƀ`�F�b�N�������Ă�����S�ẴT�[�{�Ɋp�x30�x�𑗐M
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


        // ���W���C���g�̒l
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

    // ���݂̊p�x(�x���@)���擾
    public float[] GetCurrentJointRotations()
    {
        float[] list = new float[movableArticulations.Length];
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            float currentRadRotation = joint.link.GetComponent<ArticulationBody>().jointPosition[0];

            // offset���l��(�擾���͑���)
            list[i] = Mathf.Rad2Deg * currentRadRotation + joint.offset;
        }
        return list;
    }
    /*chatgpt*/
    // ���K�������W���C���g�p�x�Ɖ�]�����Ԃ�
    public float[] GetCurrentNormalizedJointRotationsAndDirections()
    {
        float[] list = new float[2 * movableArticulations.Length];
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            ArticulationBody articulation = joint.link.GetComponent<ArticulationBody>();

            float currentRadRotation = articulation.jointPosition[0];

            // offset���l�����Ă����Ȃ��Ă��A���K������΂��Ȃ�
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

    // revolute��joint�̐ݒ���s��
    public void SetXDriveParams(int[] stiffness, int[] damping, int[] forceLimit, float[] target, float[] targetVelocity)
    {
        // ���@����ɂ̓L�����u���[�V�����@�\�͂܂����ڂ��Ă��Ȃ�

        // target��-1����1�͈̔͂ŕ\���ꂽ���́B�ȉ��œK�؂Ȕ͈͂Ƀ}�b�s���O���Ă���K�p
        // targetVelocity��-1����1�͈̔͂ŕ\���ꂽ���́B�ȉ���0����360(360�x/s)�͈̔͂Ƀ}�b�s���O���Ă���K�p
        for (int i = 0; i < movableArticulations.Length; i++)
        {
            movableJoint joint = movableArticulations[i];
            ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;

            // offset���l��
            aDrive.target = Map(target[i], -1, 1, aDrive.lowerLimit, aDrive.upperLimit, joint.invertAxis) + joint.offset;
            aDrive.targetVelocity = Map(targetVelocity[i], -1, 1, 0, 360, false); // ���x�̓}�C�i�X�ɂȂ�Ȃ��̂�inversion=false
            movableArticulations[i].link.GetComponent<ArticulationBody>().xDrive = aDrive;
        }
    }

    public void SetXDriveParamsRawTarget(float[] target, float[] targetVelocity)
    {
        if (calibration == false)
        {
            // target��-1����1�͈̔͂ŕ\���ꂽ���́B�ȉ��œK�؂Ȕ͈͂Ƀ}�b�s���O���Ă���K�p
            // targetVelocity��-1����1�͈̔͂ŕ\���ꂽ���́B�ȉ���0����360(360�x/s)�͈̔͂Ƀ}�b�s���O���Ă���K�p
            for (int i = 0; i < target.Length; i++)
            {
                movableJoint joint = movableArticulations[i];
                ArticulationDrive aDrive = joint.link.GetComponent<ArticulationBody>().xDrive;
                /*aDrive.stiffness = stiffness[i];
                aDrive.damping = damping[i];
                aDrive.forceLimit = forceLimit[i];*/
                // offset���l��
                if (joint.invertAxis)
                {
                    aDrive.target = -target[i] + joint.offset;
                }
                else
                {
                    aDrive.target = target[i] + joint.offset;
                }
                //aDrive.target = Map(target[i], aDrive.lowerLimit, aDrive.upperLimit, aDrive.lowerLimit, aDrive.upperLimit, joint.invertAxis) + joint.offset;
                aDrive.targetVelocity = Map(targetVelocity[i], -1, 1, 0, 360, false); // ���x�̓}�C�i�X�ɂȂ�Ȃ��̂�inversion=false
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
                // �S�Ă̊֐߂�30�x�ɂ���
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
        // value��[start1, stop1]����[start2, stop2]�փ}�b�v����֐�
        // inversion���w�肷�邱�ƂŔ͈͂��t�]�ł���B
        // ���̏ꍇ��value��[start1, stop1]����[stop2, start2]�փ}�b�v����
        if (inversion == false)   // �l�̔��]���s�K�v
        {
            return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
        }
        else    // �l�̔��]���K�v
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
        // value��lower����upper�͈̔͂Ŕ��]����v���O����
        // inversion���w�肷�邱�ƂŔ͈͂��t�]�ł���B���̏ꍇ��value��upper����lower�͈̔͂Ŕ��]
        if (inversion == false)   // �l�̔��]���s�K�v
        {
            return (value - lower) / (upper - lower);
        }
        else    // �l�̔��]���K�v
        {
            return (upper - value) / (upper - lower);
        }
    }

    public void MoveJoint(movableJoint joint, float target)
    {
        // target��-1����1�ɐ��K�����ꂽ����
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
                // �x���@�֕ϊ���offset���l��(�擾���͑���)
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