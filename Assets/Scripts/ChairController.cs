using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[RequireComponent(typeof(GeneralQuadrupedController))]

public class ChairController : MonoBehaviour
{
    public GameObject robot;
    private GeneralQuadrupedController generalQuadrupedController;

    public GameObject chairArmJointObj;
    public GameObject slidarJointObj;
    public GameObject chairAngleJointObj;
    public GameObject bottomSeatJointObj;
    public GameObject backSeatJointObj;
    public GameObject footRestJointObj;
    public GameObject leftArmSupportJointObj;
    public GameObject rightArmSupportJointObj;

    public GameObject humanFixingParts1;
    public GameObject humanFixingParts2;
    public GameObject humanFixingParts3;
    public GameObject humanFixingParts4;

    private GeneralQuadrupedController.movableJoint chairArmJoint;
    private GeneralQuadrupedController.movableJoint slidarJoint;
    private GeneralQuadrupedController.movableJoint chairAngleJoint;
    private GeneralQuadrupedController.movableJoint bottomSeatJoint;
    private GeneralQuadrupedController.movableJoint backSeatJoint;
    private GeneralQuadrupedController.movableJoint footRestJoint;
    private GeneralQuadrupedController.movableJoint leftArmSupportJoint;
    private GeneralQuadrupedController.movableJoint rightArmSupportJoint;

    public bool naturalMode = false;
    public bool upMode = false;
    public bool downMode = false;

    /* private struct chairPose
     {
         public 
     }*/

    // Start is called before the first frame update
    void Start()
    {
        generalQuadrupedController = this.GetComponent<GeneralQuadrupedController>();

        /*humanFixingParts1.SetActive(false);
        humanFixingParts2.SetActive(false);
        humanFixingParts3.SetActive(false);
        humanFixingParts4.SetActive(false);*/
        humanFixingParts1.GetComponent<Renderer>().enabled = false;
        humanFixingParts2.GetComponent<Renderer>().enabled = false;
        humanFixingParts3.GetComponent<Renderer>().enabled = false;
        humanFixingParts4.GetComponent<Renderer>().enabled = false;

        // リンクを持つGameObjectと比較して4脚コントローラからmovableJointを取得
        for (int i = 0; i < generalQuadrupedController.otherJoints.Length; i++)
        {
            if (generalQuadrupedController.otherJoints[i].link == chairArmJointObj)
            {
                chairArmJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == slidarJointObj)
            {
                slidarJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == chairAngleJointObj)
            {
                chairAngleJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == bottomSeatJointObj)
            {
                bottomSeatJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == backSeatJointObj)
            {
                backSeatJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == footRestJointObj)
            {
                footRestJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == leftArmSupportJointObj)
            {
                leftArmSupportJoint = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == rightArmSupportJointObj)
            {
                rightArmSupportJoint = generalQuadrupedController.otherJoints[i];
            }
        }     
    }

    // Update is called once per frame
    void Update()
    {
        if (naturalMode)
        {
            generalQuadrupedController.startMotion(chairArmJoint,          0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(slidarJoint,            0.01f, 0.05f, 0.325f);
            generalQuadrupedController.startMotion(chairAngleJoint,        0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(bottomSeatJoint,       0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(footRestJoint,          0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(backSeatJoint,         0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(leftArmSupportJoint,    0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(rightArmSupportJoint,   0.01f, 0.01f, 0f);
        }
        else if (upMode)
        {
            generalQuadrupedController.startMotion(chairArmJoint,          0.01f, 0.01f, 45f);
            generalQuadrupedController.startMotion(slidarJoint,            0.01f, 0.05f, 0.325f);
            generalQuadrupedController.startMotion(chairAngleJoint,        0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(bottomSeatJoint,       0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(footRestJoint,          0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(backSeatJoint,         0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(leftArmSupportJoint,    0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(rightArmSupportJoint,   0.01f, 0.01f, 0f);
        }
        else if (downMode)
        {
            generalQuadrupedController.startMotion(chairArmJoint,          0.01f, 0.01f, 20f);
            generalQuadrupedController.startMotion(slidarJoint,            0.01f, 0.05f, 0f);
            generalQuadrupedController.startMotion(chairAngleJoint,        0.01f, 0.01f, 180f);
            generalQuadrupedController.startMotion(bottomSeatJoint,       0.01f, 0.01f, 0f);
            generalQuadrupedController.startMotion(footRestJoint,          0.01f, 0.01f, 45f);
            generalQuadrupedController.startMotion(backSeatJoint,         0.01f, 0.01f, 45f);
            generalQuadrupedController.startMotion(leftArmSupportJoint,    0.01f, 0.01f, 45f);
            generalQuadrupedController.startMotion(rightArmSupportJoint,   0.01f, 0.01f, 45f);
        }
    }

    public void setChairArmJoint(float target)
    {
        // -1~1を0~45にマッピング
        var angle = generalQuadrupedController.Map(target, -1f, 1f, 0f, 45f, false);
        generalQuadrupedController.MoveJoint(chairArmJoint, angle);
    }

    public void setSlidarJoint(float target)
    {
        // -1~1を0~0.325にマッピング
        var pos = generalQuadrupedController.Map(target, -1f, 1f, 0f, 0.325f, false);
        generalQuadrupedController.MoveJoint(slidarJoint, pos);
    }

    public void setBackSeatJoint(float target)
    {
        // -1~1を0~0.325にマッピング
        var pos = generalQuadrupedController.Map(target, -1f, 1f, 0f, 0.325f, false);
        generalQuadrupedController.MoveJoint(backSeatJoint, pos);
    }

    public void setDownMode()
    {
        // generalQuadrupedController.MoveJoint(chairArmJoint,20f);
        generalQuadrupedController.MoveJoint(chairArmJoint, 0f);
        generalQuadrupedController.MoveJoint(slidarJoint, 0.325f);
        generalQuadrupedController.MoveJoint(chairAngleJoint, 0f);
        /*generalQuadrupedController.MoveJoint(chairAngleJoint, 180f);*/
        generalQuadrupedController.MoveJoint(bottomSeatJoint, 0f);
        /*generalQuadrupedController.MoveJoint(footRestJoint, 30f); // 変更前は45f
        generalQuadrupedController.MoveJoint(backSeatJoint, 45f);
        generalQuadrupedController.MoveJoint(leftArmSupportJoint, 45f);
        generalQuadrupedController.MoveJoint(rightArmSupportJoint, 45f);*/
    }

    public void resetChair()
    {
        generalQuadrupedController.MoveJoint(chairArmJoint, 0f);
        generalQuadrupedController.MoveJoint(slidarJoint, 0.325f);
        generalQuadrupedController.MoveJoint(chairAngleJoint, 0f);
        generalQuadrupedController.MoveJoint(bottomSeatJoint, 0f);
        generalQuadrupedController.MoveJoint(footRestJoint, 0f);
        generalQuadrupedController.MoveJoint(backSeatJoint, 0f);
        generalQuadrupedController.MoveJoint(leftArmSupportJoint, 0f);
        generalQuadrupedController.MoveJoint(rightArmSupportJoint, 0f);
    }


    public void setFoldingTheChair()
    {
        setChairArmJoint(-1f);
        setSlidarJoint(1f);
        setBackSeatJoint(0f);

        generalQuadrupedController.MoveJoint(backSeatJoint, 90f);
        generalQuadrupedController.MoveJoint(footRestJoint, 0f);
        generalQuadrupedController.MoveJoint(leftArmSupportJoint, 0f);
        generalQuadrupedController.MoveJoint(rightArmSupportJoint, 0f);
    }
}
