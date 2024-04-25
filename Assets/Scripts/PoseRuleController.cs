using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(GeneralQuadrupedController))]

public class PoseRuleController : MonoBehaviour
{
    private GeneralQuadrupedController generalQuadrupedController;

    public GameObject frontLeftTireObj;
    public GameObject frontRightTireObj;
    public GameObject rearLeftTireObj;
    public GameObject rearRightTireObj;
    public GameObject frontLeftUpper2Obj;
    public GameObject frontRightUpper2Obj;
    public GameObject rearLeftUpper2Obj;
    public GameObject rearRightUpper2Obj;

    private GeneralQuadrupedController.movableJoint frontLeftTire;
    private GeneralQuadrupedController.movableJoint frontRightTire;
    private GeneralQuadrupedController.movableJoint rearLeftTire;
    private GeneralQuadrupedController.movableJoint rearRightTire;
    private GeneralQuadrupedController.movableJoint frontLeftUpper2;
    private GeneralQuadrupedController.movableJoint frontRightUpper2;
    private GeneralQuadrupedController.movableJoint rearLeftUpper2;
    private GeneralQuadrupedController.movableJoint rearRightUpper2;

    // Start is called before the first frame update
    void Start()
    {
        generalQuadrupedController = this.GetComponent<GeneralQuadrupedController>();
        
        for (int i = 0; i < generalQuadrupedController.otherJoints.Length; i++)
        {
            if (generalQuadrupedController.otherJoints[i].link == frontLeftTireObj)
            {
                frontLeftTire = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == frontRightTireObj)
            {
                frontRightTire = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == rearLeftTireObj)
            {
                rearLeftTire = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == rearRightTireObj)
            {
                rearRightTire = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == frontLeftUpper2Obj)
            {
                frontLeftUpper2 = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == frontRightUpper2Obj)
            {
                frontRightUpper2 = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == rearLeftUpper2Obj)
            {
                rearLeftUpper2 = generalQuadrupedController.otherJoints[i];
            }
            else if (generalQuadrupedController.otherJoints[i].link == rearRightUpper2Obj)
            {
                rearRightUpper2 = generalQuadrupedController.otherJoints[i];
            }
        }
    }

    public void VehicleMode()
    {
        /*generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftUpper, 0.01f, 0.1f, -20);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftLower, 0.01f, 0.1f, -130);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftUpper, 0.01f, 0.1f, 20);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftLower, 0.01f, 0.1f, -150);  // ここズレてる
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightUpper, 0.01f, 0.1f, 20);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightLower, 0.01f, 0.1f, -130);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightUpper, 0.01f, 0.1f, -20);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightLower, 0.01f, 0.1f, -150);  // ここズレてる*/
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftUpper, 0.01f, 0.1f, 0);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftLower, 0.01f, 0.1f, -130);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftUpper, 0.01f, 0.1f, 0);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftLower, 0.01f, 0.1f, -150);  // ここズレてる
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightUpper, 0.01f, 0.1f, 0);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightLower, 0.01f, 0.1f, -130);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightUpper, 0.01f, 0.1f, -0);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightLower, 0.01f, 0.1f, -150);  // ここズレてる
    }

    public void TgMode()
    {
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftUpper, 0.01f, 0.1f, 25);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontLeftLower, 0.01f, 0.1f, -90);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftUpper, 0.01f, 0.1f, 65);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearLeftLower, 0.01f, 0.1f, -110);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightUpper, 0.01f, 0.1f, 65);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.frontRightLower, 0.01f, 0.1f, -90);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightUpper, 0.01f, 0.1f, 25);
        generalQuadrupedController.startMotion(generalQuadrupedController.quadrupedJoints.rearRightLower, 0.01f, 0.1f, -110);
    }

    public void AwcMode(float go, bool turn)
    {
        if (turn)
        {
            generalQuadrupedController.startMotion(frontLeftUpper2, 0.01f, 0.1f, 60);
            generalQuadrupedController.startMotion(frontRightUpper2, 0.01f, 0.1f, -60);
            generalQuadrupedController.startMotion(rearLeftUpper2, 0.01f, 0.1f, -60);
            generalQuadrupedController.startMotion(rearRightUpper2, 0.01f, 0.1f, 60);

            generalQuadrupedController.startMotion(frontLeftTire, 0.01f, 0.01f, generalQuadrupedController.GetMovableJointPosition(frontLeftTire) - go);
            generalQuadrupedController.startMotion(frontRightTire, 0.01f, 0.01f, generalQuadrupedController.GetMovableJointPosition(frontRightTire) + go);
            generalQuadrupedController.startMotion(rearLeftTire, 0.01f, 0.01f, generalQuadrupedController.GetMovableJointPosition(rearLeftTire) - go);
            generalQuadrupedController.startMotion(rearRightTire, 0.01f, 0.01f, generalQuadrupedController.GetMovableJointPosition(rearRightTire) + go);
        }
        else
        {
            generalQuadrupedController.startMotion(frontLeftUpper2, 0.01f, 0.1f, 0);
            generalQuadrupedController.startMotion(frontRightUpper2, 0.01f, 0.1f, 0);
            generalQuadrupedController.startMotion(rearLeftUpper2, 0.01f, 0.1f, 0);
            generalQuadrupedController.startMotion(rearRightUpper2, 0.01f, 0.1f, 0);

            generalQuadrupedController.startMotion(frontLeftTire, 0.01f, 0.01f,  generalQuadrupedController.GetMovableJointPosition(frontLeftTire) + go);
            generalQuadrupedController.startMotion(frontRightTire, 0.01f, 0.01f, generalQuadrupedController.GetMovableJointPosition(frontRightTire) + go);
            generalQuadrupedController.startMotion(rearLeftTire, 0.01f, 0.01f,   generalQuadrupedController.GetMovableJointPosition(rearLeftTire) + go);
            generalQuadrupedController.startMotion(rearRightTire, 0.01f, 0.01f,  generalQuadrupedController.GetMovableJointPosition(rearRightTire) + go);
        }
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
