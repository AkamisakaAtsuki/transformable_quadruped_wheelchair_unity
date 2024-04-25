using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// センサー値はROSの座標系で取得

public class QuadrupedSensors : MonoBehaviour
{
    public bool DEBUG = false;
    // IMU, touch sensor
    public GameObject imuSensorLocation;
    public GameObject frontLeftToe;
    public GameObject frontRightToe;
    public GameObject rearLeftToe;
    public GameObject rearRightToe;

    public bool publishRosMsg = false;
    ROSConnection m_Ros;

    public Lidar lidar1;
    public Lidar lidar2;

    private ArticulationBody ab;
    private Vector3 currentVelocity;
    private Vector3 currentAngularVelocity;
    private Vector3 lastVelocity;
    private Vector3 lastAngularVelocity;
    private Vector3 acceleration;
    private Vector3 angularAcceleration;

    private ToeCollisionDetector frontLeftToeDetector;
    private ToeCollisionDetector frontRightToeDetector;
    private ToeCollisionDetector rearLeftToeDetector;
    private ToeCollisionDetector rearRightToeDetector;

    [SerializeField]
    private string imuTopicName = "/base_imu";
    private string frontLeftToeForceTopicName = "/front_left_toe_force";
    private string frontRightToeForceTopicName = "/front_right_toe_force";
    private string rearLeftToeForceTopicName = "rear_left_toe_force";
    private string rearRightToeForceTopicName = "/rear_right_toe_force";
    private string velocityTopicName = "/base_velocity";
    private string angularVelocityTopicName = "/base_angular_velocity";

    [SerializeField]
    private ImuMsg imu;
    [SerializeField]
    private Vector3Msg frontLeftForce;
    [SerializeField]
    private Vector3Msg frontRightForce;
    [SerializeField]
    private Vector3Msg rearLeftForce;
    [SerializeField]
    private Vector3Msg rearRightForce;

    // Start is called before the first frame update
    void Start()
    {
        ab = imuSensorLocation.GetComponent<ArticulationBody>();
        if (ab == null)
        {
            Debug.LogError("ArticulationBody component is missing on the IMU sensor location object.");
        }

        frontLeftToeDetector = AddToeCollisionDetector(frontLeftToe);
        frontRightToeDetector = AddToeCollisionDetector(frontRightToe);
        rearLeftToeDetector = AddToeCollisionDetector(rearLeftToe);
        rearRightToeDetector = AddToeCollisionDetector(rearRightToe);

        // Set attachedPart for each ToeCollisionDetector
        frontLeftToe.GetComponent<ToeCollisionDetector>().attachedPart = frontLeftToe.transform.parent.gameObject;
        frontRightToe.GetComponent<ToeCollisionDetector>().attachedPart = frontRightToe.transform.parent.gameObject;
        rearLeftToe.GetComponent<ToeCollisionDetector>().attachedPart = rearLeftToe.transform.parent.gameObject;
        rearRightToe.GetComponent<ToeCollisionDetector>().attachedPart = rearRightToe.transform.parent.gameObject;

        lastVelocity = ab.velocity;
        // lastAngularVelocity = ab.angularVelocity;

        if (publishRosMsg)
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<ImuMsg>(imuTopicName);
            m_Ros.RegisterPublisher<Vector3Msg>(frontLeftToeForceTopicName);
            m_Ros.RegisterPublisher<Vector3Msg>(frontRightToeForceTopicName);
            m_Ros.RegisterPublisher<Vector3Msg>(rearLeftToeForceTopicName);
            m_Ros.RegisterPublisher<Vector3Msg>(rearRightToeForceTopicName);
        }
    }

    private ToeCollisionDetector AddToeCollisionDetector(GameObject toe)
    {
        ToeCollisionDetector detector = toe.AddComponent<ToeCollisionDetector>();
        return detector;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;

        // Calculate acceleration
        currentVelocity = ab.velocity;
        acceleration = (currentVelocity - lastVelocity) / deltaTime;
        lastVelocity = currentVelocity;

        // Calculate angular acceleration
        currentAngularVelocity = ab.angularVelocity;
        angularAcceleration = (currentAngularVelocity - lastAngularVelocity) / deltaTime;
        lastAngularVelocity = currentAngularVelocity;

        if (DEBUG)
        {
            frontLeftForce = GetFrontLeftForce();
            frontRightForce = GetFrontRightForce();
            rearLeftForce = GetRearLeftForce();
            rearRightForce = GetRearRightForce();
        }

        if (publishRosMsg)
        {
            m_Ros.Publish(imuTopicName, GetImuData());
            m_Ros.Publish(frontLeftToeForceTopicName, GetFrontLeftForce());
            m_Ros.Publish(frontRightToeForceTopicName, GetFrontRightForce());
            m_Ros.Publish(rearLeftToeForceTopicName, GetRearLeftForce());
            m_Ros.Publish(rearRightToeForceTopicName, GetRearRightForce());
            m_Ros.Publish(velocityTopicName, GetVelocity());
            m_Ros.Publish(angularVelocityTopicName, GetAngularVelocity());
        }
    }

    public ImuMsg GetImuData()
    {
        imu = new ImuMsg()
        {
            linear_acceleration = acceleration.To<FLU>(),
            // orientation = ab.transform.rotation.To<FLU>(),
            angular_velocity = ab.angularVelocity.To<FLU>()
        };
        return imu;
    }

    public Vector3Msg GetVelocity()
    {
        return currentVelocity.To<FLU>();
    }
    public Vector3Msg GetAngularVelocity()
    {
        return ab.angularVelocity.To<FLU>();
    }

    public Vector3Msg GetAngularAcceleration()
    {
        return angularAcceleration.To<FLU>();
    }

    public Vector3Msg GetRollPitchYaw()
    {
        Vector3 eulerRadians = ab.transform.rotation.eulerAngles * Mathf.Deg2Rad;

        // Adjust the angles to be in the range -pi to pi
        eulerRadians.x = (eulerRadians.x > Mathf.PI) ? eulerRadians.x - 2 * Mathf.PI : eulerRadians.x;
        eulerRadians.y = (eulerRadians.y > Mathf.PI) ? eulerRadians.y - 2 * Mathf.PI : eulerRadians.y;
        eulerRadians.z = (eulerRadians.z > Mathf.PI) ? eulerRadians.z - 2 * Mathf.PI : eulerRadians.z;

        return eulerRadians.To<FLU>();
    }

    public Vector3Msg GetFrontLeftForce()
    {
        var forceFLU = frontLeftToeDetector.toeForce.To<FLU>();
        frontLeftForce = new Vector3Msg()
        {
            x = forceFLU.x,
            y = forceFLU.y,
            z = forceFLU.z
        };
        return frontLeftForce;
    }

    public Vector3Msg GetFrontRightForce()
    {
        var forceFLU = frontRightToeDetector.toeForce.To<FLU>();
        frontRightForce = new Vector3Msg()
        {
            x = forceFLU.x,
            y = forceFLU.y,
            z = forceFLU.z
        };
        return frontRightForce;
    }

    public Vector3Msg GetRearLeftForce()
    {
        var forceFLU = rearLeftToeDetector.toeForce.To<FLU>();
        rearLeftForce = new Vector3Msg()
        {
            x = forceFLU.x,
            y = forceFLU.y,
            z = forceFLU.z
        };
        return rearLeftForce; 
    }

    public Vector3Msg GetRearRightForce()
    {
        var forceFLU = rearRightToeDetector.toeForce.To<FLU>();
        rearRightForce = new Vector3Msg()
        {
            x = forceFLU.x,
            y = forceFLU.y,
            z = forceFLU.z
        };
        return rearRightForce; 
    }
}
