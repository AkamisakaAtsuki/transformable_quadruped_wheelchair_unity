using UnityEngine;
using UnityEngine.InputSystem;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// �o�͂������http://wiki.ros.org/joy�́uMicrosoft Xbox 360 Wireless Controller for Linux�v�Ɋ�Â�
// �����_�ł́Ajoy message��axes��0~3�ԁi0: Left/Right Axis stick left, 1: Up/Down Axis stick left, 2: Left/Right Axis stick right, 3: Up/Down Axis stick right�j�܂ł̂ݎ���

public class XboxControllerInput : MonoBehaviour
{
    [SerializeField] private InputActionAsset inputActions;

    // ROS�̍��W�n�ɉ����Ēl���擾
    public bool useRealController;
    public bool useRos;
    public string joyMsgName = "/joy";
    public JoyMsg joyMsg;
    ROSConnection m_Ros;

    private Vector2 moveInput;
    private Vector2 lookInput;
    private InputAction moveAction;
    private InputAction lookAction;

    void Awake()
    {
        if (useRealController)
        {
            var gameplayActions = inputActions.FindActionMap("QuadrupedControl");

            // JoyMsg�̏�����
            joyMsg.buttons = new int[11];
            joyMsg.axes = new float[8];

            moveAction = gameplayActions.FindAction("Move");
            lookAction = gameplayActions.FindAction("Look");

            if (useRos)
            {
                m_Ros = ROSConnection.GetOrCreateInstance();
                m_Ros.RegisterPublisher<JoyMsg>(joyMsgName);
            }
        }
    }

    void OnEnable()
    {
        if (useRealController)
        {
            moveAction.Enable();
            lookAction.Enable();
        }
    }

    void OnDisable()
    {
        moveAction.Disable();
        lookAction.Disable();
    }

    public JoyMsg GetJoyMsg()
    {
        moveInput = moveAction.ReadValue<Vector2>();  // unity�̍��W�n�Ŏ擾�����
        lookInput = lookAction.ReadValue<Vector2>();

        joyMsg.axes[0] = -moveInput.x;  // �������E�i�����F���j
        joyMsg.axes[1] = moveInput.y;   // �����㉺�i�����F��j
        joyMsg.axes[2] = -lookInput.x;  // �E�����E�i�����F���j
        joyMsg.axes[3] = lookInput.y;   // �E���㉺�i�����F��j

        if (useRos)
        {
            m_Ros.Publish(joyMsgName, joyMsg);
        }

        return joyMsg;
    }

    public float[] GetLeftStickPolarCoordinates()
    {
        moveInput = moveAction.ReadValue<Vector2>();
        float angle = Mathf.Atan2(moveInput.x, moveInput.y); // Unity�̍��W�n�̊֌W��Ax��y���t�ɂ��Ă���
        float magnitude = moveInput.magnitude;

        return new float[] { angle, magnitude };
    }

    public float[] GetLeftStick()
    {
        moveInput = moveAction.ReadValue<Vector2>();

        return new float[] { moveInput.x, moveInput.y };
    }

    public float[] GetRightStick()
    {
        lookInput = lookAction.ReadValue<Vector2>();

        return new float[] { lookInput.x, lookInput.y };
    }
}