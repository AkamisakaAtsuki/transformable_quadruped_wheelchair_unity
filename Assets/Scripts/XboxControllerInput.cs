using UnityEngine;
using UnityEngine.InputSystem;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// 出力する情報はhttp://wiki.ros.org/joyの「Microsoft Xbox 360 Wireless Controller for Linux」に基づく
// 現時点では、joy messageのaxesの0~3番（0: Left/Right Axis stick left, 1: Up/Down Axis stick left, 2: Left/Right Axis stick right, 3: Up/Down Axis stick right）までのみ実装

public class XboxControllerInput : MonoBehaviour
{
    [SerializeField] private InputActionAsset inputActions;

    // ROSの座標系に沿って値を取得
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

            // JoyMsgの初期化
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
        moveInput = moveAction.ReadValue<Vector2>();  // unityの座標系で取得される
        lookInput = lookAction.ReadValue<Vector2>();

        joyMsg.axes[0] = -moveInput.x;  // 左軸左右（正方：左）
        joyMsg.axes[1] = moveInput.y;   // 左軸上下（正方：上）
        joyMsg.axes[2] = -lookInput.x;  // 右軸左右（正方：左）
        joyMsg.axes[3] = lookInput.y;   // 右軸上下（正方：上）

        if (useRos)
        {
            m_Ros.Publish(joyMsgName, joyMsg);
        }

        return joyMsg;
    }

    public float[] GetLeftStickPolarCoordinates()
    {
        moveInput = moveAction.ReadValue<Vector2>();
        float angle = Mathf.Atan2(moveInput.x, moveInput.y); // Unityの座標系の関係上、xとyを逆にしている
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