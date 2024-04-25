using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class ApproachReward
{
    private float lastDistanceToTarget;
    private Transform targetTransform;
    private Transform robotTransform;
    private float rewardCloser;
    private float rewardSame;
    private float rewardFarther;

    public void Initialize(Transform targetTransform, Transform robotTransform, float rewardCloser = 1f, float rewardSame = 0f, float rewardFarther = -1f)
    {
        lastDistanceToTarget = (targetTransform.position - robotTransform.position).magnitude;
        this.targetTransform = targetTransform;
        this.robotTransform = robotTransform;
        this.rewardCloser = rewardCloser;
        this.rewardSame = rewardSame;
        this.rewardFarther = rewardFarther;
    }

    public float Calculate()
    {
        float nowDistanceToTarget = (targetTransform.position - robotTransform.position).magnitude;

        if (nowDistanceToTarget < lastDistanceToTarget)  /*近づいた*/
        {
            lastDistanceToTarget = nowDistanceToTarget;
            return rewardCloser;
        }
        else if (nowDistanceToTarget == lastDistanceToTarget) /*同じ*/
        {
            return rewardSame;
        }
        else   /*遠ざかった*/
        {
            return rewardFarther;
        }
    }
}

public class TargetTouchReward
{
    private float touchReward;
    private float touchThreshold;
    private bool drawConcentricCircles;
    private Transform targetTransform;
    private Vector3 lastTargetPosition;
    private Transform robotTransform;
    private GameObject circlesContainer;

    public void Initialize(Transform targetTransform, Transform robotTransform, float touchReward, float touchThreshold, bool drawConcentricCircles)
    {
        this.targetTransform = targetTransform;
        this.lastTargetPosition = targetTransform.position;
        this.robotTransform = robotTransform;
        this.touchReward = touchReward;
        this.touchThreshold = touchThreshold;
        this.drawConcentricCircles = drawConcentricCircles;
        circlesContainer = new GameObject("CirclesContainer");

        DrawCircles();
    }

    public float Calculate(ref bool trigEndEpisode)
    {
        if (HasTargetPositionChanged())
        {
            DrawCircles();
        }


        // 3次元上の距離ではなく、2次元上の距離
        Vector3 targetPositionFlat = new Vector3(targetTransform.position.x, 0f, targetTransform.position.z);
        Vector3 robotPositionFlat = new Vector3(robotTransform.position.x, 0f, robotTransform.position.z);

        float distanceToTarget = (targetPositionFlat - robotPositionFlat).magnitude;


        if (distanceToTarget <= touchThreshold)
        {
            trigEndEpisode = true;
            return touchReward;
        }
        else
        {
            return 0f;
        }
    }

    private bool HasTargetPositionChanged()
    {
        // 現在の位置と前回の位置を比較
        Vector3 currentPosition = targetTransform.position;
        bool positionChanged = currentPosition != lastTargetPosition;

        // 最後の位置を更新
        lastTargetPosition = currentPosition;

        return positionChanged;
    }

    public void DrawCircles()
    {
        if (drawConcentricCircles)
        {
            // Destroy previous circles
            foreach (Transform child in circlesContainer.transform)
            {
                GameObject.Destroy(child.gameObject);
            }

            GameObject circle = new GameObject("Circle");
            circle.transform.SetParent(circlesContainer.transform);
            circle.transform.position = targetTransform.position;

            LineRenderer lineRenderer = circle.AddComponent<LineRenderer>();
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.red;
            lineRenderer.startWidth = 0.05f;
            lineRenderer.endWidth = 0.05f;

            float radius = touchThreshold;
            int segments = 100;
            lineRenderer.positionCount = segments + 1;

            for (int j = 0; j <= segments; j++)
            {
                float angle = 360f / segments * j;
                float x = Mathf.Sin(angle * Mathf.Deg2Rad) * radius;
                float z = Mathf.Cos(angle * Mathf.Deg2Rad) * radius;
                lineRenderer.SetPosition(j, new Vector3(x, 0, z) + targetTransform.position);
            }
        }
    }
}

public class BoundingBoxTargetTouchReward
{
    private float touchReward;
    private GameObject boundingBox; // キューブまたは直方体のGameObject
    private Transform robotTransform;

    public void Initialize(Transform robotTransform, GameObject boundingBox, float touchReward)
    {
        this.robotTransform = robotTransform;
        this.boundingBox = boundingBox;
        this.touchReward = touchReward;
    }

    public float Calculate(ref bool trigEndEpisode, bool DEBUG = false)
    {
        // 四隅の座標を取得
        Vector3[] corners = GetBoundingBoxCorners(boundingBox);
        Vector3 robotPositionFlat = new Vector3(robotTransform.position.x, 0f, robotTransform.position.z);
        /*Debug.Log($"corners[0].x: {corners[0].x}, corners[0].z: {corners[0].z}, " +
            $"corners[1].x: {corners[1].x}, corners[1].z: {corners[1].z}," +
            $"corners[2].x: {corners[2].x}, corners[2].z: {corners[2].z}," +
            $"corners[3].x: {corners[3].x}, corners[3].z: {corners[3].z}," +
            $"robotPositionFlat.x, {robotPositionFlat.x}, robotPositionFlat.z: {robotPositionFlat.z}");*/

        if (IsPointInsidePolygon(corners, robotPositionFlat))
        {
            if (DEBUG)
            {
                Debug.Log("The robot reaches inside the bounding box area.");
            }
            trigEndEpisode = true;
            return touchReward;
        }
        else
        {
            return 0f;
        }
    }

    private Vector3[] GetBoundingBoxCorners(GameObject box)
    {
        Vector3 center = box.transform.position;
        Vector3 extents = box.transform.localScale / 2.0f;

        Vector3[] corners = new Vector3[4];
        /*corners[0] = box.transform.TransformPoint(center + new Vector3(extents.x, 0, extents.z));
        corners[1] = box.transform.TransformPoint(center + new Vector3(-extents.x, 0, extents.z));
        corners[2] = box.transform.TransformPoint(center + new Vector3(-extents.x, 0, -extents.z));
        corners[3] = box.transform.TransformPoint(center + new Vector3(extents.x, 0, -extents.z));*/
        corners[0] = center + new Vector3(extents.x, 0, extents.z);
        corners[1] = center + new Vector3(-extents.x, 0, extents.z);
        corners[2] = center + new Vector3(-extents.x, 0, -extents.z);
        corners[3] = center + new Vector3(extents.x, 0, -extents.z);


        return corners;
    }

    private bool IsPointInsidePolygon(Vector3[] corners, Vector3 point)
    {
        bool inside = false;
        for (int i = 0, j = corners.Length - 1; i < corners.Length; j = i++)
        {
            if (((corners[i].z > point.z) != (corners[j].z > point.z)) &&
                (point.x < (corners[j].x - corners[i].x) * (point.z - corners[i].z) / (corners[j].z - corners[i].z) + corners[i].x))
            {
                inside = !inside;
            }
        }
        return inside;
    }
}


public class LinearVelocityReward
{
    // この報酬関数は論文「Learning Quadrupedal Locomotion over Challengin Terrain」を参考にしている
    // 詳細は若干違っていて、0.6m/sの扱いなどが論文とは異なっている。
    // 論文では0.6を最高速度としているが、自分は最高速度としているわけではなくて、・・・・ただし、ここも正しく従わないとエビデンスの問題がでてきそうではあるので　修正した方がよいかも。

    private Transform robotTransform;
    private Vector2 lastPosition;
    private Vector2 currentPosition;
    private Vector2 moveVector;
    private bool drawArrows;
    private Renderer[] robotRenderers;
    private float maxVelocity;
    private float lastTime;
    private float currentTime;

    public void Initialize(Transform robotTransform, float maxVelocity, bool drawArrows)
    {
        this.robotTransform = robotTransform;
        this.maxVelocity = maxVelocity;
        this.lastTime = Time.time;
        this.lastPosition = new Vector2(robotTransform.position.x, robotTransform.position.z);
        this.robotRenderers = robotTransform.GetComponentsInChildren<Renderer>();
        this.drawArrows = drawArrows;
    }

    public float Calculate(JoyMsg joyMsg, Vector3Msg baseVelocityRos, bool DEBUG=false)
    {
        Vector2 controllerCommand = new Vector2(-joyMsg.axes[0], joyMsg.axes[1]);

        // Get the Y rotation angle of the robot in radians
        float rotationAngle = -robotTransform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        // Create a 2D rotation matrix
        Matrix4x4 rotationMatrix = new Matrix4x4();
        rotationMatrix.SetRow(0, new Vector4(Mathf.Cos(rotationAngle), -Mathf.Sin(rotationAngle), 0, 0));
        rotationMatrix.SetRow(1, new Vector4(Mathf.Sin(rotationAngle), Mathf.Cos(rotationAngle), 0, 0));
        rotationMatrix.SetRow(2, new Vector4(0, 0, 1, 0));
        rotationMatrix.SetRow(3, new Vector4(0, 0, 0, 1));

        // Rotate the controller command
        controllerCommand = rotationMatrix.MultiplyVector(controllerCommand);

        // ➀ 平面（Unityではx,z平面）ロボットの移動ベクトル（2次元）を計算
        /*currentPosition = new Vector2(robotTransform.position.x, robotTransform.position.z);
        currentTime = Time.time;
        moveVector = (currentPosition - lastPosition) / (currentTime - lastTime); 
        lastPosition = currentPosition;
        lastTime = currentTime;*/
        moveVector = new Vector2(baseVelocityRos.From<FLU>().x, baseVelocityRos.From<FLU>().y);

        if (drawArrows)
        {
            DrawArrows(controllerCommand, moveVector);
        }
        // maxVelocityで除算することで、ロボットによって最高速度が遅くても早くても、0~1の範囲のベクトルにして評価できるうようにしている
        // ➁ ➀で求めたベクトルとcontrollerCommandの内積を計算（= velocity_prとおく）
        float velocity_pr = Vector2.Dot(moveVector / maxVelocity, controllerCommand); // controllerCommandはたしか単位ベクトルなので、moveVectorをmaxVelocityで除算すればロボットの速度のスケールと同じになるはず。
        if (DEBUG)
        {
            Debug.Log($"Dot product of controller input and robot speed: {velocity_pr}");   //  DEBUGを有効にすると、正しく内積計算ができているか確認することができる
        }

        // ➂ velocity_pr < 0.6ならexp(-2.0(velocity_pr - 0.6)^2)、velocity_pr >= 0.6なら1.0、controllerCommandが0なら0.0を返す
        if (controllerCommand == Vector2.zero)
        {
            return 0.0f;
        }
        else if (velocity_pr < 0.6)
        {
            return Mathf.Exp(-2.0f * Mathf.Pow((velocity_pr - 0.6f), 2));
        }
        else
        {
            return 1.0f;
        }
    }
    public void DrawArrows(Vector2 controllerCommand, Vector2 moveVector)
    {
        // maxVelocityで除算しているのでロボットの速度が最大速度maxVelocityに到達したときに、だいたい長さが1になる（厳密には違うが（2乗の平方根なので））。
        // Convert 2D vectors to 3D for drawing
        Vector3 movingDirection3D = new Vector3(moveVector.x / maxVelocity, 0, moveVector.y / maxVelocity);   // 描画の関係上moveVectorのサイズを拡大している
        Vector3 controllerCommand3D = new Vector3(controllerCommand.x, 0, controllerCommand.y);

        // Debug.Log($"x: {controllerCommand.x}, y: {controllerCommand.y}");

        Vector3 objectCenter = robotRenderers.Aggregate(Vector3.zero, (acc, r) => acc + r.bounds.center) / robotRenderers.Length;

        // Draw arrows on the robot
        DrawArrow(objectCenter, movingDirection3D, Color.blue); // Green for actual movement
        DrawArrow(objectCenter, controllerCommand3D, Color.red);  // Red for controller command
    }

    public void DrawArrow(Vector3 start, Vector3 direction, Color color)
    {
        float arrowHeadLength = 0.25f;
        float arrowHeadAngle = 25.0f;

        Debug.DrawRay(start, direction, color);

        Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, arrowHeadAngle, 0) * new Vector3(0, 0, -1);
        Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, -arrowHeadAngle, 0) * new Vector3(0, 0, -1);

        Debug.DrawRay(start + direction, right * arrowHeadLength, color);
        Debug.DrawRay(start + direction, left * arrowHeadLength, color);
    }
}

public class AngularVelocityReward
{
    private Transform robotTransform;
    private float lastRotationAngle;
    private float lastTime;
    private Renderer[] robotRenderers;
    private bool drawArrows;

    public void Initialize(Transform robotTransform, bool drawArrows)
    {
        this.robotTransform = robotTransform;
        this.lastTime = Time.time;
        this.lastRotationAngle = robotTransform.rotation.eulerAngles.y;
        this.robotRenderers = robotTransform.GetComponentsInChildren<Renderer>();
        this.drawArrows = drawArrows;
    }

    public float Calculate(JoyMsg joyMsg, Vector3Msg angularVelocityRos, bool DEBUG = false)
    {
        float controllerCommandTargetAngularVelocity = joyMsg.axes[2];

        float omega_pr = (float)angularVelocityRos.z * controllerCommandTargetAngularVelocity;

        if (DEBUG)
        {
            Debug.Log("AngularVelocityReward");
            Debug.Log($"JoyMsg.axes[2]: {controllerCommandTargetAngularVelocity}");
            Debug.Log($"Base Angular Velocity: {omega_pr}");
        }
        
        if (omega_pr < 0.6)
        {
            return Mathf.Exp(-1.5f * Mathf.Pow((omega_pr - 0.6f), 2));
        }
        else
        {
            return 1.0f;
        }
    }

/*    public float CalculateYAngularVelocity()  // Y軸周りの角速度(ROSにおけるZ軸周りの角速度)
    {
        // Get the current rotation angle
        float currentRotationAngle = robotTransform.rotation.eulerAngles.y;

        // Get the current time
        float currentTime = Time.time;

        // Calculate the difference in angles
        float angleDifference = currentRotationAngle - lastRotationAngle;

        // Handle the case where the angle crosses from 359 to 0 degrees, or vice versa
        if (angleDifference > 180.0f)
        {
            angleDifference -= 360.0f;
        }
        else if (angleDifference < -180.0f)
        {
            angleDifference += 360.0f;
        }

        // Calculate the difference in time
        float timeDifference = currentTime - lastTime;

        // Calculate the angular velocity
        float angularVelocity = angleDifference / timeDifference;

        // Save the current rotation angle and time for the next frame
        lastRotationAngle = currentRotationAngle;
        lastTime = currentTime;

        // Return the calculated angular velocity
        return angularVelocity;
    }*/

}

public class BaseMotionReward
{
    private Transform robotTransform;
    private Vector3 lastPosition;
    private Vector3 lastRotation;
    private Vector3 currentPosition;
    private Vector2 moveVector;
    private bool drawArrows;
    private Renderer[] robotRenderers;
    private float maxVelocity;
    private float lastTime;
    private float currentTime;

    public void Initialize(Transform robotTransform, float maxVelocity, bool drawArrows)
    {
        this.robotTransform = robotTransform;
        this.maxVelocity = maxVelocity;
        this.lastTime = Time.time;
        this.lastPosition = robotTransform.position;
        this.lastRotation = robotTransform.rotation.eulerAngles;
        this.robotRenderers = robotTransform.GetComponentsInChildren<Renderer>();
        this.drawArrows = drawArrows;

        if (Mathf.Approximately(maxVelocity, 0.0f)) // maxVelocityが0に近い（または0）場合
        {
            this.maxVelocity = 0.01f;   // Calculateでエラーが出ないように、小さい値を設定している。この値に意味はない。
            throw new ArgumentException("maxVelocity should not be zero or close to zero.");
        }
    }

    public float Calculate(JoyMsg joyMsg, Vector3Msg baseVelocityRos, Vector3Msg baseAngularVelocityRos, bool DEBUG = false)
    {
        Vector2 controllerCommand = new Vector2(-joyMsg.axes[0], joyMsg.axes[1]);


        // Get the Y rotation angle of the robot in radians
        float rotationAngle = -robotTransform.rotation.eulerAngles.y * Mathf.Deg2Rad;

        // Create a 2D rotation matrix
        Matrix4x4 rotationMatrix = new Matrix4x4();
        rotationMatrix.SetRow(0, new Vector4(Mathf.Cos(rotationAngle), -Mathf.Sin(rotationAngle), 0, 0));
        rotationMatrix.SetRow(1, new Vector4(Mathf.Sin(rotationAngle), Mathf.Cos(rotationAngle), 0, 0));
        rotationMatrix.SetRow(2, new Vector4(0, 0, 1, 0));
        rotationMatrix.SetRow(3, new Vector4(0, 0, 0, 1));

        // Rotate the controller command
        controllerCommand = rotationMatrix.MultiplyVector(controllerCommand);

        // ➀ 平面（Unityではx,z平面）ロボットの移動ベクトル（2次元）を計算
        //Vector3 velocityVector3 = CalculateVelocity();
        // moveVector = new Vector2(velocityVector3.x, velocityVector3.z);
        moveVector = new Vector2(baseVelocityRos.From<FLU>().x, baseVelocityRos.From<FLU>().z);

        // Vector3 angularVelocityVector3 = CalculateAngularVelocity();
        Vector3 angularVelocityVector3 = baseAngularVelocityRos.From<FLU>();

        // maxVelocityで除算することで、ロボットによって最高速度が遅くても早くても、0~1の範囲のベクトルにして評価できるうようにしている
        // ➁ ➀で求めたベクトルとcontrollerCommandの内積を計算（= velocity_prとおく）
        float velocity_pr = Vector2.Dot(moveVector / maxVelocity, controllerCommand); // controllerCommandはたしか単位ベクトルなので、moveVectorをmaxVelocityで除算すればロボットの速度のスケールと同じになるはず。
        
        if (DEBUG)
        {
            Debug.Log($"Dot product of controller input and robot speed: {velocity_pr}");   //  DEBUGを有効にすると、正しく内積計算ができているか確認することができる
        }

        // ➂ velocity_pr < 0.6ならexp(-2.0(velocity_pr - 0.6)^2)、velocity_pr >= 0.6なら1.0、controllerCommandが0なら0.0を返す
        if (controllerCommand == Vector2.zero)  // ストップコマンド
        {
            float v0 = baseVelocityRos.From<FLU>().magnitude;
            float reward_b = Mathf.Exp(-1.5f * Mathf.Pow(v0, 2)) + Mathf.Exp(-1.5f * new Vector2(angularVelocityVector3.x, angularVelocityVector3.z).magnitude);
            return reward_b;
        }
        else
        {
            float v0 = new Vector2(
                baseVelocityRos.From<FLU>().x / maxVelocity - velocity_pr * controllerCommand.x,
                baseVelocityRos.From<FLU>().y / maxVelocity - velocity_pr * controllerCommand.y
            ).magnitude;
            float reward_b = Mathf.Exp(-1.5f * Mathf.Pow(v0, 2)) + Mathf.Exp(-1.5f * new Vector2(angularVelocityVector3.x, angularVelocityVector3.z).magnitude);
            return reward_b;
        }
    }

  /*  public Vector3 CalculateVelocity()
    {
        // 現在の位置を取得
        Vector3 currentPosition = robotTransform.position;

        // 現在の時間を取得
        float currentTime = Time.time;

        // 前回のフレームとの時間差を計算
        float timeDifference = currentTime - lastTime;

        // ゼロ除算を回避
        Vector3 velocity = Vector3.zero;
        if (timeDifference > 0)
        {
            Vector3 positionDifference = currentPosition - lastPosition;
            velocity = positionDifference / timeDifference;
        }

        Debug.Log($"v: {velocity.x}, {velocity.y}, {velocity.z}");

        // 現在の位置と時間を保存
        lastPosition = currentPosition;
        lastTime = currentTime;

        // 計算した速度を返す
        return velocity;
    }

    public Vector3 CalculateAngularVelocity()
    {
        Vector3 currentRotation = robotTransform.rotation.eulerAngles;
        float currentTime = Time.time;
        float timeDifference = currentTime - lastTime;

        // 前回の回転角と現在の回転角の差を計算
        Vector3 angleDifference = currentRotation - lastRotation;

        // 角度の差が180度を超える場合は、逆方向の差を計算する
        for (int i = 0; i < 3; ++i)
        {
            if (angleDifference[i] > 180.0f)
            {
                angleDifference[i] -= 360.0f;
            }
            else if (angleDifference[i] < -180.0f)
            {
                angleDifference[i] += 360.0f;
            }
        }

        // 前回の時間が0の場合や、時間の差が非常に小さい場合は、角速度を計算できないので(0,0,0)を返す
        if (lastTime == 0 || timeDifference < 0.0001f)
        {
            Debug.Log(timeDifference);
            return Vector3.zero;
        }

        // 角速度を計算
        Vector3 angularVelocity = angleDifference / timeDifference;

        Debug.Log($"av: {angularVelocity.x}, {angularVelocity.y}, {angularVelocity.z}");

        lastRotation = currentRotation;
        lastTime = currentTime;

        return angularVelocity;
    }*/
}

public class FootClearanceReward
{

}
public class BodyCollisionReward
{

}
public class TargetSmoothnessReward
{

}
public class TorqueReward
{

}

// 胴体の角度が決められた範囲外の値となった場合に終了する
public class FallDownReward
{
    private Transform robotTransform;
    private Vector3 robotRotation;
    private float fallDownReward;  // 転んだ時に与えられる報酬
    private float notFallDownReward; // 転倒していないときに貰える報酬
    private bool exitWhenFallDown; //　転んだら終了するかどうか
    private float xMin;
    private float xMax;
    private float yMin;
    private float yMax;
    private float zMin;
    private float zMax;

    public void Initialize(Transform robotTransform, float fallDownReward, float notFallDownReward, bool exitWhenFallDown, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
    {
        this.robotTransform = robotTransform;
        this.fallDownReward = fallDownReward;
        this.notFallDownReward = notFallDownReward;
        this.exitWhenFallDown = exitWhenFallDown;
        this.xMin = xMin;
        this.xMax = xMax;
        this.yMin = yMin;
        this.yMax = yMax;
        this.zMin = zMin;
        this.zMax = zMax;
    }

    public float Calculate(ref bool trigEndEpisode, bool DEBUG = false)
    {
        robotRotation = robotTransform.localRotation.eulerAngles;
        if (robotRotation.x > 180)
        {
            robotRotation.x = robotRotation.x - 360;
        }
        if (robotRotation.y > 180)
        {
            robotRotation.y = robotRotation.y - 360;
        }
        if (robotRotation.z > 180)
        {
            robotRotation.z = robotRotation.z - 360;
        }

        if (DEBUG)
        {
            Debug.Log($"robotTransform.localRotation.eulerAngles: ({robotRotation.x}, {robotRotation.y}, {robotRotation.z})");
        }

        if (robotRotation.x < xMin || robotRotation.x > xMax)
        {
            if (exitWhenFallDown)
            {
                trigEndEpisode = true;
            }
            return fallDownReward;
        }
        if (robotRotation.y < yMin || robotRotation.y > yMax)
        {
            if (exitWhenFallDown)
            {
                trigEndEpisode = true;
            }
            return fallDownReward;
        }
        if (robotRotation.z < zMin || robotRotation.z > zMax)
        {
            if (exitWhenFallDown)
            {
                trigEndEpisode = true;
            }
            return fallDownReward;
        }
        return notFallDownReward;
    }
}

/*// ロボットの方向を

Vector3 bodyDirection = frontPoint.position - rearPoint.position;


// ジョイスティックに関連した報酬関数は複数考えられるので、クラスを作成し

public class Joystick
{
    public Transform frontPoint;
    public Transform rearPoint;
    public Transform robotTransform;
    public Vector3 previousRobotPosition;

    public void Initialize(Transform frontPoint, Transform rearPoint, Transform robotTransform)
    {
        this.frontPoint = frontPoint;
        this.rearPoint = rearPoint;
        this.robotTransform = robotTransform;
        previousRobotPosition = robotTransform.position;
    }

    public float // 進行方向の類似性

    public float //
}*/