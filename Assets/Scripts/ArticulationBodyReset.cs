using System.Collections.Generic;
using UnityEngine;

public class ArticulationBodyReset
{
    private List<Transform> articulationTransforms;
    private List<Vector3> initialPositions;
    private List<Quaternion> initialRotations;
    private List<ArticulationBody> articulationBodies;

    public void InitializeArticulationBodies(ArticulationBody rootArticulation)
    {
        // ルートと子のArticulationBodyをリストに追加
        articulationBodies = new List<ArticulationBody>(rootArticulation.GetComponentsInChildren<ArticulationBody>());

        // 各ArticulationBodyの初期状態を保存
        articulationTransforms = new List<Transform>();
        initialPositions = new List<Vector3>();
        initialRotations = new List<Quaternion>();
        foreach (ArticulationBody articulationBody in articulationBodies)
        {
            articulationTransforms.Add(articulationBody.transform);
            initialPositions.Add(articulationBody.transform.localPosition);
            initialRotations.Add(articulationBody.transform.localRotation);
        }
    }

    private void SetArticulationBodiesEnabledState(bool state)
    {
        foreach (ArticulationBody articulationBody in articulationBodies)
        {
            articulationBody.enabled = state;
        }
    }

    private void ResetPositionsAndRotations()
    {
        for (int i = 0; i < articulationTransforms.Count; i++)
        {
            articulationTransforms[i].localPosition = initialPositions[i];
            articulationTransforms[i].localRotation = initialRotations[i];
        }
    }

    private void ResetVelocityAndAngularVelocity()
    {
        foreach (ArticulationBody articulationBody in articulationBodies)
        {
            articulationBody.velocity = Vector3.zero;
            articulationBody.angularVelocity = Vector3.zero;
        }
    }

    public void ResetArticulationBodies(GameObject rootArticulationObject, Vector3 offset)
    {
        SetArticulationBodiesEnabledState(false);
        ResetPositionsAndRotations();
        ResetVelocityAndAngularVelocity();
        rootArticulationObject.transform.localPosition = offset;
        SetArticulationBodiesEnabledState(true);
    }

    // 特定のGameObjectの名前と新しいTransformを引数として、
    // 対応するArticulationBodyの初期状態を変更するメソッド
    public void UpdateInitialTransform(string gameObjectName, Transform newTransform)
    {
        // articulationBodiesリストを検索して、名前が一致するArticulationBodyを見つける
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // 位置と回転を新しいTransformの値に更新
                initialPositions[i] = newTransform.localPosition;
                initialRotations[i] = newTransform.localRotation;
                break;
            }
        }
    }

    // 特定のGameObjectの名前に基づいて回転のみを更新するメソッド
    public void UpdateInitialRotation(string gameObjectName, Quaternion newRotation)
    {
        // articulationBodiesリストを検索して、名前が一致するArticulationBodyを見つける
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // 回転を新しいQuaternionの値に更新
                initialRotations[i] = newRotation;
                break;
            }
        }
    }

    // 特定のGameObjectの名前に基づいて回転のみを更新するメソッド
    public void UpdateInitialPosition(string gameObjectName, Vector3 newPosition)
    {
        // articulationBodiesリストを検索して、名前が一致するArticulationBodyを見つける
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // 回転を新しいQuaternionの値に更新
                initialPositions[i] = newPosition;
                break;
            }
        }
    }

    public Quaternion GetRotation(string gameObjectName)
    {
        // articulationBodiesリストを検索して、名前が一致するArticulationBodyを見つける
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // 一致するArticulationBodyの初期回転を返す
                return initialRotations[i];
            }
        }
        // 一致するものが見つからなかった場合にはQuaternion.identityを返す
        return Quaternion.identity;
    }
}