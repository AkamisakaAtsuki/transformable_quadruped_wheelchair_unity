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
        // ���[�g�Ǝq��ArticulationBody�����X�g�ɒǉ�
        articulationBodies = new List<ArticulationBody>(rootArticulation.GetComponentsInChildren<ArticulationBody>());

        // �eArticulationBody�̏�����Ԃ�ۑ�
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

    // �����GameObject�̖��O�ƐV����Transform�������Ƃ��āA
    // �Ή�����ArticulationBody�̏�����Ԃ�ύX���郁�\�b�h
    public void UpdateInitialTransform(string gameObjectName, Transform newTransform)
    {
        // articulationBodies���X�g���������āA���O����v����ArticulationBody��������
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // �ʒu�Ɖ�]��V����Transform�̒l�ɍX�V
                initialPositions[i] = newTransform.localPosition;
                initialRotations[i] = newTransform.localRotation;
                break;
            }
        }
    }

    // �����GameObject�̖��O�Ɋ�Â��ĉ�]�݂̂��X�V���郁�\�b�h
    public void UpdateInitialRotation(string gameObjectName, Quaternion newRotation)
    {
        // articulationBodies���X�g���������āA���O����v����ArticulationBody��������
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // ��]��V����Quaternion�̒l�ɍX�V
                initialRotations[i] = newRotation;
                break;
            }
        }
    }

    // �����GameObject�̖��O�Ɋ�Â��ĉ�]�݂̂��X�V���郁�\�b�h
    public void UpdateInitialPosition(string gameObjectName, Vector3 newPosition)
    {
        // articulationBodies���X�g���������āA���O����v����ArticulationBody��������
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // ��]��V����Quaternion�̒l�ɍX�V
                initialPositions[i] = newPosition;
                break;
            }
        }
    }

    public Quaternion GetRotation(string gameObjectName)
    {
        // articulationBodies���X�g���������āA���O����v����ArticulationBody��������
        for (int i = 0; i < articulationBodies.Count; i++)
        {
            if (articulationBodies[i].gameObject.name == gameObjectName)
            {
                // ��v����ArticulationBody�̏�����]��Ԃ�
                return initialRotations[i];
            }
        }
        // ��v������̂�������Ȃ������ꍇ�ɂ�Quaternion.identity��Ԃ�
        return Quaternion.identity;
    }
}