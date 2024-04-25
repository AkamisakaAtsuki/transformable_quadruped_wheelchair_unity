using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrajectoryVisualizer : MonoBehaviour
{
    public ArticulationBody rootArticulation;
    public int maxTargetPoints = 5; // �ێ�����ő吔
    // public float lineWidth = 0.1f;
    public List<LineRenderer> lineRenderers;

    private List<List<Vector3>> targetPoints = new List<List<Vector3>>();

    /*public void UpdateTrajectoryVisualizer(List<Vector3> targetPoints)
    {
        for (int i = 0; i < targetPoints.Count; i++)
        {
            LineRenderer lineRenderer = lineRenderers[i];
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, targetPoints[i]);
            lineRenderer.SetPosition(1, targetPoints[(i + 1) % targetPoints.Count]);
        }
    }*/

    // Update LineRenderer positions

public void UpdateTrajectoryVisualizer(List<Vector3> newTargetPoints)
    {
        // �V����targetPoints��ێ�
        targetPoints.Add(newTargetPoints);

        // �ő吔�𒴂����ꍇ�A�Â��f�[�^���폜
        if (targetPoints.Count > maxTargetPoints)
        {
            int removeCount = targetPoints.Count - maxTargetPoints;
            targetPoints.RemoveRange(0, removeCount);
        }

        /*for (int i = 0; i < targetPoints.Count; i++)
        {
            LineRenderer lineRenderer = GetLineRenderer(i);
            lineRenderer.positionCount = 2;
            lineRenderer.SetPosition(0, targetPoints[i]);
            lineRenderer.SetPosition(1, targetPoints[(i + 1) % targetPoints.Count]);
        }*/
        for (int i = 0; i < newTargetPoints.Count; i++)
        {
            LineRenderer lineRenderer = lineRenderers[i];
            lineRenderer.positionCount = targetPoints.Count;

            for (int j = 0; j < targetPoints.Count; j++)
            {
                lineRenderer.SetPosition(j, rootArticulation.transform.TransformPoint(targetPoints[j][i]));
            }
        }
    }

    /*private LineRenderer GetLineRenderer(int index)
    {
        if (index < lineRenderers.Count)
        {
            return lineRenderers[index];
        }
        else
        {
            // LineRenderer������Ȃ��ꍇ�͐V���ɍ쐬
            GameObject newLineObject = new GameObject("LineRenderer");
            newLineObject.transform.SetParent(transform);

            LineRenderer newLineRenderer = newLineObject.AddComponent<LineRenderer>();
            lineRenderers.Add(newLineRenderer);

            return newLineRenderer;
        }
    }*/
}