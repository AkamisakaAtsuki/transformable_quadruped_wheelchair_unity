using UnityEngine;
using UnityEngine.AI;

// このスクリプトでは、パスの計算のみを行い、ロボット自体の動作制御は行わない。進行方向のベクトル（ワールド座標系）を取得することができる。

public class RobotNavigation : MonoBehaviour
{
	[SerializeField]
	private Transform m_Target;
	[SerializeField]
	private GameObject navMeshObject;  // ロボットのbaseオブジェクトにnavMeshをアタッチすると、ロボットの位置が動作中にずれたりする可能性があるため、navMeshアタッチ用のオブジェクトをbaseリンク直下に置き、それをここにドラッグアンドドロップ
	public bool drawPath;
	private Transform robotRootTransform;
	public Vector3 direction;

	private NavMeshAgent m_Agent;
	private LineRenderer lineRenderer;

	void Start()
	{
		robotRootTransform = navMeshObject.GetComponent<Transform>();
		m_Agent = navMeshObject.GetComponent<NavMeshAgent>();
		m_Agent.updatePosition = false;
		m_Agent.updateRotation = false;
		// m_Agent.isStopped = true;
		m_Agent.nextPosition = robotRootTransform.position;

		direction = new Vector3(0, 0, 0);

		if (drawPath)
        {
			// LineRendererの基本設定
			lineRenderer = gameObject.AddComponent<LineRenderer>();
			lineRenderer.widthMultiplier = 0.2f;
			lineRenderer.material = new Material(Shader.Find("Standard"));
			lineRenderer.startColor = Color.blue;
			lineRenderer.endColor = Color.red;
        }

		//m_Agent.enabled = false;
	}

	void DrawPath(NavMeshPath path)
	{
		if (path.corners.Length < 2)  // If the path has 1 or no corners, there is no need
			return;

		lineRenderer.positionCount = path.corners.Length;

		for (int i = 0; i < path.corners.Length; i++)
		{
			lineRenderer.SetPosition(i, path.corners[i]);
		}
	}

	void FixedUpdate()
	{
		
		//m_Agent.enabled = true;
		m_Agent.SetDestination(m_Target.position);
		direction = m_Agent.desiredVelocity.normalized;
		m_Agent.nextPosition = robotRootTransform.position;
		if (drawPath)
        {
			DrawPath(m_Agent.path);
        }
		//m_Agent.enabled = false;
	}
}