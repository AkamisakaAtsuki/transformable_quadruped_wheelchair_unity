using UnityEngine;
using UnityEngine.AI;

// ���̃X�N���v�g�ł́A�p�X�̌v�Z�݂̂��s���A���{�b�g���̂̓��쐧��͍s��Ȃ��B�i�s�����̃x�N�g���i���[���h���W�n�j���擾���邱�Ƃ��ł���B

public class RobotNavigation : MonoBehaviour
{
	[SerializeField]
	private Transform m_Target;
	[SerializeField]
	private GameObject navMeshObject;  // ���{�b�g��base�I�u�W�F�N�g��navMesh���A�^�b�`����ƁA���{�b�g�̈ʒu�����쒆�ɂ��ꂽ�肷��\�������邽�߁AnavMesh�A�^�b�`�p�̃I�u�W�F�N�g��base�����N�����ɒu���A����������Ƀh���b�O�A���h�h���b�v
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
			// LineRenderer�̊�{�ݒ�
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