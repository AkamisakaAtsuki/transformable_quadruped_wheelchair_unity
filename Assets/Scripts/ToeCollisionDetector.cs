using UnityEngine;

public class ToeCollisionDetector : MonoBehaviour
{
    public Vector3 toeForce { get; private set; }
    public GameObject attachedPart;
    public MeshCollider childCollider;

    private void Start()
    {
        childCollider = FindMeshColliderInChildren(transform);
    }

    private MeshCollider FindMeshColliderInChildren(Transform parentTransform)
    {
        MeshCollider foundCollider = null;

        // Check if this GameObject has a MeshCollider
        MeshCollider mc = parentTransform.GetComponent<MeshCollider>();
        if (mc != null && parentTransform.gameObject != attachedPart)
        {
            foundCollider = mc;
        }

        // Check all children recursively
        foreach (Transform child in parentTransform)
        {
            MeshCollider childCollider = FindMeshColliderInChildren(child);
            if (childCollider != null)
            {
                foundCollider = childCollider;
            }
        }

        return foundCollider;
    }
    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject != attachedPart)
        {
            // 以前のコードをここに配置
            Vector3 totalForce = Vector3.zero;
            foreach (ContactPoint contact in collision.contacts)
            {
                float mass = 0.0f;
                if (collision.rigidbody != null)
                {
                    mass = collision.rigidbody.mass;
                }
                else if (collision.collider.attachedRigidbody != null)
                {
                    mass = collision.collider.attachedRigidbody.mass;
                }
                else if (collision.collider.attachedArticulationBody != null)
                {
                    mass = collision.collider.attachedArticulationBody.mass;
                }

                Vector3 forceDirection = contact.normal;
                float forceMagnitude = -1 * mass * collision.relativeVelocity.magnitude / Time.fixedDeltaTime;
                Vector3 contactForce = forceDirection * forceMagnitude;
                totalForce += contactForce;
            }
            toeForce = totalForce;
            
            // Debug.Log("Toe Force: " + toeForce);
        }
    }
}