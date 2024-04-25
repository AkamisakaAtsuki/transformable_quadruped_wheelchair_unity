using UnityEngine;
using Unity.MLAgents;

public class GroundTouchDetector : MonoBehaviour
{
    public string touchGroundTag;
    public bool hasTouchedGround = false;


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchGroundTag)
        {
            Debug.Log("Ground Touch Detected!");
            hasTouchedGround = true;
        }
    }
}
