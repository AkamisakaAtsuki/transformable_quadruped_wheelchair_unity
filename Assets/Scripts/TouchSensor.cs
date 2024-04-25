using UnityEngine;

public class TouchSensor : MonoBehaviour
{
    public bool raw = false;


    private void OnCollisionEnter(Collision collision)
    {
        raw = true;
    }

    // センサの値を取得したら逐一リセット
    public void resetData()
    {
        raw = false;
    }
}
