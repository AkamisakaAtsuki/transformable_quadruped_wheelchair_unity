using UnityEngine;

public class TouchSensor : MonoBehaviour
{
    public bool raw = false;


    private void OnCollisionEnter(Collision collision)
    {
        raw = true;
    }

    // �Z���T�̒l���擾�����璀�ꃊ�Z�b�g
    public void resetData()
    {
        raw = false;
    }
}
