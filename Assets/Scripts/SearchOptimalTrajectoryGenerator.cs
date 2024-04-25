using System;
using System.IO;
using UnityEngine;

public class SearchOptimalTrajectoryGenerator : MonoBehaviour
{
    private string directoryPath;
    private string currentCsvFilePath; // ���݂̃��O�pCSV�t�@�C���̃p�X��ێ�
    private QuadrupedTrajectoryGenerator quadrupedTrajectoryGenerator;
    
    // Start is called before the first frame update
    void Start()
    {
        directoryPath = Path.Combine(Application.dataPath, "Csv");
    }

    // Update is called once per frame
    void Update()
    {
    }

    public void LogTgInfo(int nEpisode,
                          int currentFrontPitch,
                          int currentRearPitch,
                          float currentFrontX,
                          float currentFrontZ,
                          float currentRearX,
                          float currentRearZ,
                          ref int nGoals,
                          int numberOfEpisodesPerConfig)
    {
        // nEpisode��1���ڂ�numberOfEpisodesPerConfig-1���ǂ������m�F
        if (nEpisode % numberOfEpisodesPerConfig == numberOfEpisodesPerConfig-1)
        {
            // Csv�t�H���_�����݂��Ȃ��ꍇ�͍쐬
            if (!Directory.Exists(directoryPath))
            {
                Directory.CreateDirectory(directoryPath);
            }

            // currentCsvFilePath��null�܂��͋�̏ꍇ�A�V����CSV�t�@�C�����쐬
            if (string.IsNullOrEmpty(currentCsvFilePath))
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                currentCsvFilePath = Path.Combine(directoryPath, $"log_{timestamp}.csv");

                // �J����������������
                string header = "FrontPitch,RearPitch,FrontX,FrontZ,RearX,RearZ,nGoals\n";
                File.WriteAllText(currentCsvFilePath, header);
            }

            // CSV�`���Ńf�[�^��ۑ�
            string csvData = $"{currentFrontPitch},{currentRearPitch},{currentFrontX},{currentFrontZ},{currentRearX},{currentRearZ},{nGoals}\n";
            File.AppendAllText(currentCsvFilePath, csvData);

            nGoals = 0; // �S�[���񐔂�0�ɂ��� 
        }
    }
}