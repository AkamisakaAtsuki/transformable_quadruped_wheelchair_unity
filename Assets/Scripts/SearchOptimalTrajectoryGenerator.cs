using System;
using System.IO;
using UnityEngine;

public class SearchOptimalTrajectoryGenerator : MonoBehaviour
{
    private string directoryPath;
    private string currentCsvFilePath; // 現在のログ用CSVファイルのパスを保持
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
        // nEpisodeの1桁目がnumberOfEpisodesPerConfig-1かどうかを確認
        if (nEpisode % numberOfEpisodesPerConfig == numberOfEpisodesPerConfig-1)
        {
            // Csvフォルダが存在しない場合は作成
            if (!Directory.Exists(directoryPath))
            {
                Directory.CreateDirectory(directoryPath);
            }

            // currentCsvFilePathがnullまたは空の場合、新しいCSVファイルを作成
            if (string.IsNullOrEmpty(currentCsvFilePath))
            {
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                currentCsvFilePath = Path.Combine(directoryPath, $"log_{timestamp}.csv");

                // カラム名を書き込む
                string header = "FrontPitch,RearPitch,FrontX,FrontZ,RearX,RearZ,nGoals\n";
                File.WriteAllText(currentCsvFilePath, header);
            }

            // CSV形式でデータを保存
            string csvData = $"{currentFrontPitch},{currentRearPitch},{currentFrontX},{currentFrontZ},{currentRearX},{currentRearZ},{nGoals}\n";
            File.AppendAllText(currentCsvFilePath, csvData);

            nGoals = 0; // ゴール回数は0にする 
        }
    }
}