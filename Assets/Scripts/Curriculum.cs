using System;
using System.IO;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class Curriculum
{
    private Dictionary<string, object> _properties = new Dictionary<string, object>();

    public void SetProperty(string key, object value)
    {
        _properties[key] = value;
    }

    public object GetProperty(string key)
    {
        _properties.TryGetValue(key, out var value);
        return value;
    }

    // Retrieve all properties as a dictionary
    public Dictionary<string, object> GetAllProperties()
    {
        return _properties;
    }
}



public class LinearStaircaseClimbingCurriculum
{
    public bool goalFlag = false;
    public List<Curriculum> Curriculums { get; set; } = new List<Curriculum>();
    public TextAsset curriculumFile; // UnityエディタからアタッチするCSV形式のカリキュラムファイル
    private int currentPhaseIndex = 0; // 現在のカリキュラムフェーズのインデックス
    public int currentGoalsAchieved = 0; // 現在のフェーズで達成されたゴール数
    private List<bool> goalFlagHistory = new List<bool>(); // goalFlagの履歴を保存するリストを追加

    private string csvFileName = null; // CSVファイル名を保持する変数

    /*   public void InitCurriculum(int startCuriculumNumber = 0)
       {
           currentPhaseIndex = startCuriculumNumber;
           string[] data = curriculumFile.text.Split(new char[] { '\n' });
           string[] headers = data[0].Split(','); // ヘッダー行を解析
           for (int i = 1; i < data.Length - 1; i++)
           {
               string[] row = data[i].Split(',');
               var curriculum = new Curriculum();
               for (int j = 0; j < headers.Length; j++)
               {
                   var property = typeof(Curriculum).GetProperty(headers[j]);
                   if (property != null)
                   {
                       var value = Convert.ChangeType(row[j], property.PropertyType);
                       property.SetValue(curriculum, value);
                   }
               }
               Curriculums.Add(curriculum);
           }
           Debug.Log(textOfCurrentCurriculum());
       }
   */
    public void InitCurriculum(int startCuriculumNumber = 0)
    {
        currentPhaseIndex = startCuriculumNumber;
        string[] data = curriculumFile.text.Split(new char[] { '\n' }, StringSplitOptions.RemoveEmptyEntries);
        string[] headers = data[0].Split(',');
        for (int i = 1; i < data.Length; i++)
        {
            string[] row = data[i].Split(',');
            var curriculum = new Curriculum();
            for (int j = 0; j < headers.Length; j++)
            {
                string header = headers[j].Trim(); // カラム名
                object value = row[j].Trim(); // 値をトリムして余分なスペースを削除
                if (int.TryParse(value.ToString(), out int intValue))
                {
                    value = intValue;
                }
                else if (float.TryParse(value.ToString(), out float floatValue))
                {
                    value = floatValue;
                }
                curriculum.SetProperty(header, value);
            }
            Curriculums.Add(curriculum);
        }
    }

    /*    public void InitCurriculum(int startCuriculumNumber=0)  // デフォルトは0番からカリキュラムをスタート。途中でエディタが落ちるなどがあった場合は途中からスタートできるようにしている
        {
            currentPhaseIndex = startCuriculumNumber;
            string[] data = curriculumFile.text.Split(new char[] { '\n' });
            for (int i = 1; i < data.Length-1; i++)
            {
                string[] row = data[i].Split(',');
                Curriculum curriculum = new Curriculum
                {
                    Phase = int.Parse(row[0]),
                    GoalCount = int.Parse(row[1]),
                    RiserRatio = float.Parse(row[2]),
                    WeightRatio = float.Parse(row[3]),
                    GoalDistanceFactor = float.Parse(row[4])
                };
                Curriculums.Add(curriculum);
            }
            Debug.Log(textOfCurrentCurriculum());
        }
    */
    // カリキュラムを返す。全てのカリキュラムが終了したら、Unityエディタを自動で停止させる。
    // これまでの学習ステップ数を受け取り、カリキュラムが変化した際のCSVの値を記録する
    /*public (int Phase, float RiserRatio, float WeightRatio, float GoalDistanceFactor) MonitorCurriculum(bool goalFlag, int totalStepCount)
    {
        // goalFlagの履歴を更新
        goalFlagHistory.Add(goalFlag);
        if (goalFlagHistory.Count > 10)
        {
            goalFlagHistory.RemoveAt(0);  // 最新の10個のみを保持するため、古いものを削除
        }
        int achievedGoalsInHistory = goalFlagHistory.FindAll(g => g).Count;  // Python APIから制御するときの癖に対応する為に追加したコード（①）

        if (goalFlag)
        {
            currentGoalsAchieved++;
            *//*if (currentGoalsAchieved == Curriculums[currentPhaseIndex].GoalCount)*//*
            if (achievedGoalsInHistory == Curriculums[currentPhaseIndex].GoalCount)  // Python APIから制御するときの癖に対応する為に追加したコード（②）
            {
                // ここでCSVにcurrentPhaseIndexとステップ数を書き込む
                WriteToCSV(currentPhaseIndex, totalStepCount);

                currentPhaseIndex++;
                currentGoalsAchieved = 0;
                goalFlagHistory = new List<bool>();
                if (currentPhaseIndex >= Curriculums.Count)
                {
                    #if UNITY_EDITOR
                    EditorApplication.isPlaying = false;
                    #endif
                    return (-1, 0f, 0f, 0f); // エディタが停止した後の返り値
                }
                Debug.Log(textOfCurrentCurriculum());
            }
        }
        else
        {
            currentGoalsAchieved = 0;
        }

        return (Curriculums[currentPhaseIndex].Phase, Curriculums[currentPhaseIndex].RiserRatio, Curriculums[currentPhaseIndex].WeightRatio, Curriculums[currentPhaseIndex].GoalDistanceFactor);
    }
*/

    public Dictionary<string, object> MonitorCurriculum(bool goalFlag, int totalStepCount)
    {
        // goalFlagの履歴を更新
        goalFlagHistory.Add(goalFlag);
        if (goalFlagHistory.Count > 10)
        {
            goalFlagHistory.RemoveAt(0);  // 最新の10個のみを保持するため、古いものを削除
        }
        int achievedGoalsInHistory = goalFlagHistory.FindAll(g => g).Count;
        Debug.Log($"achievedGoalsInHistory: {achievedGoalsInHistory}");

        if (goalFlag)
        {
            currentGoalsAchieved++;
            // Within MonitorCurriculum and any other method that accesses properties
            int goalCount = (int)Curriculums[currentPhaseIndex].GetProperty("GoalCount");

            if (achievedGoalsInHistory == goalCount)
            {
                // ここでCSVにcurrentPhaseIndexとステップ数を書き込む
                WriteToCSV(currentPhaseIndex, totalStepCount);

                currentPhaseIndex++;
                currentGoalsAchieved = 0;
                goalFlagHistory = new List<bool>();
                if (currentPhaseIndex >= Curriculums.Count)
                {
                    #if UNITY_EDITOR
                    EditorApplication.isPlaying = false;
                    #endif
                    // エディタが停止した後のデフォルト値を動的に生成
                    return GenerateDefaultCurriculumValues();
                }
            }
        }
        else
        {
            currentGoalsAchieved = 0;
        }

        // 現在のカリキュラムフェーズのデータを辞書に変換する
        return GetCurrentCurriculumData();
    }

    private Dictionary<string, object> GetCurrentCurriculumData()
    {
        var curriculumData = new Dictionary<string, object>();
        var currentCurriculum = Curriculums[currentPhaseIndex].GetAllProperties();
        foreach (var prop in currentCurriculum)
        {
            curriculumData.Add(prop.Key, prop.Value);
        }
        return curriculumData;
    }

    private Dictionary<string, object> GenerateDefaultCurriculumValues()
    {
        // カリキュラムのすべての可能なプロパティに対してデフォルト値を設定する
        // このデフォルト値は、システムの要件に応じて適宜変更してください
        var defaultValues = new Dictionary<string, object>();
        defaultValues.Add("Phase", -1);
        defaultValues.Add("GoalCount", 0);
        defaultValues.Add("RiserHeight", 0.0f); // RiserRatioがRiserHeightに変更されたと仮定
        defaultValues.Add("WeightRatio", 0.0f);
        defaultValues.Add("GoalDistanceFactor", 0.0f);
        // 他のすべてのプロパティに対しても同様にデフォルト値を追加する
        return defaultValues;
    }


    public string textOfCurrentCurriculum()
    {
        var currentCurriculum = Curriculums[currentPhaseIndex].GetAllProperties();
        var propertyStrings = currentCurriculum.Select(kv => $"{kv.Key}: {kv.Value}");
        string printText = string.Join(", ", propertyStrings);
        return printText;
    }

    private void WriteToCSV(int currentPhase, int stepCount)
    {
        if (csvFileName == null)
        {
            string dateTime = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            csvFileName = $"Assets/CurriculumStartAtStep_{dateTime}.csv"; // 初めてのカリキュラム変化時のステップ数をファイル名として使用
        }
        string newLine = string.Format("{0},{1}\n", currentPhase, stepCount);
        File.AppendAllText(csvFileName, newLine);
    }

    // goalFlagの履歴を外部から参照できるようにするためのプロパティ
    public string GoalFlagHistoryStr
    {
        get
        {
            return string.Join("", goalFlagHistory.ConvertAll(g => g ? "1" : "0"));
        }
    }
}
