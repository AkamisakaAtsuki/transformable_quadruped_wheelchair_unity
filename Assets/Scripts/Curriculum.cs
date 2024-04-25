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
    public TextAsset curriculumFile; // Unity�G�f�B�^����A�^�b�`����CSV�`���̃J���L�������t�@�C��
    private int currentPhaseIndex = 0; // ���݂̃J���L�������t�F�[�Y�̃C���f�b�N�X
    public int currentGoalsAchieved = 0; // ���݂̃t�F�[�Y�ŒB�����ꂽ�S�[����
    private List<bool> goalFlagHistory = new List<bool>(); // goalFlag�̗�����ۑ����郊�X�g��ǉ�

    private string csvFileName = null; // CSV�t�@�C������ێ�����ϐ�

    /*   public void InitCurriculum(int startCuriculumNumber = 0)
       {
           currentPhaseIndex = startCuriculumNumber;
           string[] data = curriculumFile.text.Split(new char[] { '\n' });
           string[] headers = data[0].Split(','); // �w�b�_�[�s�����
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
                string header = headers[j].Trim(); // �J������
                object value = row[j].Trim(); // �l���g�������ė]���ȃX�y�[�X���폜
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

    /*    public void InitCurriculum(int startCuriculumNumber=0)  // �f�t�H���g��0�Ԃ���J���L���������X�^�[�g�B�r���ŃG�f�B�^��������Ȃǂ��������ꍇ�͓r������X�^�[�g�ł���悤�ɂ��Ă���
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
    // �J���L��������Ԃ��B�S�ẴJ���L���������I��������AUnity�G�f�B�^�������Œ�~������B
    // ����܂ł̊w�K�X�e�b�v�����󂯎��A�J���L���������ω������ۂ�CSV�̒l���L�^����
    /*public (int Phase, float RiserRatio, float WeightRatio, float GoalDistanceFactor) MonitorCurriculum(bool goalFlag, int totalStepCount)
    {
        // goalFlag�̗������X�V
        goalFlagHistory.Add(goalFlag);
        if (goalFlagHistory.Count > 10)
        {
            goalFlagHistory.RemoveAt(0);  // �ŐV��10�݂̂�ێ����邽�߁A�Â����̂��폜
        }
        int achievedGoalsInHistory = goalFlagHistory.FindAll(g => g).Count;  // Python API���琧�䂷��Ƃ��̕ȂɑΉ�����ׂɒǉ������R�[�h�i�@�j

        if (goalFlag)
        {
            currentGoalsAchieved++;
            *//*if (currentGoalsAchieved == Curriculums[currentPhaseIndex].GoalCount)*//*
            if (achievedGoalsInHistory == Curriculums[currentPhaseIndex].GoalCount)  // Python API���琧�䂷��Ƃ��̕ȂɑΉ�����ׂɒǉ������R�[�h�i�A�j
            {
                // ������CSV��currentPhaseIndex�ƃX�e�b�v������������
                WriteToCSV(currentPhaseIndex, totalStepCount);

                currentPhaseIndex++;
                currentGoalsAchieved = 0;
                goalFlagHistory = new List<bool>();
                if (currentPhaseIndex >= Curriculums.Count)
                {
                    #if UNITY_EDITOR
                    EditorApplication.isPlaying = false;
                    #endif
                    return (-1, 0f, 0f, 0f); // �G�f�B�^����~������̕Ԃ�l
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
        // goalFlag�̗������X�V
        goalFlagHistory.Add(goalFlag);
        if (goalFlagHistory.Count > 10)
        {
            goalFlagHistory.RemoveAt(0);  // �ŐV��10�݂̂�ێ����邽�߁A�Â����̂��폜
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
                // ������CSV��currentPhaseIndex�ƃX�e�b�v������������
                WriteToCSV(currentPhaseIndex, totalStepCount);

                currentPhaseIndex++;
                currentGoalsAchieved = 0;
                goalFlagHistory = new List<bool>();
                if (currentPhaseIndex >= Curriculums.Count)
                {
                    #if UNITY_EDITOR
                    EditorApplication.isPlaying = false;
                    #endif
                    // �G�f�B�^����~������̃f�t�H���g�l�𓮓I�ɐ���
                    return GenerateDefaultCurriculumValues();
                }
            }
        }
        else
        {
            currentGoalsAchieved = 0;
        }

        // ���݂̃J���L�������t�F�[�Y�̃f�[�^�������ɕϊ�����
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
        // �J���L�������̂��ׂẲ\�ȃv���p�e�B�ɑ΂��ăf�t�H���g�l��ݒ肷��
        // ���̃f�t�H���g�l�́A�V�X�e���̗v���ɉ����ēK�X�ύX���Ă�������
        var defaultValues = new Dictionary<string, object>();
        defaultValues.Add("Phase", -1);
        defaultValues.Add("GoalCount", 0);
        defaultValues.Add("RiserHeight", 0.0f); // RiserRatio��RiserHeight�ɕύX���ꂽ�Ɖ���
        defaultValues.Add("WeightRatio", 0.0f);
        defaultValues.Add("GoalDistanceFactor", 0.0f);
        // ���̂��ׂẴv���p�e�B�ɑ΂��Ă����l�Ƀf�t�H���g�l��ǉ�����
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
            csvFileName = $"Assets/CurriculumStartAtStep_{dateTime}.csv"; // ���߂ẴJ���L�������ω����̃X�e�b�v�����t�@�C�����Ƃ��Ďg�p
        }
        string newLine = string.Format("{0},{1}\n", currentPhase, stepCount);
        File.AppendAllText(csvFileName, newLine);
    }

    // goalFlag�̗������O������Q�Ƃł���悤�ɂ��邽�߂̃v���p�e�B
    public string GoalFlagHistoryStr
    {
        get
        {
            return string.Join("", goalFlagHistory.ConvertAll(g => g ? "1" : "0"));
        }
    }
}
