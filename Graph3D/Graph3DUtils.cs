using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Collections.Generic;

namespace Graph3DLibrary
{
    #region Engine3D.ErrorLog
    /// <summary>
    /// Обработчик ошибок
    /// </summary>
    public static class ErrorLog
    {
        private static string lastError = "";  // текст последней ошибки
        private static int lastErrorNumber = 0;// код последней ошибки
        public static long Counter = 0;

        /// <summary>
        /// Возвращает текст всех ошибок в Engine3D. после последнего вызова данной функции. 
        /// После вызова, текст ошибки сбрасывается.
        /// </summary>
        /// <returns>Текст всех последних ошибок в модуле</returns>
        public static string GetLastError(bool ClearError = true)
        {
            string s = lastError;
            lastError = "";
            if (ClearError) lastErrorNumber = 0;
            return s;
        }

        /// <summary>
        /// Возвращает код последней ошибки в модуле Engine3D. после последнего вызова данной функции. 
        /// После вызова, код ошибки сбрасывается в 0.
        /// </summary>
        /// <returns>Код последней ошибки</returns>
        public static int GetLastErrorNumber(bool ClearError = true)
        {
            int err = lastErrorNumber;
            lastErrorNumber = 0;
            if (ClearError) lastError = "";
            return err;
        }

        /// <summary>
        /// Протоколирование ошибки
        /// </summary>
        /// <param name="ClassName">Имя класса, в котором произошла ошибка</param>
        /// <param name="FunctionName">Имя функции, в которой произошла ошибка</param>
        /// <param name="Message">Текст ошибки</param>
        /// <param name="errCode">Код ошибки</param>
        public static void AddErrorMessage(string ClassName, string FunctionName, string Message, int errCode)
        {
            lastError += "\nКласс: '" + ClassName + "', функция '" + FunctionName + "', ошибка: " + Message;
            lastErrorNumber = errCode;
        }
    }
    #endregion

    #region FPSStatistics
    /// <summary>
    /// Статистика по FPS
    /// </summary>
    public class FPSStatistics
    {
        public DateTime[] TimePoint;
        public string[] PointDescription;
        public int CurrentTimePoint = 0;
        public FPSStatistics(string description = "start", int TimePointsLimit = 2)
        {
            if (TimePointsLimit < 2) TimePointsLimit = 2;
            TimePoint = new DateTime[TimePointsLimit];
            PointDescription = new string[TimePointsLimit];
            CurrentTimePoint = 0;
            Restart(description);
        }

        public int Restart(string description = "start")
        {
            try
            {
                CurrentTimePoint = 0;
                NextPoint(description);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("FPSStatistics", "Restart", er.Message, -1);
                return -1;
            }
        }

        public int NextPoint(string description)
        {
            try
            {
                if (CurrentTimePoint >= TimePoint.Length) throw new Exception("Превышен заданный лимит по контрольным точкам.");
                TimePoint[CurrentTimePoint] = DateTime.Now;
                PointDescription[CurrentTimePoint++] = description;
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("FPSStatistics", "NextPoint", er.Message, -1);
                return -1;
            }
        }

        public int FPS()
        {
            try
            {
                if (CurrentTimePoint < 2) throw new Exception("Не задано стартовое и конечное время.");
                long T = (TimePoint[CurrentTimePoint - 1].Ticks - TimePoint[0].Ticks);
                if (T < 1) T = 1;
                return (int)(10000000 / T);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("FPSStatistics", "FPS", er.Message, -1);
                return -1;
            }
        }

        public float Percent(int PointNumber = 1)
        {
            try
            {
                if (PointNumber < 1) throw new Exception("Некорректная точка.");
                if (CurrentTimePoint < 2) throw new Exception("Не задано стартовое и конечное время.");
                long T = (TimePoint[CurrentTimePoint - 1].Ticks - TimePoint[0].Ticks);
                if (T < 1) T = 1;
                return ((TimePoint[PointNumber].Ticks - TimePoint[PointNumber - 1].Ticks) * 100 / T);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("FPSStatistics", "Percent", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion
}