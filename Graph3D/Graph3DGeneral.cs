using System;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.Threading.Tasks;

/// <summary>
/// Модуль трехмерной GDI-графики. 
/// Разработал: Егоров Денис Алексеевич, 2001-2018 год
/// </summary>
namespace Graph3DLibrary
{
    #region Engine3D
    /// <summary>
    ///  Общие методы для работы с 3D-пространственными объектами 
    /// </summary>
    public static class Engine3D
    {
        /// <summary>
        /// Градус в радианах
        /// </summary>
        public const float RadianDegrees = (float)Math.PI / 180;
        public const float Radian90 = (float)Math.PI / 2;
        public const float Radian180 = (float)Math.PI;
        public const float Radian270 = (float)Math.PI * 3 / 2;
        public const float Radian360 = (float)Math.PI * 2;

        public const int MaxScreenValue = (int)5E8;

        /// <summary>
        /// Определение вхождения точки в треугольник (на плоскости)
        /// </summary>
        /// <param name="P1">Точка вершины треугольника</param>
        /// <param name="P2">Точка вершины треугольника</param>
        /// <param name="P3">Точка вершины треугольника</param>
        /// <param name="pTest">Тестовая точка</param>
        /// <returns></returns>
        public static bool TestPointInTrangle(Point P1, Point P2, Point P3, Point pTest)
        {
            int a = (P1.X - pTest.X) * (P2.Y - P1.Y) - (P2.X - P1.X) * (P1.Y - pTest.Y);
            int b = (P2.X - pTest.X) * (P3.Y - P2.Y) - (P3.X - P2.X) * (P2.Y - pTest.Y);
            int c = (P3.X - pTest.X) * (P1.Y - P3.Y) - (P1.X - P3.X) * (P3.Y - pTest.Y);
            return ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0));
        }

        /// <summary>
        /// Определение вхождения точки в треугольник (на плоскости)
        /// </summary>
        /// <param name="P1">Точка вершины треугольника</param>
        /// <param name="P2">Точка вершины треугольника</param>
        /// <param name="P3">Точка вершины треугольника</param>
        /// <param name="pTest">Тестовая точка</param>
        /// <returns></returns>
        public static bool TestPointInTrangle(PointF P1, PointF P2, PointF P3, PointF pTest)
        {
            float a = (P1.X - pTest.X) * (P2.Y - P1.Y) - (P2.X - P1.X) * (P1.Y - pTest.Y);
            float b = (P2.X - pTest.X) * (P3.Y - P2.Y) - (P3.X - P2.X) * (P2.Y - pTest.Y);
            float c = (P3.X - pTest.X) * (P1.Y - P3.Y) - (P1.X - P3.X) * (P3.Y - pTest.Y);
            return ((a >= 0 && b >= 0 && c >= 0) || (a <= 0 && b <= 0 && c <= 0));
        }


        /*
         * проверка принадлежности точки многоугольнику
                template <class T>
        bool pt_in_polygon(const T &test,const std::vector &polygon)
        {
        if (polygon.size()<3) return false; 

        std::vector::const_iterator end=polygon.end();

        T last_pt=polygon.back(); 

        last_pt.x-=test.x;
        last_pt.y-=test.y;

        double sum=0.0;

        for(
        std::vector::const_iterator iter=polygon.begin();
        iter!=end;
        ++iter
        )
        {
        T cur_pt=*iter;
        cur_pt.x-=test.x;
        cur_pt.y-=test.y;

        double del= last_pt.x*cur_pt.y-cur_pt.x*last_pt.y;
        double xy= cur_pt.x*last_pt.x+cur_pt.y*last_pt.y;

        sum+=
        (
        atan((last_pt.x*last_pt.x+last_pt.y*last_pt.y - xy)/del)+
        atan((cur_pt.x*cur_pt.x+cur_pt.y*cur_pt.y- xy )/del)
        );

        last_pt=cur_pt;

        }

        return fabs(sum)>eps;

        }



        T – тип точки, например:
        struct PointD
        {
        double x,y;
        };
         * 
         * 
         * 
         * Господа, кому интересно, привожу более быстрый алгоритм. Уже не мой. 
        Отдельное и огромное спасибо forgotten за статейку.
        template bool pt_in_polygon2(const T &test,const std::vector &polygon)
        {

        static const int q_patt[2][2]= { {0,1}, {3,2} };

        if (polygon.size()<3) return false;

        std::vector::const_iterator end=polygon.end();
        T pred_pt=polygon.back(); 
        pred_pt.x-=test.x;
        pred_pt.y-=test.y;

        int pred_q=q_patt[pred_pt.y<0][pred_pt.x<0];

        int w=0;

        for(std::vector::const_iterator iter=polygon.begin();iter!=end;++iter)
        {
        T cur_pt = *iter; 

        cur_pt.x-=test.x;
        cur_pt.y-=test.y;

        int q=q_patt[cur_pt.y<0][cur_pt.x<0];

        switch (q-pred_q)
        {
        case -3:++w;break;
        case 3:--w;break;
        case -2:if(pred_pt.x*cur_pt.y>=pred_pt.y*cur_pt.x) ++w;break;
        case 2:if(!(pred_pt.x*cur_pt.y>=pred_pt.y*cur_pt.x)) --w;break;
        }

        pred_pt = cur_pt;
        pred_q = q;

        }

        return w!=0;

        }
        */

        /// <summary>
        /// Фильтрация юнитов по Z-координате: скрытие расположенных слишкоми близко к наблюдателю юнитов (или за пределами тумана)
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int FilterUnitByZPos(Unit3D[] unit, ref int[] ActiveUnitIndex, ref int activeUnitCount, int MinZ = 0, int MaxZ = -1)
        {
            try
            {
                int i = 0;
                while (i < activeUnitCount)
                {
                    if ((unit[ActiveUnitIndex[i]].CameraPosition.Z + unit[ActiveUnitIndex[i]].ScreenRadius) <= MinZ)
                        ActiveUnitIndex[i--] = ActiveUnitIndex[--activeUnitCount];
                    i++;
                }

                if (MaxZ > MinZ)
                {
                    i = 0;
                    while (i < activeUnitCount)
                    {
                        if ((unit[ActiveUnitIndex[i]].CameraPosition.Z - unit[ActiveUnitIndex[i]].ScreenRadius) >= MaxZ)
                            ActiveUnitIndex[i--] = ActiveUnitIndex[--activeUnitCount];
                        i++;
                    }
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "FilterUnitByZPos", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Фильтрация юнитов по координатам XY (скрытие вышедших за пределы экрана юнитов)
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int FilterUnitByXYPos(Point ScreenSize, Unit3D[] unit, ref int[] ActiveUnitIndex, ref int activeUnitCount)
        {
            try
            {
                int i = 0;

                while (i < activeUnitCount)
                {
                    if (
                        ((unit[ActiveUnitIndex[i]].ScreenPosition.X + unit[ActiveUnitIndex[i]].ScreenRadius) <= 0)
                        |
                        ((unit[ActiveUnitIndex[i]].ScreenPosition.Y + unit[ActiveUnitIndex[i]].ScreenRadius) <= 0)
                        |
                        ((unit[ActiveUnitIndex[i]].ScreenPosition.X - unit[ActiveUnitIndex[i]].ScreenRadius) >= ScreenSize.X)
                        |
                        ((unit[ActiveUnitIndex[i]].ScreenPosition.Y - unit[ActiveUnitIndex[i]].ScreenRadius) >= ScreenSize.Y)
                        )
                        ActiveUnitIndex[i--] = ActiveUnitIndex[--activeUnitCount];
                    i++;
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "FilterUnitByXYPos", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Поворот координат вокруг оси
        /// </summary>
        /// <param name="angle">Угол (в радианах), на который необходмо повернуть координаты</param>
        /// <param name="axis">Ось, вокруг которой необходимо повернуть координаты</param>
        /// <param name="point">набор координат (точки в пространстве)</param>
        /// <returns>Возвращает 0 в случае успеха и не 0 в случае ошибки. Текст ошибки можно получить с помощью функции GetLastError()</returns>
        public static int RotatePoint3D(float angle, Axis3D axis, ref Point3D[] point, int firstIndex = 0, int lastIndex = -1)
        {
            try
            {
                float sin_angle = (float)Math.Sin(angle);
                float cos_angle = (float)Math.Cos(angle);
                float tmp = 0;

                if ((lastIndex < 0) | (lastIndex >= point.Length)) lastIndex = point.Length - 1;
                if (firstIndex < 0) firstIndex = 0;
                if (firstIndex >= point.Length) firstIndex = point.Length - 1;

                switch (axis)
                {
                    case Axis3D.OXyz:
                        for (int i = firstIndex; i <= lastIndex; i++)
                        {
                            tmp = point[i].Z * cos_angle - point[i].Y * sin_angle;
                            point[i].Y = point[i].Y * cos_angle + point[i].Z * sin_angle;
                            point[i].Z = tmp;
                        }
                        break;
                    case Axis3D.OYxz:
                        for (int i = firstIndex; i <= lastIndex; i++)
                        {
                            tmp = point[i].X * cos_angle - point[i].Z * sin_angle;
                            point[i].Z = point[i].Z * cos_angle + point[i].X * sin_angle;
                            point[i].X = tmp;
                        }
                        break;
                    case Axis3D.OZyx:
                        for (int i = firstIndex; i <= lastIndex; i++)
                        {
                            tmp = point[i].Y * cos_angle - point[i].X * sin_angle;
                            point[i].X = point[i].X * cos_angle + point[i].Y * sin_angle;
                            point[i].Y = tmp;
                        }
                        break;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "RotatePoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Поворот координат вокруг оси
        /// </summary>
        /// <param name="angle">Угол (в радианах), на который необходмо повернуть координаты</param>
        /// <param name="axis">Ось, вокруг которой необходимо повернуть координаты</param>
        /// <param name="point">набор координат (точка в пространстве)</param>
        /// <returns>Возвращает 0 в случае успеха и не 0 в случае ошибки. Текст ошибки можно получить с помощью функции GetLastError()</returns>
        public static int RotatePoint3D(float angle, Axis3D axis, Point3D point)
        {
            try
            {
                float sin_angle = (float)Math.Sin(angle);
                float cos_angle = (float)Math.Cos(angle);
                float tmp = 0;

                switch (axis)
                {
                    case Axis3D.OXyz:
                        tmp = point.Z * cos_angle - point.Y * sin_angle;
                        point.Y = point.Y * cos_angle + point.Z * sin_angle;
                        point.Z = tmp;
                        break;
                    case Axis3D.OYxz:
                        tmp = point.X * cos_angle - point.Z * sin_angle;
                        point.Z = point.Z * cos_angle + point.X * sin_angle;
                        point.X = tmp;
                        break;
                    case Axis3D.OZyx:
                        tmp = point.Y * cos_angle - point.X * sin_angle;
                        point.X = point.X * cos_angle + point.Y * sin_angle;
                        point.Y = tmp;
                        break;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "RotatePoint3D", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Масштабирование координат точки
        /// </summary>
        /// <param name="zoom">Коэффициент масштабирования</param>
        /// <param name="point">Набор координат</param>
        /// <returns>Возвращает 0 в случае успеха и не 0 в случае ошибки. Текст ошибки можно получить с помощью функции GetLastError()</returns>
        public static int ZoomPoint3D(float zoom, ref Point3D[] point)
        {
            try
            {
                for (int i=0;i<point.Length;i++)
                {
                    point[i].X *= zoom;
                    point[i].Y *= zoom;
                    point[i].Z *= zoom;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "ZoomPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Масштабирование координат точки
        /// </summary>
        /// <param name="zoomX">Коэффициент масштабирования по оси OX</param>
        /// <param name="zoomY">Коэффициент масштабирования по оси OY</param>
        /// <param name="zoomZ">Коэффициент масштабирования по оси OZ</param>
        /// <param name="point">Набор координат</param>
        /// <returns>Возвращает 0 в случае успеха и не 0 в случае ошибки. Текст ошибки можно получить с помощью функции GetLastError()</returns>
        public static int ZoomPoint3D(float zoomX, float zoomY, float zoomZ, ref Point3D[] point)
        {
            try
            {
                for (int i = 0; i < point.Length; i++)
                {
                    point[i].X *= zoomX;
                    point[i].Y *= zoomY;
                    point[i].Z *= zoomZ;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "ZoomPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Смещение координат 
        /// </summary>
        /// <param name="dX">Смещение по оси OX</param>
        /// <param name="dY">Смещение по оси OY</param>
        /// <param name="dZ">Смещение по оси OZ</param>
        /// <param name="point">набор координат (точки в пространстве)</param>
        /// <returns></returns>
        public static int MovePoint3D(float dX, float dY, float dZ, ref Point3D[] point, int firstIndex = 0, int lastIndex = -1)
        {
            try
            {
                if ((lastIndex < 0) | (lastIndex >= point.Length)) lastIndex = point.Length - 1;
                if (firstIndex < 0) firstIndex = 0;
                if (firstIndex >= point.Length) firstIndex = point.Length - 1;

                for (int i = firstIndex; i <= lastIndex; i++)
                {
                    point[i].X += dX;
                    point[i].Y += dY;
                    point[i].Z += dZ;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "MovePoint3D", er.Message, -1);
                return -1;
            }
        }


        

        /// <summary>
        /// Смещение координат 
        /// </summary>
        /// <param name="dX">Смещение по оси OX</param>
        /// <param name="dY">Смещение по оси OY</param>
        /// <param name="dZ">Смещение по оси OZ</param>
        /// <param name="point">координаты (точка в пространстве)</param>
        /// <returns></returns>
        public static int MovePoint3D(float dX, float dY, float dZ, Point3D point)
        {
            try
            {
                point.X += dX;
                point.Y += dY;
                point.Z += dZ;
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "MovePoint3D", er.Message, -1);
                return -1;
            }
        }        

        /// <summary>
        /// Преобразование координат в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="point3D">Набор преобразуемых координат</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int Point3DPresentationConversion(Point3D[] sourcePoint3D, Point3D[] destanationPoint3D, float K, int screenCenterX, int screenCenterY, int firstIndex = 0, int lastIndex = -1)
        {
            try
            {
                float z;
                float v;

                if (destanationPoint3D.Length < sourcePoint3D.Length) throw new Exception("Размер буфера-приёмника меньше размера буфера-источника.");
                if ((lastIndex < 0) | (lastIndex >= sourcePoint3D.Length)) lastIndex = sourcePoint3D.Length - 1;
                if (firstIndex < 0) firstIndex = 0;
                if (firstIndex >= sourcePoint3D.Length) return 0;

                if (K < 1) K = 1;

                for (int i = firstIndex; i <= lastIndex; i++)
                {
                    z = K / ((sourcePoint3D[i].Z < 1) ? 1 : sourcePoint3D[i].Z); 

                    v = sourcePoint3D[i].X * z;
                    destanationPoint3D[i].X = screenCenterX + (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                    v = sourcePoint3D[i].Y * z;
                    destanationPoint3D[i].Y = -screenCenterY - (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);                    

                    destanationPoint3D[i].Z = (float)Math.Round(sourcePoint3D[i].Z);
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "Point3DPresentationConversion(1)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование координат в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="point">Набор преобразуемых координат</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int Point3DPresentationConversion(Point3D[] point, float K, int screenCenterX, int screenCenterY, int firstIndex = 0, int lastIndex = -1)
        {
            try
            {
                float z;
                float v;

                if ((lastIndex < 0) | (lastIndex >= point.Length)) lastIndex = point.Length - 1;
                if (firstIndex < 0) firstIndex = 0;
                if (firstIndex >= point.Length) return 0;

                if (K < 1) K = 1;

                for (int i = firstIndex; i <= lastIndex; i++)
                {
                    z = K / ((point[i].Z <=1) ? 1 : point[i].Z);

                    v = point[i].X * z;
                    point[i].X = screenCenterX + (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                    v = point[i].Y * z;
                    point[i].Y = -screenCenterY - (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);
                    
                    point[i].Z = (int)Math.Round(point[i].Z);
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "Point3DPresentationConversion(2)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование индексированных координат в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="point">Набор преобразуемых координат</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int Point3DPresentationConversion(Point3D[] point, float K, int screenCenterX, int screenCenterY, int[] index, int indexCount, int firstIndex = 0, int lastIndex = -1)
        {
            try
            {
                float z;
                float v;

                if ((lastIndex < 0) | (lastIndex >= indexCount)) lastIndex = indexCount - 1;
                if (firstIndex < 0) firstIndex = 0;
                if (firstIndex >= indexCount) return 0;

                if (K < 1) K = 1;

                for (int i = firstIndex; i <= lastIndex; i++)
                {
                    z = K / ((point[index[i]].Z <= 1) ? 1 : point[index[i]].Z);

                    v = point[index[i]].X * z;
                    point[index[i]].X = screenCenterX + (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                    v = point[index[i]].Y * z;
                    point[index[i]].Y = -screenCenterY - (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                    point[index[i]].Z = (int)Math.Round(point[index[i]].Z);
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "Point3DPresentationConversion(3)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование координаты в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="point">Координата</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int Point3DPresentationConversion(Point3D point, float K, int screenCenterX, int screenCenterY)
        {
            try
            {
                float z;
                float v;

                if (K < 1) K = 1;

                z = K / ((point.Z <= 1) ? 1 : point.Z);

                v = point.X * z;
                point.X = screenCenterX + (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                v = point.Y * z;
                point.Y = -screenCenterY - (int)(v > MaxScreenValue ? MaxScreenValue : v < -MaxScreenValue ? -MaxScreenValue : v);

                point.Z = (int)Math.Round(point.Z);

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "Point3DPresentationConversion(4)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование единичного значения в соответствии с перспективной проекцией в указанной пространственной позиции
        /// </summary>
        /// <param name="positionZ"Z-координата пространственной позиции></param>
        /// <param name="BaseValue">Исходное значение</param>
        /// <param name="ConvertedValue">Сконвертированное значение</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int ValuePresentationConversion(float positionZ, float BaseValue, ref float ConvertedValue, float K)
        {
            try
            {
                if (K < 1) K = 1;
                ConvertedValue = BaseValue * K / ((positionZ <= 1) ? 1 : positionZ);

                if (ConvertedValue > MaxScreenValue) ConvertedValue = MaxScreenValue;
                if (ConvertedValue < -MaxScreenValue) ConvertedValue = -MaxScreenValue;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "ValuePresentationConversion(1)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование единичного значения в соответствии с перспективной проекцией в указанной пространственной позиции
        /// </summary>
        /// <param name="positionZ"Z-координата пространственной позиции></param>
        /// <param name="BaseValue">Исходное значение</param>
        /// <param name="ConvertedValue">Сконвертированное значение</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0: все координаты преобразованы успешно; значение не 0: ошибка</returns>
        public static int ValuePresentationConversion(Point3D[] point, float[] Value, float K, int pointCount = -1)
        {
            try
            {
                if (K < 1) K = 1;
                if (pointCount == -1) pointCount = point.Length;
                for (int i = 0; i < pointCount; i++)
                {
                    Value[i] *= K / ((point[i].Z <= 1) ? 1 : point[i].Z);

                    if (Value[i] > MaxScreenValue) Value[i] = MaxScreenValue;
                    if (Value[i] < -MaxScreenValue) Value[i] = -MaxScreenValue;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "ValuePresentationConversion(2)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Вычисление косинуса угла между двумя нормализованными векторами, выходящими из общей вершины
        /// </summary>
        /// <param name="vector1">Вектор №1</param>
        /// <param name="vector2">Вектор №2</param>
        /// <returns>Косинус угла между двумя векторами, выходящими из общей вершины</returns>
        public static float GetCosAngleVectors(Point3D vector1, Point3D vector2)
        {
            return (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z);
        }

        /// <summary>
        /// Нормализация угла до 0..2PI
        /// </summary>
        /// <param name="radianAngle"></param>
        /// <returns></returns>
        public static float GetNormalizeDegrees(float radianAngle)
        {
            int delta = (int)(radianAngle / Engine3D.Radian360 + 1);
            return radianAngle - (delta - 1) * Engine3D.Radian360;
        }

        /// <summary>
        /// Создает копию массива Point3D
        /// </summary>
        /// <param name="source">Исходный массив</param>
        /// <param name="dest">Получаемый массив</param>
        /// /// <param name="withCreate">Признак, указывает создавать ли получаемый массив или использовать уже выделенную память</param>
        /// <returns>Возвращает 0 в случае успеха, не 0 в случае ошибки</returns>
        public static int CopyPoint3D(Point3D[] source, ref Point3D[] dest, bool withCreate = false)
        {
            try
            {
                if (withCreate)
                {
                    dest = new Point3D[source.Length];
                    for (int i = 0; i < source.Length; i++) dest[i] = new Point3D(source[i]);
                }
                else
                {
                    if (dest.Length < source.Length) throw new Exception("Длинна исходного массива больше длинны массива-получателя");
                    for (int i = 0; i < source.Length; i++) dest[i].CopyFrom(source[i]);
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "CopyPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Копирует координаты Point3D, при необходимости создавая новый экземпляр класса Point3D
        /// </summary>
        /// <param name="source">Источник</param>
        /// <param name="dest">Получаемый экземпляр</param>
        /// /// <param name="withCreate">Признак, указывает создавать ли получаемый экземпляр или использовать уже выделенную память</param>
        /// <returns>Возвращает 0 в случае успеха, не 0 в случае ошибки</returns>
        public static int CopyPoint3D(Point3D source, ref Point3D dest, bool withCreate = false)
        {
            try
            {
                if (withCreate) dest = new Point3D();
                dest.CopyFrom(source);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "CopyPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Копирует координаты Point3D, при необходимости создавая новый экземпляр класса Point3D
        /// </summary>
        /// <param name="sourceX">X-координата источника</param>
        /// <param name="sourceY">Y-координата источника</param>
        /// <param name="sourceZ">Z-координата источника</param>
        /// <param name="dest">Получаемый экземпляр</param>
        /// /// <param name="withCreate">Признак, указывает создавать ли получаемый экземпляр или использовать уже выделенную память</param>
        /// <returns>Возвращает 0 в случае успеха, не 0 в случае ошибки</returns>
        public static int CopyPoint3D(int sourceX, int sourceY, int sourceZ, ref Point3D dest, bool withCreate = false)
        {
            try
            {
                if (withCreate) dest = new Point3D();
                dest.CopyFrom(sourceX, sourceY, sourceZ);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "CopyPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Копирует координаты Point3D, при необходимости создавая новый экземпляр класса Point3D
        /// </summary>
        /// <param name="sourceX">X-координата источника</param>
        /// <param name="sourceY">Y-координата источника</param>
        /// <param name="sourceZ">Z-координата источника</param>
        /// <param name="dest">Получаемый экземпляр</param>
        /// /// <param name="withCreate">Признак, указывает создавать ли получаемый экземпляр или использовать уже выделенную память</param>
        /// <returns>Возвращает 0 в случае успеха, не 0 в случае ошибки</returns>
        public static int CopyPoint3D(float sourceX, float sourceY, float sourceZ, ref Point3D dest, bool withCreate = false)
        {
            try
            {
                if (withCreate) dest = new Point3D();
                dest.CopyFrom(sourceX, sourceY, sourceZ);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "CopyPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Построение нормали к двум векторам, исходящим из единой вершины
        /// </summary>
        /// <param name="Vector12">Первый вектор</param>
        /// <param name="Vector32">Второй вектор</param>
        /// <param name="NormalVector">Нормаль</param>
        /// <returns></returns>
        public static int BuiltNormal(Point3D Vector12, Point3D Vector32, ref Point3D NormalVector)
        {
            try
            {
                NormalVector.X = -Vector12.Z * Vector32.Y + Vector12.Y * Vector32.Z;
                NormalVector.Y = -Vector12.X * Vector32.Z + Vector12.Z * Vector32.X;
                NormalVector.Z = -Vector12.Y * Vector32.X + Vector12.X * Vector32.Y;
                return NormalizeVector(NormalVector);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "BuiltNormal", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Построение нормали к двум векторам, исходящим из единой вершины
        /// </summary>
        /// <param name="Vector12">Первый вектор</param>
        /// <param name="Vector32">Второй вектор</param>
        /// <param name="NormalVector">Нормаль</param>
        /// <returns></returns>
        public static int BuiltNormal(Point3D Vector12, Point3D Vector32, ref float zAxis)
        {
            try
            {
                zAxis = -Vector12.Y * Vector32.X + Vector12.X * Vector32.Y;
                return NormalizeVector(ref zAxis);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "BuiltNormal", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Нормализация вектора
        /// </summary>
        /// <param name="vector">Вектор</param>
        /// <param name="zOnly">Признак нормализации только z-координаты</param>
        /// <returns></returns>
        public static int NormalizeVector(Point3D vector)
        {
            try
            {
                float viR = (float)Math.Sqrt(vector.X * vector.X + vector.Y * vector.Y + vector.Z * vector.Z);
                viR = (viR < 0.001) ? float.MaxValue : 1 / viR;

                vector.X *= viR;
                vector.Y *= viR;
                vector.Z *= viR;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "NormalizeVector", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Нормализация z-составляющей вектора
        /// </summary>
        /// <param name="zAxis"></param>
        /// <returns></returns>
        public static int NormalizeVector(ref float zAxis)
        {
            try
            {
                zAxis = (Math.Abs(zAxis) < 0.001) ? 0 : (zAxis > 0) ? 1 : -1;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "NormalizeVector", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Нормализация вектора
        /// </summary>
        /// <param name="vector">Вектор</param>
        /// <param name="VectorLength">Длина вектора</param>
        /// <returns></returns>
        public static int NormalizeVector(Point3D vector, float VectorLength)
        {
            try
            {
                VectorLength = (VectorLength < 0.0001) ? float.MaxValue : 1 / VectorLength;

                vector.X *= VectorLength;
                vector.Y *= VectorLength;
                vector.Z *= VectorLength;

                return 0;
            }

            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "NormalizeVector", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Подготовка статичного полигона
        /// </summary>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="polygon">Динамический полигон, ссылающийся на набор вершин</param>
        /// <param name="staticPolygon">Получаемый статичный полигон</param>
        /// <returns></returns>
        public static int BuiltStaticPolygon(ref Point3D[] vertex3D, ref Polygon3D polygon, ref PointF[] staticPolygon)
        {
            try
            {
                staticPolygon[0].X = vertex3D[polygon.PointIndex[0]].X;
                staticPolygon[0].Y = vertex3D[polygon.PointIndex[0]].Y;

                staticPolygon[1].X = vertex3D[polygon.PointIndex[1]].X;
                staticPolygon[1].Y = vertex3D[polygon.PointIndex[1]].Y;

                staticPolygon[2].X = vertex3D[polygon.PointIndex[2]].X;
                staticPolygon[2].Y = vertex3D[polygon.PointIndex[2]].Y;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "BuiltStaticPolygon", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Вычисление дистанции между точкой и координатой  (0,0,0) в пространстве
        /// </summary>
        /// <param name="point">точка в пространстве</param>
        /// <returns></returns>
        public static float GetDistance(Point3D point)
        {
            return (float)Math.Sqrt(point.X * point.X + point.Y * point.Y + point.Z * point.Z);
        }

        /// <summary>
        /// Вычисление дистанции между точкой и координатой  (0,0) на плоскости
        /// </summary>
        /// <param name="point">точка на плоскости</param>
        /// <returns></returns>
        public static float GetDistance(Point point)
        {
            return (float)Math.Sqrt(point.X * point.X + point.Y * point.Y);
        }

        /// <summary>
        /// Вычисление дистанции между двумя точками в пространстве
        /// </summary>
        /// <param name="point1">Первая точка</param>
        /// <param name="point2">Вторая точка</param>
        /// <returns></returns>
        public static float GetDistance(Point3D point1, Point3D point2)
        {
            return (float)Math.Sqrt(Math.Pow(point1.X - point2.X, 2) + Math.Pow(point1.Y - point2.Y, 2) + Math.Pow(point1.Z - point2.Z, 2));
        }

        /// <summary>
        /// Вычисление дистанции между двумя точками на плоскости
        /// </summary>
        /// <param name="point1">Первая точка</param>
        /// <param name="point2">Вторая точка</param>
        /// <returns></returns>
        public static float GetDistance(Point point1, Point point2)
        {
            return (float)Math.Sqrt(Math.Pow(point1.X - point2.X, 2) + Math.Pow(point1.Y - point2.Y, 2));
        }

        /// <summary>
        /// Вычисление квадрата дистанции между двумя точками в пространстве
        /// </summary>
        /// <param name="point1">Первая точка</param>
        /// <param name="point2">Вторая точка</param>
        /// <returns></returns>
        public static float GetSquareDistance(Point3D point1, Point3D point2)
        {
            return (float)(Math.Pow(point1.X - point2.X, 2) + Math.Pow(point1.Y - point2.Y, 2) + Math.Pow(point1.Z - point2.Z, 2));
        }

        /// <summary>
        /// Вычисление квадрата длинны вектора из точки (0,0,0) к указанной
        /// </summary>
        /// <param name="point1">Первая точка</param>
        /// <param name="point2">Вторая точка</param>
        /// <returns></returns>
        public static float GetSquareDistance(Point3D point1)
        {
            return point1.X * point1.X + point1.Y * point1.Y + point1.Z * point1.Z;
        }

        /// <summary>
        /// Расчет освещения полигонов от направленного бесконечно удаленного точечного источника света ("солнечный свет")
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightVector">Вектор направления на источник света</param>
        /// <param name="LightColor">Цвет источника света</param>
        /// <param name="LightPower">Сила света на нулевом расстоянии от точечного источника света (от 0 )</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddDirectionalLighting(ref Polygon3D[] polygon, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightVector, Color LightColor, float LightPower)
        {
            try
            {
                float lightNormalCosAngle = 0;
                float tmp = 0;
                int R;
                int G;
                int B;
                int sideIdx;
                float power;
                int dC;
                int sideCount;
                Point3D reflectedVector = new Point3D();

                if (LightPower < 0) LightPower = 0;

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);
                    
                    lightNormalCosAngle = Engine3D.GetCosAngleVectors(LightVector, polygon[activePolygonIndexes[i]].Normal);
                    if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    power = lightNormalCosAngle * LightPower;

                    #region Вычисляем отраженный луч
                    /*
                     Vector a = ..;//вектор, который мы отражаем(вектор из начала, до точки пересечения)
                     Vector n = ..;//нормаль в точке пересечения
                     Vector b;//результат
                     float k = n.x*a.x+n.y*a.y+n.z*a.z;
                     b.x = a.x + n.x * k *-2.0f;
                     b.y = a.y + n.y * k *-2.0f;
                     b.x = a.z + n.z * k *-2.0f;
                     */


                    lightNormalCosAngle = polygon[activePolygonIndexes[i]].Normal.Z * lightNormalCosAngle * 2 - LightVector.Z;
                    if (lightNormalCosAngle < 0) lightNormalCosAngle = 0;
                    #endregion

                    #region Оптимизация вычислений
                    power *= (float)0.5;
                    lightNormalCosAngle += lightNormalCosAngle;
                    #endregion

                    for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                    {
                        #region Оптимизация
                        tmp = power * ((1 - polygon[activePolygonIndexes[i]].Matte[sideIdx]) * lightNormalCosAngle + polygon[activePolygonIndexes[i]].Matte[sideIdx]);
                        #endregion

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].R + LightColor.R) * tmp);
                        R = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R + ((dC > 0) ? dC : 0);
                        if (R > 255) R = 255; else if (R < 0) R = 0;

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].G + LightColor.G) * tmp);
                        G = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G + ((dC > 0) ? dC : 0);
                        if (G > 255) G = 255; else if (G < 0) G = 0;

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].B + LightColor.B) * tmp);
                        B = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B + ((dC > 0) ? dC : 0);
                        if (B > 255) B = 255; else if (B < 0) B = 0;

                        polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R, G, B);

                        power = -power;
                    }
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddDirectionalLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения полигонов фоновым светом ("рассеянное освещение")
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightColor">Цвет точечного источника света</param>
        /// <param name="LightPower">Сила света на нулевом расстоянии от точечного источника света (от 0 )</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddAmbientLighting(ref Polygon3D[] polygon, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Color LightColor, float LightPower)
        {
            try
            {
                int R;
                int G;
                int B;
                int sideCount;
                int sideIdx;

                if (LightPower < 0) LightPower = 0;
                LightPower *= (float)0.5; // для расчета среднего по двум компонентам цвета                

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);

                    for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                    {

                        R = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R + (int)((polygon[activePolygonIndexes[i]].color[sideIdx].R + LightColor.R) * LightPower);
                        if (R > 255) R = 255; else if (R < 0) R = 0;

                        G = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G + (int)((polygon[activePolygonIndexes[i]].color[sideIdx].G + LightColor.G) * LightPower);
                        if (G > 255) G = 255; else if (G < 0) G = 0;

                        B = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B + (int)((polygon[activePolygonIndexes[i]].color[sideIdx].B + LightColor.B) * LightPower);
                        if (B > 255) B = 255; else if (B < 0) B = 0;

                        polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R, G, B);
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddAmbientLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения полигонов от точечного источника света ("лампа")
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightPoint">Координаты точечного источника света</param>
        /// <param name="LightColor">Цвет точечного источника света</param>
        /// <param name="LightPower">Сила света на нулевом расстоянии от точечного источника света (от 0 )</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddPointLighting(ref Polygon3D[] polygon, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightPoint, Color LightColor, float LightPower)
        {
            try
            {
                float lightNormalCosAngle = 0;
                float tmp = 0;
                int R;
                int G;
                int B;
                int dC;
                float power;
                float distK;
                float dist;
                int sideCount;
                int sideIdx;
                Point3D LightVector = new Point3D();
                Graph3DLibrary.ErrorLog.GetLastErrorNumber();

                if (LightPower < 0) LightPower = 0;                

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);

                    LightVector.X = polygon[activePolygonIndexes[i]].Center.X - LightPoint.X;
                    LightVector.Y = polygon[activePolygonIndexes[i]].Center.Y - LightPoint.Y;
                    LightVector.Z = polygon[activePolygonIndexes[i]].Center.Z - LightPoint.Z;

                    dist = GetDistance(polygon[activePolygonIndexes[i]].Center, LightPoint);

                    if (NormalizeVector(LightVector, dist) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                    #region Находим угол между нормалью полигона и вектором на источник света
                    lightNormalCosAngle = Engine3D.GetCosAngleVectors(LightVector, polygon[activePolygonIndexes[i]].Normal);
                    if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    #endregion

                    if (dist > 1)
                        distK = 1 / dist;
                    else
                        distK = 1;

                    power = lightNormalCosAngle * LightPower * distK;

                    #region Вычисляем отраженный луч
                    lightNormalCosAngle = polygon[activePolygonIndexes[i]].Normal.Z * lightNormalCosAngle * 2 - LightVector.Z;
                    if (lightNormalCosAngle < 0) lightNormalCosAngle = 0;
                    #endregion

                    #region Оптимизация вычислений
                    power *= (float)0.5;
                    lightNormalCosAngle *= 3;
                    #endregion

                    for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                    {
                        #region Оптимизация
                        tmp = power * ((1 - polygon[activePolygonIndexes[i]].Matte[sideIdx]) * lightNormalCosAngle + polygon[activePolygonIndexes[i]].Matte[sideIdx]);
                        #endregion

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].R + LightColor.R) * tmp);
                        R = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R + (dC > 0 ? dC : 0);
                        if (R > 255) R = 255; else if (R < 0) R = 0;

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].G + LightColor.G) * tmp);
                        G = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G + (dC > 0 ? dC : 0);
                        if (G > 255) G = 255; else if (G < 0) G = 0;

                        dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].B + LightColor.B) * tmp);
                        B = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B + (dC > 0 ? dC : 0);
                        if (B > 255) B = 255; else if (B < 0) B = 0;

                        polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R, G, B);
                        power = -power;
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddDirectionalLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения полигонов от направленного источника света ("фонарь")
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightPoint">Координаты точечного источника света</param>
        /// <param name="LightVector">Направление точечного источника света</param>
        /// <param name="InternalConAngle">Угол внутренноего конуса освещения</param>
        /// <param name="ExternalConAngle">Угол внешнего конуса освещения</param>
        /// <param name="LightColor">Цвет точечного источника света</param>
        /// <param name="LightPower">Сила света на нулевом расстоянии от точечного источника света (от 0 )</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>

        public static int AddSpotLighting(ref Polygon3D[] polygon, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightPoint, Point3D LightVector, float CosinusInternalAngle, float CosinusExternalAngle, Color LightColor, float LightPower)
        {
            try
            {
                float lightNormalCosAngle = 0;
                float polyLightCosAngle = 0;
                float tmp = 0;
                int R;
                int G;
                int B;
                int dC;
                float power;
                float distK;
                float dist;
                int sideCount;
                int sideIdx;

                float deltaIEx = (CosinusInternalAngle - CosinusExternalAngle);
                Point3D PolyLightVector = new Point3D();

                if (LightPower <= 0) return 0;

                Graph3DLibrary.ErrorLog.GetLastErrorNumber();

                if (CosinusInternalAngle < CosinusExternalAngle) throw new Exception("Внутренний конус освещения не может быть больше внешнего");
                if (deltaIEx > 0.0001) deltaIEx = 1 / deltaIEx;
                else deltaIEx = float.MaxValue;                

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);

                    PolyLightVector.X = polygon[activePolygonIndexes[i]].Center.X - LightPoint.X;
                    PolyLightVector.Y = polygon[activePolygonIndexes[i]].Center.Y - LightPoint.Y;
                    PolyLightVector.Z = polygon[activePolygonIndexes[i]].Center.Z - LightPoint.Z;

                    dist = GetDistance(polygon[activePolygonIndexes[i]].Center, LightPoint);

                    if (dist > 1) distK = 1 / dist;
                    else distK = 1;

                    if (NormalizeVector(PolyLightVector, dist) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                    #region Находим угол между вектором света и вектором на источник света от полигона
                    polyLightCosAngle = Engine3D.GetCosAngleVectors(PolyLightVector, LightVector);
                    if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    #endregion

                    if (polyLightCosAngle > CosinusExternalAngle)
                    {
                        #region Находим угол между нормалью полигона и вектором на источник света
                        lightNormalCosAngle = Engine3D.GetCosAngleVectors(PolyLightVector, polygon[activePolygonIndexes[i]].Normal);
                        if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                        #endregion

                        #region Расчет силы освещенности                        
                        if (polyLightCosAngle < CosinusInternalAngle) power = (polyLightCosAngle - CosinusExternalAngle) * deltaIEx;
                        else power = 1;
                        power *= lightNormalCosAngle * LightPower * distK * (float)0.5;
                        #endregion

                        #region Вычисляем отраженный блик
                        lightNormalCosAngle = (polygon[activePolygonIndexes[i]].Normal.Z * lightNormalCosAngle * 2 - PolyLightVector.Z) * 3;
                        if (lightNormalCosAngle < 0) lightNormalCosAngle = 0;
                        #endregion

                        for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                        {
                            #region Оптимизация
                            tmp = power * ((1 - polygon[activePolygonIndexes[i]].Matte[sideIdx]) * lightNormalCosAngle + polygon[activePolygonIndexes[i]].Matte[sideIdx]);
                            #endregion

                            dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].R + LightColor.R) * tmp);
                            R = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R + (dC > 0 ? dC : 0);
                            if (R > 255) R = 255; else if (R < 0) R = 0;

                            dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].G + LightColor.G) * tmp);
                            G = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G + (dC > 0 ? dC : 0);
                            if (G > 255) G = 255; else if (G < 0) G = 0;

                            dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].B + LightColor.B) * tmp);
                            B = polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B + (dC > 0 ? dC : 0);
                            if (B > 255) B = 255; else if (B < 0) B = 0;

                            polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R, G, B);
                            power = -power;
                        }
                    }
                }
                return 0;

            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddSpotLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет мягкого освещения полигонов от направленного источника света ("фонарь")
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightPoint">Координаты точечного источника света</param>
        /// <param name="LightVector">Направление точечного источника света</param>
        /// <param name="InternalConAngle">Угол внутренноего конуса освещения</param>
        /// <param name="ExternalConAngle">Угол внешнего конуса освещения</param>
        /// <param name="LightColor">Цвет точечного источника света</param>
        /// <param name="LightPower">Сила света на нулевом расстоянии от точечного источника света (от 0 )</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddSpotSoftLighting(ref Polygon3D[] polygon, ref Point3D[] vertex3D, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightPoint, Point3D LightVector, float CosinusInternalAngle, float CosinusExternalAngle, Color LightColor, float LightPower)
        {
            try
            {
                float lightNormalCosAngle = 0;
                float polyLightCosAngle = 0;
                float tmp = 0;
                int[] R = new int[2];
                int[] G = new int[2];
                int[] B = new int[2];
                int[] countR = new int[2];
                int[] countG = new int[2];
                int[] countB = new int[2];
                int j;
                int dC;
                float power;
                float distK;
                float dist;
                int sideCount;
                int sideIdx;

                float deltaIEx = (CosinusInternalAngle - CosinusExternalAngle);
                Point3D PolyLightVector = new Point3D();

                if (LightPower <= 0) return 0;

                Graph3DLibrary.ErrorLog.GetLastErrorNumber();

                if (CosinusInternalAngle < CosinusExternalAngle) throw new Exception("Внутренний конус освещения не может быть больше внешнего");
                if (deltaIEx > 0.0001) deltaIEx = 1 / deltaIEx;
                else deltaIEx = float.MaxValue;

                

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);

                    R[0] = R[1] = G[0] = G[1] = B[0] = B[1] = 0;
                    countR[0] = countR[1] = countG[0] = countG[1] = countB[0] = countB[1] = 0;

                    for (j = 0; j < 3; j++)
                    {
                        PolyLightVector.X = vertex3D[polygon[activePolygonIndexes[i]].PointIndex[j]].X - LightPoint.X;
                        PolyLightVector.Y = vertex3D[polygon[activePolygonIndexes[i]].PointIndex[j]].Y - LightPoint.Y;
                        PolyLightVector.Z = vertex3D[polygon[activePolygonIndexes[i]].PointIndex[j]].Z - LightPoint.Z;
                        dist = GetDistance(vertex3D[polygon[activePolygonIndexes[i]].PointIndex[j]], LightPoint);

                        if (dist > 1) distK = 1 / dist;
                        else distK = 1;

                        if (NormalizeVector(PolyLightVector, dist) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                        #region Находим угол между вектором света и вектором на источник света от полигона
                        polyLightCosAngle = Engine3D.GetCosAngleVectors(PolyLightVector, LightVector);
                        if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                        #endregion

                        if (polyLightCosAngle > CosinusExternalAngle)
                        {
                            #region Находим угол между нормалью полигона и вектором на источник света
                            lightNormalCosAngle = Engine3D.GetCosAngleVectors(PolyLightVector, polygon[activePolygonIndexes[i]].Normal);
                            if (Graph3DLibrary.ErrorLog.GetLastErrorNumber() != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                            #endregion

                            #region Расчет силы освещенности
                            if (polyLightCosAngle < CosinusInternalAngle) power = (polyLightCosAngle - CosinusExternalAngle) * deltaIEx;
                            else power = 1;
                            power *= lightNormalCosAngle * LightPower * distK * (float)0.5;
                            #endregion

                            #region Вычисляем отраженный блик
                            lightNormalCosAngle = (polygon[activePolygonIndexes[i]].Normal.Z * lightNormalCosAngle * 2 - PolyLightVector.Z) * 3;
                            if (lightNormalCosAngle < 0) lightNormalCosAngle = 0;
                            #endregion

                            for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                            {
                                #region Оптимизация
                                tmp = power * ((1 - polygon[activePolygonIndexes[i]].Matte[sideIdx]) * lightNormalCosAngle + polygon[activePolygonIndexes[i]].Matte[sideIdx]);
                                #endregion

                                dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].R + LightColor.R) * tmp);
                                R[sideIdx] += (dC > 0 ? dC : 0);
                                if (dC > 0) countR[sideIdx]++;

                                dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].G + LightColor.G) * tmp);
                                G[sideIdx] += (dC > 0 ? dC : 0);
                                if (dC > 0) countG[sideIdx]++;

                                dC = (int)((polygon[activePolygonIndexes[i]].color[sideIdx].B + LightColor.B) * tmp);
                                B[sideIdx] += (dC > 0 ? dC : 0);
                                if (dC > 0) countB[sideIdx]++;
                                power = -power;
                            }
                        }
                    }
                    for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                    {
                        if (countR[sideIdx] < 1) countR[sideIdx] = 1;
                        if (countG[sideIdx] < 1) countG[sideIdx] = 1;
                        if (countB[sideIdx] < 1) countB[sideIdx] = 1;
                        R[sideIdx] = (int)((float)R[sideIdx] / countR[sideIdx]);
                        G[sideIdx] = (int)((float)G[sideIdx] / countR[sideIdx]);
                        B[sideIdx] = (int)((float)B[sideIdx] / countR[sideIdx]);
                        R[sideIdx] += polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R;
                        G[sideIdx] += polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G;
                        B[sideIdx] += polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B;
                        if (R[sideIdx] > 255) R[sideIdx] = 255; else if (R[sideIdx] < 0) R[sideIdx] = 0;
                        if (G[sideIdx] > 255) G[sideIdx] = 255; else if (G[sideIdx] < 0) G[sideIdx] = 0;
                        if (B[sideIdx] > 255) B[sideIdx] = 255; else if (B[sideIdx] < 0) B[sideIdx] = 0;
                        polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R[sideIdx], G[sideIdx], B[sideIdx]);
                    }
                }
                return 0;

            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddSpotSoftLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения полигонов указанным источником света
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightVector">Вектор света</param>
        /// <param name="LightColor">Цвет источника света</param>
        /// <param name="LightPower">Сила света (от 0 до 1)</param>
        /// <param name="LightType">Тип источника света</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddLight(ref Polygon3D[] polygon, ref Point3D[] vertex3D, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightVector, Color LightColor, float LightPower, LightTypes LightType = LightTypes.Directional)
        {
            try
            {
                switch (LightType)
                {
                    case LightTypes.Directional:
                        return AddDirectionalLighting(ref polygon, ref activePolygonIndexes, activePolygonIndexesCount, LightVector, LightColor, LightPower);
                    case LightTypes.Point:
                        return AddPointLighting(ref polygon, ref activePolygonIndexes, activePolygonIndexesCount, LightVector, LightColor, LightPower);
                    case LightTypes.Ambient:
                        return AddAmbientLighting(ref polygon, ref activePolygonIndexes, activePolygonIndexesCount, LightColor, LightPower);
                    case LightTypes.Spot:
                    default:
                        throw new Exception("Для указанного типа источника света нет обработки");
                }
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddLight(1)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения полигонов указанным источником света
        /// </summary>
        /// <param name="polygon">Набор полигонов</param>
        /// <param name="vertex3D">Набор вершин</param>
        /// <param name="activePolygonIndexes">Фильтр полигонов</param>
        /// <param name="activePolygonIndexesCount">Объем фильтра полигонов</param>
        /// <param name="LightPoint">Расположение источника света</param>
        /// <param name="LightVector">Вектор света</param>
        /// <param name="InternalConAngle">Косинус угла внутренноего конуса света</param>
        /// <param name="ExternalConAngle">Косинус угла внешнего конуса света</param>
        /// <param name="LightColor">Цвет источника света</param>
        /// <param name="LightPower">Сила света (от 0 до 1)</param>
        /// <param name="LightType">Тип источника света</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int AddLight(ref Polygon3D[] polygon, ref Point3D[] vertex3D, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Point3D LightPoint, Point3D LightVector, float InternalCosinusAngle, float ExternalCosinusAngle, Color LightColor, float LightPower, LightTypes LightType = LightTypes.SpotSoft)
        {
            try
            {
                switch (LightType)
                {
                    case LightTypes.SpotSoft:
                        return AddSpotSoftLighting(ref polygon, ref vertex3D, ref activePolygonIndexes, activePolygonIndexesCount, LightPoint, LightVector, InternalCosinusAngle, ExternalCosinusAngle, LightColor, LightPower);
                    case LightTypes.Spot:
                        return AddSpotLighting(ref polygon, ref activePolygonIndexes, activePolygonIndexesCount, LightPoint, LightVector, InternalCosinusAngle, ExternalCosinusAngle, LightColor, LightPower);
                    default:
                        throw new Exception("Для указанного типа источника света нет обработки");
                }
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddLight(2)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Добавление тумана
        /// </summary>
        /// <param name="polygon">Список полигонов</param>
        /// <param name="activePolygonIndexes">Список индексов активных полигонов</param>
        /// <param name="activePolygonIndexesCount">Количество активных полигонов</param>
        /// <param name="FogColor">Цвет тумана</param>
        /// <param name="MinFogDistance">Минимальная дистанция, где виден туман</param>
        /// <param name="FullFogDistance">Максимальная дистанция, где видны полигоны</param>
        /// <returns></returns>
        public static int AddFog(ref Polygon3D[] polygon, ref int[] activePolygonIndexes, int activePolygonIndexesCount, Color FogColor, int MinFogDistance = 0, int FullFogDistance = 5000)
        {
            try
            {
                int R;
                int G;
                int B;
                int sideCount;
                int sideIdx;
                float dFog = 0;
                int deltaFog = FullFogDistance - MinFogDistance;

                

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    if (polygon[activePolygonIndexes[i]].Center.Z < MinFogDistance) continue;

                    sideCount = (polygon[activePolygonIndexes[i]].DoubleSided ? 2 : 1);

                    for (sideIdx = 0; sideIdx < sideCount; sideIdx++)
                    {
                        if (polygon[activePolygonIndexes[i]].Center.Z > FullFogDistance)
                        {
                            polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = FogColor;
                        }
                        else
                        {
                            dFog = (polygon[activePolygonIndexes[i]].Center.Z - MinFogDistance) / deltaFog;

                            R = (int)(polygon[activePolygonIndexes[i]].LightingColor[sideIdx].R * (1 - dFog) + FogColor.R * dFog);
                            if (R > 255) R = 255; else if (R < 0) R = 0;

                            G = (int)(polygon[activePolygonIndexes[i]].LightingColor[sideIdx].G * (1 - dFog) + FogColor.G * dFog);
                            if (G > 255) G = 255; else if (G < 0) G = 0;

                            B = (int)(polygon[activePolygonIndexes[i]].LightingColor[sideIdx].B * (1 - dFog) + FogColor.B * dFog);
                            if (B > 255) B = 255; else if (B < 0) B = 0;

                            polygon[activePolygonIndexes[i]].LightingColor[sideIdx] = Color.FromArgb(polygon[activePolygonIndexes[i]].color[sideIdx].A, R, G, B);
                        }
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddFog(1)", er.Message, -1);
                return -1;
            }
        }

        public static int AddFog(ref Color drawColor, float PositionZ, Color FogColor, int MinFogDistance, int FullFogDistance)
        {
            try
            {
                int R;
                int G;
                int B;
                float dFog;
                int deltaFog = FullFogDistance - MinFogDistance;

                if (PositionZ < MinFogDistance) return 0;

                if (PositionZ > FullFogDistance)
                {
                    drawColor = FogColor;
                }
                else
                {
                    dFog = (PositionZ - MinFogDistance) / deltaFog;

                    R = (int)(drawColor.R * (1 - dFog) + FogColor.R * dFog);
                    if (R > 255) R = 255; else if (R < 0) R = 0;

                    G = (int)(drawColor.G * (1 - dFog) + FogColor.G * dFog);
                    if (G > 255) G = 255; else if (G < 0) G = 0;

                    B = (int)(drawColor.B * (1 - dFog) + FogColor.B * dFog);
                    if (B > 255) B = 255; else if (B < 0) B = 0;

                    drawColor = Color.FromArgb(drawColor.A, R, G, B);
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "AddFog(2)", er.Message, -1);
                return -1;
            }
        }


        private static int PartSortPoint3D(ref int[] PointIndex, ref Point3D[] point, int startIDX = 0, int finishIDX = -1)
        {
            try
            {
                int j;
                int idx = startIDX;
                if (finishIDX == -1) finishIDX = point.Length;

                for (int i = startIDX; i < finishIDX - 1; i++)
                {
                    idx = i;
                    for (j = i + 1; j < finishIDX; j++)
                        if (point[PointIndex[idx]].Z < point[PointIndex[j]].Z) idx = j;
                    if (idx != i)
                    {
                        j = PointIndex[i];
                        PointIndex[i] = PointIndex[idx];
                        PointIndex[idx] = j;
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "PartSortPoint3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сортировка юнитов по удаленности от камеры
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int SortPoint3D(ref int[] PointIndex, int PointIndexLength, ref Point3D[] point)
        {
            // алгоритм быстрой сортировки с дроблением исходного массива
            try
            {
                // сортирую по группам

                int groupStep = (int)Math.Pow(PointIndexLength, 0.6);
                if (groupStep < 5) groupStep = 5;

                int idx1 = 0;
                int idx2 = 0;
                int groupCount = (int)Math.Floor((float)(PointIndexLength / groupStep)) + 1;
                int[] groupTop = new int[groupCount];
                int groupIndex = 0;
                while (idx1 < PointIndexLength)
                {
                    idx2 = idx1 + groupStep;
                    if (idx2 > PointIndexLength) idx2 = PointIndexLength;
                    groupTop[groupIndex++] = idx2 - idx1;
                    PartSortPoint3D(ref PointIndex, ref point, idx1, idx2);
                    idx1 += groupStep;
                }

                idx1 = -1;
                idx2 = -1;

                // собираю из групп
                if (PointIndexLength > groupStep)
                {
                    bool AllEmpty = true;
                    int[] StackBuff = new int[PointIndexLength];

                    { // ищу первый миниимальный столбик
                        idx1 = 0;
                        for (int i = 1; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            if (point[PointIndex[i * groupStep + groupTop[i] - 1]].Z < point[PointIndex[idx1 * groupStep + groupTop[idx1] - 1]].Z) idx1 = i;
                        }
                    }

                    groupIndex = 0;
                    bool setIdx = false;
                    while (true)
                    {
                        AllEmpty = true;
                        // ищу второй минимальный столбик
                        idx2 = -1;
                        for (int i = 0; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            AllEmpty = false;
                            if (i == idx1) continue;
                            setIdx = (idx2 == -1);
                            if (!setIdx) setIdx = (point[PointIndex[i * groupStep + groupTop[i] - 1]].Z < point[PointIndex[idx2 * groupStep + groupTop[idx2] - 1]].Z);
                            if (setIdx) idx2 = i;
                        }
                        if (AllEmpty) break;
                        if (idx2 == -1) idx2 = idx1;

                        // Выгружаю часть минимального столбика
                        int dx1 = idx1 * groupStep - 1;
                        int dx2 = idx2 * groupStep - 1;
                        while (groupTop[idx1] > 0)
                        {
                            if (point[PointIndex[dx1 + groupTop[idx1]]].Z <= point[PointIndex[dx2 + groupTop[idx2]]].Z)
                            {
                                StackBuff[groupIndex++] = PointIndex[dx1 + groupTop[idx1]];
                                groupTop[idx1]--;
                            }
                            else break;
                        }
                        idx1 = idx2;
                    }

                    // переписываем обратно в стек значения из временного буфера 
                    idx2 = PointIndexLength - 1;
                    for (idx1 = 0; idx1 < PointIndexLength;) PointIndex[idx1++] = StackBuff[idx2--];

                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "SortPoint3D", er.Message, -1);
                return -1;
            }
        }

        private static int PartSortUnit3DSize(ref int[] PointIndex, ref Unit3D[] unit, int startIDX = 0, int finishIDX = -1)
        {
            try
            {
                int j;
                int idx = startIDX;
                if (finishIDX == -1) finishIDX = unit.Length;

                for (int i = startIDX; i < finishIDX - 1; i++)
                {
                    idx = i;
                    for (j = i + 1; j < finishIDX; j++)
                        if (unit[PointIndex[idx]].ScreenRadius < unit[PointIndex[j]].ScreenRadius) idx = j;
                    if (idx != i)
                    {
                        j = PointIndex[i];
                        PointIndex[i] = PointIndex[idx];
                        PointIndex[idx] = j;
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "PartSortUnit3DSize", er.Message, -1);
                return -1;
            }
        }

        private static int PartSortUnit3D(ref int[] PointIndex, ref Unit3D[] unit, int startIDX = 0, int finishIDX = -1)
        {
            try
            {
                int j;
                int idx = startIDX;
                if (finishIDX == -1) finishIDX = unit.Length;

                for (int i = startIDX; i < finishIDX - 1; i++)
                {
                    idx = i;
                    for (j = i + 1; j < finishIDX; j++)
                        if (unit[PointIndex[idx]].ScreenPosition.Z < unit[PointIndex[j]].ScreenPosition.Z) idx = j;
                    if (idx != i)
                    {
                        j = PointIndex[i];
                        PointIndex[i] = PointIndex[idx];
                        PointIndex[idx] = j;
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "PartSortUnit3D", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сортировка юнитов по экранному размеру
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int SortUnit3DSize(ref int[] PointIndex, int PointIndexLength, Unit3D[] unit)
        {
            // алгоритм быстрой сортировки с дроблением исходного массива
            try
            {
                // сортирую по группам
                int groupStep = (int)Math.Pow(PointIndexLength, 0.6);
                if (groupStep < 5) groupStep = 5;

                int idx1 = 0;
                int idx2 = 0;
                int groupCount = (int)Math.Floor((float)(PointIndexLength / groupStep)) + 1;
                int[] groupTop = new int[groupCount];
                int groupIndex = 0;
                while (idx1 < PointIndexLength)
                {
                    idx2 = idx1 + groupStep;
                    if (idx2 > PointIndexLength) idx2 = PointIndexLength;
                    groupTop[groupIndex++] = idx2 - idx1;
                    PartSortUnit3DSize(ref PointIndex, ref unit, idx1, idx2);
                    idx1 += groupStep;
                }

                idx1 = -1;
                idx2 = -1;

                // собираю из групп
                if (PointIndexLength > groupStep)
                {
                    bool AllEmpty = true;
                    int[] StackBuff = new int[PointIndexLength];

                    { // ищу первый миниимальный столбик
                        idx1 = 0;
                        for (int i = 1; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            if (unit[PointIndex[i * groupStep + groupTop[i] - 1]].ScreenRadius < unit[PointIndex[idx1 * groupStep + groupTop[idx1] - 1]].ScreenRadius) idx1 = i;
                        }
                    }

                    groupIndex = 0;
                    bool setIdx = false;
                    while (true)
                    {
                        AllEmpty = true;
                        // ищу второй минимальный столбик
                        idx2 = -1;
                        for (int i = 0; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            AllEmpty = false;
                            if (i == idx1) continue;
                            setIdx = (idx2 == -1);
                            if (!setIdx) setIdx = (unit[PointIndex[i * groupStep + groupTop[i] - 1]].ScreenRadius < unit[PointIndex[idx2 * groupStep + groupTop[idx2] - 1]].ScreenRadius);
                            if (setIdx) idx2 = i;
                        }
                        if (AllEmpty) break;
                        if (idx2 == -1) idx2 = idx1;

                        // Выгружаю часть минимального столбика
                        int dx1 = idx1 * groupStep - 1;
                        int dx2 = idx2 * groupStep - 1;
                        while (groupTop[idx1] > 0)
                        {
                            if (unit[PointIndex[dx1 + groupTop[idx1]]].ScreenRadius <= unit[PointIndex[dx2 + groupTop[idx2]]].ScreenRadius)
                            {
                                StackBuff[groupIndex++] = PointIndex[dx1 + groupTop[idx1]];
                                groupTop[idx1]--;
                            }
                            else break;
                        }
                        idx1 = idx2;
                    }

                    // переписываем обратно в стек значения из временного буфера 
                    idx2 = PointIndexLength - 1;
                    for (idx1 = 0; idx1 < PointIndexLength;) PointIndex[idx1++] = StackBuff[idx2--];

                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "SortUnit3DSize", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сортировка юнитов по удаленности от камеры
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public static int SortUnit3D(ref int[] PointIndex, int PointIndexLength, Unit3D[] unit)
        {
            // алгоритм быстрой сортировки с дроблением исходного массива
            try
            {
                // сортирую по группам
                int groupStep = (int)Math.Pow(PointIndexLength, 0.6);
                if (groupStep < 5) groupStep = 5;

                int idx1 = 0;
                int idx2 = 0;
                int groupCount = (int)Math.Floor((float)(PointIndexLength / groupStep)) + 1;
                int[] groupTop = new int[groupCount];
                int groupIndex = 0;
                while (idx1 < PointIndexLength)
                {
                    idx2 = idx1 + groupStep;
                    if (idx2 > PointIndexLength) idx2 = PointIndexLength;
                    groupTop[groupIndex++] = idx2 - idx1;
                    PartSortUnit3D(ref PointIndex, ref unit, idx1, idx2);
                    idx1 += groupStep;
                }

                idx1 = -1;
                idx2 = -1;

                // собираю из групп
                if (PointIndexLength > groupStep)
                {
                    bool AllEmpty = true;
                    int[] StackBuff = new int[PointIndexLength];

                    { // ищу первый миниимальный столбик
                        idx1 = 0;
                        for (int i = 1; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            if (unit[PointIndex[i * groupStep + groupTop[i] - 1]].ScreenPosition.Z < unit[PointIndex[idx1 * groupStep + groupTop[idx1] - 1]].ScreenPosition.Z) idx1 = i;
                        }
                    }

                    groupIndex = 0;
                    bool setIdx = false;
                    while (true)
                    {
                        AllEmpty = true;
                        // ищу второй минимальный столбик
                        idx2 = -1;
                        for (int i = 0; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            AllEmpty = false;
                            if (i == idx1) continue;
                            setIdx = (idx2 == -1);
                            if (!setIdx) setIdx = (unit[PointIndex[i * groupStep + groupTop[i] - 1]].ScreenPosition.Z < unit[PointIndex[idx2 * groupStep + groupTop[idx2] - 1]].ScreenPosition.Z);
                            if (setIdx) idx2 = i;
                        }
                        if (AllEmpty) break;
                        if (idx2 == -1) idx2 = idx1;

                        // Выгружаю часть минимального столбика
                        int dx1 = idx1 * groupStep - 1;
                        int dx2 = idx2 * groupStep - 1;
                        while (groupTop[idx1] > 0)
                        {
                            if (unit[PointIndex[dx1 + groupTop[idx1]]].ScreenPosition.Z <= unit[PointIndex[dx2 + groupTop[idx2]]].ScreenPosition.Z)
                            {
                                StackBuff[groupIndex++] = PointIndex[dx1 + groupTop[idx1]];
                                groupTop[idx1]--;
                            }
                            else break;
                        }
                        idx1 = idx2;
                    }

                    // переписываем обратно в стек значения из временного буфера 
                    idx2 = PointIndexLength - 1;
                    for (idx1 = 0; idx1 < PointIndexLength;) PointIndex[idx1++] = StackBuff[idx2--];

                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Engine3D", "SortUnit3D", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion

    #region Object3D
    /// <summary>
    /// Базовый абстрактный класс для всех 3D объектов 
    /// </summary>
    public abstract class Object3D
    { }
    #endregion   

    #region Unit3D
    /// <summary>
    /// 3D-Юнит, содержащий информацию о позиции и размерах пространственного объекта, а так же информацию для построения модели
    /// </summary>
    public class Unit3D : Object3D
    {
        public UnitInfo Information;

        public UnitActions[] Actions;
        public UnitActions[] ActionsMemory;

        /// <summary>
        /// Первичная позиция
        /// </summary>
        protected Point3D PrimaryPosition;

        /// <summary>
        /// Установка начальных координат
        /// </summary>
        /// <param name="point"></param>
        public void SetPrimaryPosition(Point3D point)
        {
            CameraPosition.X = PrimaryPosition.X = point.X;
            CameraPosition.Y = PrimaryPosition.Y = point.Y;
            CameraPosition.Z = PrimaryPosition.Z = point.Z;
        }

        /// <summary>
        /// Установка начальных координат
        /// </summary>
        /// <param name="point"></param>
        public void SetPrimaryPosition(float X,float Y,float Z)
        {
            CameraPosition.X = PrimaryPosition.X = X;
            CameraPosition.Y = PrimaryPosition.Y = Y;
            CameraPosition.Z = PrimaryPosition.Z = Z;
        }

        /// <summary>
        /// Сброс камерных координат к начальным
        /// </summary>
        public void ResetCameraPosition()
        {
            CameraPosition.CopyFrom(PrimaryPosition);
        }

        /// <summary>
        /// Расчетная пространственная позиция
        /// </summary>
        public Point3D CameraPosition;

        /// <summary>
        /// Экранная позиция
        /// </summary>
        public Point3D ScreenPosition
        {
            get { return screenPosition; }
        }
        protected Point3D screenPosition; 

        /// <summary>
        /// Собственный радиус
        /// </summary>
        public float PrimaryRadius;

        /// <summary>
        /// Экранный радиус
        /// </summary>
        public float ScreenRadius
        {
            get { return screenRadius; }
        }
        protected float screenRadius;

        /// <summary>
        /// Квадрат расстояния от центра масс до центра камеры (до центра экрана)
        /// </summary>
        public float SquareDistanceFromCamera
        {
            get { return squareDistanceFromCamera; }
        }
        protected float squareDistanceFromCamera;

        /// <summary>
        /// Конструктор
        /// </summary>
        /// <returns></returns>
        public Unit3D()
        {
            PrimaryPosition = new Point3D(0, 0, 0);
            CameraPosition = new Point3D(0, 0, 0);
            screenPosition = new Point3D(0, 0, 0);
            PrimaryRadius = 0;
            screenRadius = 0;
        }

        /// <summary>
        /// Расчет расстояния до центра камеры
        /// </summary>
        /// <returns></returns>
        public int CalculateSquareDistanceFromCamera()
        {
            try
            {
                squareDistanceFromCamera = Engine3D.GetSquareDistance(PrimaryPosition);
                return ErrorLog.GetLastErrorNumber();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Unit3D", "CalculateSquareDistanceFromCamera", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование координат в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0 - успешно, не 0 - ошибка</returns>
        public int CalculateScreenValues(float K, int screenCenterX, int screenCenterY)
        {
            try
            {
                if (K < 1) K = 1;
                float z = K / ((CameraPosition.Z <= (float)0.001) ? (float)0.1 : CameraPosition.Z);
                screenRadius = PrimaryRadius * z;

                screenPosition.Z = CameraPosition.Z;
                screenPosition.X = CameraPosition.X * z + screenCenterX;
                screenPosition.Y = -screenCenterY - CameraPosition.Y * z;

                screenPosition.X = (screenPosition.X > int.MaxValue) ? int.MaxValue - 1 : (screenPosition.X < int.MinValue) ? int.MinValue + 1 : screenPosition.X;
                screenPosition.Y = (screenPosition.Y > int.MaxValue) ? int.MaxValue - 1 : (screenPosition.Y < int.MinValue) ? int.MinValue + 1 : screenPosition.Y;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Unit3D", "CalculateScreenValues", er.Message, -1);
                return -1;
            }
        }        
    }
    #endregion

    #region VertexModel3D
    public abstract class VertexModel3D : Object3D
    {
        /// <summary>
        /// Первичный буфер вершин модели 
        /// </summary>
        public Point3D[] MainVertex3D
        {
            get { return mainVertex3D;  }
        }
        protected Point3D[] mainVertex3D;

        /// <summary>
        /// Координаты вершин модели в мировой системе координат
        /// </summary>
        public Point3D[] CameraVertex3D;

        /// <summary>
        /// Экранные координаты вершин модели
        /// </summary>
        public Point3D[] ScreenVertex3D;

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public abstract ModelTypes ModelType();

        /// <summary>
        /// Сохранение главного буфера модели в внешний буфер
        /// </summary>
        /// <param name="buff">внешний буфер</param>
        /// <param name="withCreate">признак выделения памяти под внешний буфер</param>
        /// <returns></returns>
        public int SaveMainVertexBuffer(ref Point3D[] buff, bool withCreate = false)
        {
            try
            {
                return Engine3D.CopyPoint3D(mainVertex3D, ref buff, withCreate);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "SaveMainVertexBuffer", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка главного буфера модели из внешнего буфера. Размерность буфера должна соответствовать заданным при создании модели параметрам.
        /// </summary>
        /// <param name="buff">внешний буфер</param>
        /// <param name="withCreate">признак выделения памяти под главный буфер</param>
        /// <returns></returns>
        public int LoadMainVertexBuffer(Point3D[] buff, bool withCreate = false)
        {
            try
            {
                if (Engine3D.CopyPoint3D(buff, ref mainVertex3D, withCreate) != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "LoadMainVertexBuffer", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Выдает минимальные и максимальные значения координат по осям XYZ
        /// </summary>
        /// <param name="minValues">Минимальные значения</param>
        /// <param name="maxValues">Максимальные значения</param>
        /// <param name="source_mode">Тип источника: 0 - mainVertex3D; 1 - CameraVertex3D; 2 - ScreenVertex3D</param>
        /// <returns></returns>
        public int MinMaxVertexValue(ref Point3D minValues, ref Point3D maxValues, int source_mode)
        {
            try
            {
                Point3D[] buff=null;
                switch (source_mode)
                {
                    case 0:
                        buff = mainVertex3D;
                        break;
                    case 1:
                        buff = CameraVertex3D;
                        break;
                    case 2:
                        buff = ScreenVertex3D;
                        break;
                    default:
                        throw new Exception("Указан некорректный номер источника");
                }

                if (buff.Length == 0) return 1;

                minValues.CopyFrom(buff[0]);
                maxValues.CopyFrom(buff[0]);

                for (int i = 0; i < buff.Length; i++)
                {
                    if (minValues.X > buff[i].X) minValues.X = buff[i].X;
                    if (minValues.Y > buff[i].Y) minValues.Y = buff[i].Y;
                    if (minValues.Z > buff[i].Z) minValues.Z = buff[i].Z;

                    if (maxValues.X < buff[i].X) maxValues.X = buff[i].X;
                    if (maxValues.Y < buff[i].Y) maxValues.Y = buff[i].Y;
                    if (maxValues.Z < buff[i].Z) maxValues.Z = buff[i].Z;
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "MinMaxVertexValue", er.Message, -1);
                return -1;
            }
        }

        public int AverageVertexValue(ref Point3D averageValues, int source_mode)
        {
            try
            {
                Point3D[] buff = null;
                switch (source_mode)
                {
                    case 0:
                        buff = mainVertex3D;
                        break;
                    case 1:
                        buff = CameraVertex3D;
                        break;
                    case 2:
                        buff = ScreenVertex3D;
                        break;
                    default:
                        throw new Exception("Указан некорректный номер источника");
                }

                averageValues.CopyFrom();
                if (buff.Length == 0) return 1;

                for (int i = 0; i < buff.Length; i++)
                {
                    averageValues.X += buff[i].X;
                    averageValues.Y += buff[i].Y;
                    averageValues.Z += buff[i].Z;
                }
                averageValues.X /= buff.Length;
                averageValues.Y /= buff.Length;
                averageValues.Z /= buff.Length; 
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "AverageVertexValue", er.Message, -1);
                return -1;
            }
        }

        public int CenterVertexValue(ref Point3D centerValues, int source_mode)
        {
            try
            {
                Point3D minVal = new Point3D();
                Point3D maxVal = new Point3D();                

                if (MinMaxVertexValue(ref minVal, ref maxVal, source_mode) != 0) throw new Exception(ErrorLog.GetLastError());

                centerValues.CopyFrom((maxVal.X + minVal.X)/2, (maxVal.Y + minVal.Y)/2,(maxVal.Z + minVal.Z)/2);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "CenterVertexValue", er.Message, -1);
                return -1;
            }
        }

        public override string ToString()
        {
            return "VertexModel3D";
        }

        /// <summary>
        /// Расчет максимального радиуса модели по заданным исходным параметрам
        /// </summary>
        /// <returns></returns>
        public float GetModelMaxRadius()
        {
            try
            {
                float modelMaxRadius = 0;
                float r;
                for (int i = 0; i < mainVertex3D.Length; i++)
                {
                    r = Engine3D.GetDistance(mainVertex3D[i]);
                    if (r > modelMaxRadius) modelMaxRadius = r;
                }

                return modelMaxRadius;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "GetModelMaxRadius", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Создание буферов вершин
        /// </summary>
        /// <returns></returns>
        protected int CreateDeformVertexBuffers(bool newMemory)
        {
            try
            {
                if (newMemory)
                {
                    CameraVertex3D = new Point3D[mainVertex3D.Length];
                    ScreenVertex3D = new Point3D[mainVertex3D.Length];
                }
                for (int i = 0; i < mainVertex3D.Length; i++)
                {
                    Engine3D.CopyPoint3D(mainVertex3D[i], ref CameraVertex3D[i], newMemory);
                    Engine3D.CopyPoint3D(mainVertex3D[i], ref ScreenVertex3D[i], newMemory);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "CreateDeformVertexBuffers", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Сброс состояния вершинной деформированной модели к исходной
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ResetCameraModel()
        {
            try
            {
                for (int i = 0; i < mainVertex3D.Length; i++) CameraVertex3D[i].CopyFrom(mainVertex3D[i]);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "ResetCameraModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сохранение состояния вершинной деформированной модели в качестве исходной
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int SaveCameraModelToMain()
        {
            try
            {
                for (int i = 0; i < mainVertex3D.Length; i++)
                    CameraVertex3D[i].CopyTo(ref mainVertex3D[i]);
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "SaveCameraModelToMain", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Вращение объекта вокруг оси
        /// </summary>
        /// <param name="angle">Угол вращения (в радианах)</param>
        /// <param name="axis">Ось вращения</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int Rotate(float angle, Axis3D axis)
        {
            try
            {
                return Engine3D.RotatePoint3D(angle, axis, ref CameraVertex3D);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "Rotate", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Масштабирование модели
        /// </summary>
        /// <param name="zoom">Коэффициент масштабирования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int Zoom(float zoom)
        {
            try
            {
                return Engine3D.ZoomPoint3D(zoom, ref CameraVertex3D);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "Zoom", er.Message, -1);
                return -1;
            }
        }

        public int Zoom(float zoomX,float zoomY,float zoomZ)
        {
            try
            {
                return Engine3D.ZoomPoint3D(zoomX, zoomY, zoomZ, ref CameraVertex3D);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "Zoom(2)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Смещение координат 
        /// </summary>
        /// <param name="dX">Смещение по оси 0X</param>
        /// <param name="dY">Смещение по оси 0Y</param>
        /// <param name="dZ">Смещение по оси 0Z</param>
        /// <returns></returns>
        public int Move(float dX = 0, float dY = 0, float dZ = 0)
        {
            try
            {
                return Engine3D.MovePoint3D(dX, dY, dZ, ref CameraVertex3D);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VertexModel3D", "Move", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion
        
    #region VolumetricModel3D
    /// <summary>
    /// Модель
    /// </summary>
    public abstract class VolumetricModel3D : VertexModel3D
    {
        /// <summary>
        /// Для пользовательских данных, в файле не сохраняется
        /// </summary>        
        public object Tag = null;

        /// <summary>
        /// Признак выпуклости объекта (и его составляющих). Влияет на очередность вывода полигонов.
        /// </summary>
        public bool ClosedSurface = true;

        /// <summary>
        /// Полигоны модели
        /// </summary>
        public Polygon3D[] Polygon
        {
            get { return polygon; }
        }
        protected Polygon3D[] polygon;
        
        /// <summary>
        /// Список индексов активных упорядоченных полигонов. Задает фильтрацию и сортировку полигонов.
        /// </summary>
        public int[] ActivePolygonIndexes
        {
            get { return activePolygonIndexes; }
        }
        protected int[] activePolygonIndexes;        

        /// <summary>
        /// Текущая вершина стека полигонов
        /// </summary>
        public int ActivePolygonIndexesCount
        {
            get { return activePolygonIndexesCount; }
        }
        protected int activePolygonIndexesCount = 0;           

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "VolumetricModel3D";
        }        
        
        /// <summary>
        /// Перестроение модели по заданным ранее параметрам
        /// </summary>
        /// <returns>Возвращает 0 в случае успешного построения и не 0 в случае ошибки</returns>
        public abstract int RebuildModel(bool newMemory = false);

        /// <summary>
        /// Создание карты полигонов. Производится расстановка индексов вершин полигонов.
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public abstract int CreatePolygonMap();

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName"></param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName, ModelTypes model_type = ModelTypes.NONE)
        {
            try
            {
                if (model_type==ModelTypes.NONE) model_type = this.ModelType();

                StreamWriter file = new StreamWriter(FileName, false, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("* Файл 3D-модели");
                file.WriteLine("* Время создания файла: " + DateTime.Now.ToString());

                file.WriteLine("#ModelType#");
                file.WriteLine(((int)model_type).ToString());

                file.WriteLine("#Points#");
                file.WriteLine("* Построчно X,Y,Z");
                file.WriteLine(mainVertex3D.Length.ToString());
                for (int i = 0; i < mainVertex3D.Length; i++)
                    file.WriteLine(mainVertex3D[i].X.ToString("0.0000", culture) + "," +
                                   mainVertex3D[i].Y.ToString("0.0000", culture) + "," +
                                   mainVertex3D[i].Z.ToString("0.0000", culture));

                file.WriteLine("#Polygons#");
                file.WriteLine("* Построчно Индексы вершин (3шт),  цвет (ARGB) передний/задний, степень матовости передний/задний, признак двусторонности, тип закраски");
                file.WriteLine(polygon.Length.ToString());
                for (int i = 0; i < polygon.Length; i++)
                {
                    file.WriteLine(polygon[i].PointIndex[0].ToString() + "," +
                                   polygon[i].PointIndex[1].ToString() + "," +
                                   polygon[i].PointIndex[2].ToString() + ",   " +
                                   polygon[i].color[0].A.ToString() + "," +
                                   polygon[i].color[0].R.ToString() + "," +
                                   polygon[i].color[0].G.ToString() + "," +
                                   polygon[i].color[0].B.ToString() + ",   " +
                                   polygon[i].color[1].A.ToString() + "," +
                                   polygon[i].color[1].R.ToString() + "," +
                                   polygon[i].color[1].G.ToString() + "," +
                                   polygon[i].color[1].B.ToString() + ",   " +
                                   polygon[i].Matte[0].ToString("0.0000", culture) + "," +
                                   polygon[i].Matte[1].ToString("0.0000", culture) + ",   " +
                                   (polygon[i].DoubleSided ? "1" : "0") + ",   " +
                                   ((int)polygon[i].FillType).ToString()
                                   );
                }

                file.WriteLine("#Properties#");
                file.WriteLine("* Свойства модели");
                file.WriteLine("ClosedSurface=" + (ClosedSurface?"1":"0"));
                
                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        public virtual int OpenModelFromFile(string FileName, ref ModelTypes model_type)
        {
            try
            {
                int currentMode = -1;
                if (FileName.Trim() == "") throw new Exception("Задано пустое имя файла");
                string sLine = "";
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();

                        if (sLine == "") continue;

                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
                            case "#ModelType#":
                                currentMode = 0;
                                continue;                           
                            case "#Points#":
                                currentMode = 1;
                                continue;
                            case "#Polygons#":
                                currentMode = 2;
                                continue;
                            case "#Properties#":
                                currentMode = 3;
                                continue;                            
                            default:
                                if (sLine.Substring(0, 1) == "#")
                                {
                                    currentMode = -1;
                                    continue;
                                }
                                break;
                        }
                        if (currentMode == -1) continue;

                        switch (currentMode)
                        {
                            case 0:
                                model_type = (ModelTypes)int.Parse(sLine);
                                break;
                            case 1:
                                {
                                    mainVertex3D = new Point3D[int.Parse(sLine)];
                                    string[] separated;                                    
                                    for (int i = 0; i < mainVertex3D.Length; i++)
                                    {
                                        if (file.EndOfStream) throw new Exception("Неизвестный формата файла: неожиданный конец файла");
                                        sLine = file.ReadLine().Trim();
                                        if (sLine == "")
                                        {
                                            i--;
                                            continue;
                                        }
                                        if (sLine.Substring(0, 1) == "*")
                                        {
                                            i--;
                                            continue;
                                        }
                                        separated = sLine.Split(',');
                                        if (separated.Length != 3) throw new Exception("Неизвестный формата файла: неверное количество позиций в строке");
                                        mainVertex3D[i] = new Point3D(float.Parse(separated[0].Trim(), culture), float.Parse(separated[1].Trim(), culture), float.Parse(separated[2].Trim(), culture));
                                    }
                                    break;
                                }

                            case 2:
                                {
                                    polygon = new Polygon3D[int.Parse(sLine)];
                                    string[] separated;
                                    for (int i = 0; i < polygon.Length; i++)
                                    {
                                        if (file.EndOfStream) throw new Exception("Неизвестный формата файла: неожиданный конец файла");
                                        sLine = file.ReadLine().Trim();
                                        if (sLine == "")
                                        {
                                            i--;
                                            continue;
                                        }
                                        if (sLine.Substring(0, 1) == "*")
                                        {
                                            i--;
                                            continue;
                                        }
                                        separated = sLine.Split(',');
                                        if (separated.Length != 15) throw new Exception("Неизвестный формата файла: неверное количество позиций в строке");

                                        polygon[i] = new Polygon3D(int.Parse(separated[0].Trim()), int.Parse(separated[1].Trim()), int.Parse(separated[2].Trim()));

                                        polygon[i].color[0] = Color.FromArgb(int.Parse(separated[3].Trim()),
                                                                             int.Parse(separated[4].Trim()),
                                                                             int.Parse(separated[5].Trim()),
                                                                             int.Parse(separated[6].Trim()));

                                        polygon[i].color[1] = Color.FromArgb(int.Parse(separated[7].Trim()),
                                                                             int.Parse(separated[8].Trim()),
                                                                             int.Parse(separated[9].Trim()),
                                                                             int.Parse(separated[10].Trim()));

                                        polygon[i].Matte[0] = float.Parse(separated[11].Trim(), culture);
                                        polygon[i].Matte[1] = float.Parse(separated[12].Trim(), culture);

                                        polygon[i].DoubleSided = (separated[13].Trim() == "1");
                                        polygon[i].FillType = (PolygonFillType)int.Parse(separated[14].Trim());
                                    }
                                    break;
                                }
                            case 3:
                                int semIdx=sLine.IndexOf("=");
                                if (semIdx>0)
                                {
                                    string PropertyName = sLine.Substring(0, semIdx).Trim();
                                    string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();

                                    if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: "+sLine);

                                    switch(PropertyName)
                                    {
                                        case "ClosedSurface":
                                            ClosedSurface = (PropertyValue == "1");
                                            break;
                                    }
                                }
                                break;
                        }
                    }
                }
                finally
                {
                    file.Close();
                    file.Dispose();
                }
                CreateDeformVertexBuffers(true);
                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }

        public int RebuildPolygonMap()
        {
            try
            {
                if (polygon != null)
                    if (polygon.Length > 0)
                    {
                        Polygon3D tmp = new Polygon3D(polygon[0].PointIndex[0], polygon[0].PointIndex[1], polygon[0].PointIndex[2]);
                        polygon[0].CopyPropertiesTo(tmp);
                        CreatePolygonMap();
                        this.SetDoubleSided(tmp.DoubleSided);
                        this.SetColor(tmp.color[0], 0, -1, PolygonSides.FrontSide);
                        this.SetColor(tmp.color[1], 0, -1, PolygonSides.RearSide);
                        this.SetMatte(tmp.Matte[0], 0, -1, PolygonSides.FrontSide);
                        this.SetMatte(tmp.Matte[1], 0, -1, PolygonSides.RearSide);
                        this.SetFillType(tmp.FillType);
                    }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "RebuildPolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Создание экземпляра на основе шаблона
        /// </summary>
        /// <param name="Template">Шаблон</param>
        public int CreateModelFrom(VolumetricModel3D Template)
        {
            try
            {
                mainVertex3D = new Point3D[Template.MainVertex3D.Length];
                Engine3D.CopyPoint3D(Template.MainVertex3D, ref mainVertex3D, true);
                CreateDeformVertexBuffers(true);
                polygon = new Polygon3D[Template.Polygon.Length];
                for (int i = 0; i < polygon.Length; i++)
                {
                    polygon[i] = new Polygon3D(Template.Polygon[i].PointIndex[0], Template.Polygon[i].PointIndex[1], Template.Polygon[i].PointIndex[2]);
                    Template.polygon[i].CopyPropertiesTo(polygon[i]);                        
                }
                ClosedSurface = Template.ClosedSurface;
                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CreateModelFrom", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сброс стека полигонов
        /// </summary>
        /// <returns></returns>
        public int ResetActivePolygonIndexes()
        {
            try
            {                                
                activePolygonIndexesCount = activePolygonIndexes.Length;
                for (int i = 0; i < activePolygonIndexesCount; i++) activePolygonIndexes[i] = i;
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ResetActivePolygonIndexes", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Последовательный расчет нормалей для всех полигонов модели
        /// </summary>
        /// <param name="zAxis">Расчет только Z-нормали</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int CalculatePolygonNormals(bool zAxis = false)
        {
            if (Environment.ProcessorCount < 2) 
                return CalculatePolygonNormalsSeries(zAxis);
            else 
                return CalculatePolygonNormalsParallel(zAxis);
        }

        public int CalculatePolygonNormalsSeries(bool zAxis=false)
        {
            try
            {
                Point3D Vector12 = new Point3D();
                Point3D Vector32 = new Point3D();

                for (int i = 0; i < activePolygonIndexesCount; i++)
                {

                    if (!zAxis)
                    {
                        Vector12.X = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                        Vector12.Y = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                        Vector12.Z = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z;
                        Vector32.X = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                        Vector32.Y = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                        Vector32.Z = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z;
                        if (Engine3D.BuiltNormal(Vector12, Vector32, ref polygon[activePolygonIndexes[i]].Normal) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    }
                    else
                    {
                        Vector12.X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                        Vector12.Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                        Vector32.X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                        Vector32.Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                        if (Engine3D.BuiltNormal(Vector12, Vector32, ref polygon[activePolygonIndexes[i]].NormalZ) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    }
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CalculatePolygonNormalsSeries", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Параллельный расчет нормалей для всех полигонов модели
        /// </summary>
        /// <param name="zAxis">Расчет только Z-нормали</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int CalculatePolygonNormalsParallel(bool zAxis = false)
        {
            try
            {
                int partSize = activePolygonIndexesCount / Environment.ProcessorCount;
                int lastPartSize = activePolygonIndexesCount % Environment.ProcessorCount;
                int partCount = Environment.ProcessorCount + ((lastPartSize > 0) ? 1 : 0);
                if (lastPartSize == 0) lastPartSize = partSize;

                Parallel.For(0, partCount, (int partIndex) =>
                  {
                      Point3D Vector12 = new Point3D();
                      Point3D Vector32 = new Point3D();

                      for (int i = partIndex * partSize; i < partIndex  * partSize + ((partIndex<(partCount-1))?partSize:lastPartSize); i++)
                      {

                          if (!zAxis)
                          {
                              Vector12.X = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                              Vector12.Y = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                              Vector12.Z = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z;
                              Vector32.X = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                              Vector32.Y = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                              Vector32.Z = CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z - CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z;
                              if (Engine3D.BuiltNormal(Vector12, Vector32, ref polygon[activePolygonIndexes[i]].Normal) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                          }
                          else
                          {
                              Vector12.X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                              Vector12.Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                              Vector32.X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                              Vector32.Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y - ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;
                              if (Engine3D.BuiltNormal(Vector12, Vector32, ref polygon[activePolygonIndexes[i]].NormalZ) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                          }
                      }
                  });
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CalculatePolygonNormalsParallel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Частичная сортировка стека полигонов по Z-координате
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int PartSortActivePolygonIndexes(int start=0, int finish=-1)
        {
            try
            {
                int maxIdx = start;
                int buff;
                int j;
                if (finish == -1) finish = activePolygonIndexesCount;
                for (int i = start; i < finish - 1; i++)
                {                    
                    maxIdx = i;
                    for (j = i + 1; j < finish; j++)
                        if (polygon[activePolygonIndexes[maxIdx]].Center.Z < polygon[activePolygonIndexes[j]].Center.Z) maxIdx = j;                        

                    if (maxIdx != i)
                    {
                        buff = activePolygonIndexes[i];
                        activePolygonIndexes[i] = activePolygonIndexes[maxIdx];
                        activePolygonIndexes[maxIdx] = buff;
                    }
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "PartSortActivePolygonIndexes", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Сортировка стека полигонов по Z-координате
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int SortActivePolygonIndexes()
        {
            if (Environment.ProcessorCount < 2)
                return SortActivePolygonIndexesSerial();
            else
                return SortActivePolygonIndexesParallel();
        }

        /// <summary>
        /// Параллельная сортировка стека полигонов по Z-координате
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int SortActivePolygonIndexesParallel()
        {
            // алгоритм быстрой сортировки с дроблением исходного массива
            try
            {
                // сортирую по группам
                int groupStep = (int)Math.Pow(activePolygonIndexesCount, 0.6);
                if (groupStep < 5) groupStep = 5;

                int idx1 = 0;
                int idx2 = 0;
                int groupCount = (int)Math.Floor((float)(activePolygonIndexesCount / groupStep)) + 1;
                int[] groupTop = new int[groupCount];
                int groupIndex = 0;
                int[][] indexes = new int[groupCount][];
                while (idx1 < activePolygonIndexesCount)
                {
                    idx2 = idx1 + groupStep;
                    if (idx2 > activePolygonIndexesCount) idx2 = activePolygonIndexesCount;
                    indexes[groupIndex] = new int[2] { idx1, idx2 };
                    groupTop[groupIndex++] = idx2 - idx1;
                    idx1 += groupStep;
                }
                Parallel.For(0, groupIndex, (int kIndex) =>
                {
                    PartSortActivePolygonIndexes(indexes[kIndex][0], indexes[kIndex][1]);
                });

                idx1 = -1;
                idx2 = -1;

                // собираю из групп
                if (activePolygonIndexesCount > groupStep)
                {
                    bool AllEmpty = true;
                    int[] StackBuff = new int[activePolygonIndexesCount];

                    { // ищу первый миниимальный столбик
                        idx1 = 0;
                        for (int i = 1; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            if (polygon[activePolygonIndexes[i * groupStep + groupTop[i] - 1]].Center.Z < polygon[activePolygonIndexes[idx1 * groupStep + groupTop[idx1] - 1]].Center.Z) idx1 = i;
                        }
                    }

                    groupIndex = 0;
                    bool setIdx = false;
                    while (true)
                    {
                        AllEmpty = true;
                        // ищу второй минимальный столбик
                        idx2 = -1;
                        for (int i = 0; i < groupCount; i++)
                        {
                            if (groupTop[i] <= 0) continue;
                            AllEmpty = false;
                            if (i == idx1) continue;
                            setIdx = (idx2 == -1);
                            if (!setIdx) setIdx = (polygon[activePolygonIndexes[i * groupStep + groupTop[i] - 1]].Center.Z < polygon[activePolygonIndexes[idx2 * groupStep + groupTop[idx2] - 1]].Center.Z);
                            if (setIdx) idx2 = i;
                        }
                        if (AllEmpty) break;
                        if (idx2 == -1) idx2 = idx1;

                        // Выгружаю часть минимального столбика
                        int dx1 = idx1 * groupStep - 1;
                        int dx2 = idx2 * groupStep - 1;
                        while (groupTop[idx1] > 0)
                        {
                            if (polygon[activePolygonIndexes[dx1 + groupTop[idx1]]].Center.Z <= polygon[activePolygonIndexes[dx2 + groupTop[idx2]]].Center.Z)
                            {
                                StackBuff[groupIndex++] = activePolygonIndexes[dx1 + groupTop[idx1]];
                                groupTop[idx1]--;
                            }
                            else break;
                        }
                        idx1 = idx2;
                    }

                    // переписываем обратно в стек значения из временного буфера 
                    idx2 = activePolygonIndexesCount - 1;
                    for (idx1 = 0; idx1 < activePolygonIndexesCount;) activePolygonIndexes[idx1++] = StackBuff[idx2--];

                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SortActivePolygonIndexesParallel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Последовательная сортировка стека полигонов по Z-координате
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int SortActivePolygonIndexesSerial()
        {
            // алгоритм быстрой сортировки с дроблением исходного массива
            try
            {
                // сортирую по группам
               int groupStep = (int)Math.Pow(activePolygonIndexesCount,0.6);
                if (groupStep < 5) groupStep = 5;

                int idx1 = 0;
                int idx2 = 0;
                int groupCount = (int)Math.Floor((float)(activePolygonIndexesCount / groupStep))+1;
                int[] groupTop = new int[groupCount];
                int groupIndex = 0;
                while (idx1<activePolygonIndexesCount)
                {
                    idx2 = idx1 + groupStep;
                    if (idx2 > activePolygonIndexesCount) idx2 = activePolygonIndexesCount;
                    groupTop[groupIndex++] = idx2 - idx1;
                    PartSortActivePolygonIndexes(idx1, idx2);
                    idx1 += groupStep;
                }
                
                idx1 = -1;
                idx2 = -1;

                // собираю из групп
                if (activePolygonIndexesCount > groupStep)
                {
                    bool AllEmpty = true;
                    int[] StackBuff = new int[activePolygonIndexesCount];

                    { // ищу первый миниимальный столбик
                        idx1 = 0;
                        for (int i = 1; i < groupCount; i++)
                        {                            
                            if (groupTop[i] <= 0) continue;
                            if (polygon[activePolygonIndexes[i * groupStep + groupTop[i] - 1]].Center.Z< polygon[activePolygonIndexes[idx1 * groupStep + groupTop[idx1] - 1]].Center.Z) idx1 = i;
                        }
                    }

                    groupIndex = 0;
                    bool setIdx = false;
                    while (true)
                    {
                        AllEmpty = true;
                        // ищу второй минимальный столбик
                        idx2 = -1;
                        for (int i=0;i<groupCount;i++)
                        {                            
                            if (groupTop[i] <= 0) continue;
                            AllEmpty = false;
                            if (i == idx1) continue;
                            setIdx = (idx2 == -1);
                            if (!setIdx) setIdx = (polygon[activePolygonIndexes[i * groupStep + groupTop[i] - 1]].Center.Z<polygon[activePolygonIndexes[idx2 * groupStep + groupTop[idx2] - 1]].Center.Z);
                            if (setIdx) idx2 = i;
                        }
                        if (AllEmpty) break;
                        if (idx2 == -1) idx2 = idx1;

                        // Выгружаю часть минимального столбика
                        int dx1 = idx1 * groupStep-1;
                        int dx2 = idx2 * groupStep-1;
                        while (groupTop[idx1] > 0)
                        {                            
                            if (polygon[activePolygonIndexes[dx1 + groupTop[idx1]]].Center.Z <= polygon[activePolygonIndexes[dx2 + groupTop[idx2]]].Center.Z)
                            {
                                StackBuff[groupIndex++] = activePolygonIndexes[dx1 + groupTop[idx1]];
                                groupTop[idx1]--;
                            }
                            else break;
                        }
                        idx1 = idx2;
                    }

                    // переписываем обратно в стек значения из временного буфера 
                    idx2 = activePolygonIndexesCount - 1;
                    for (idx1 = 0; idx1 < activePolygonIndexesCount;) activePolygonIndexes[idx1++] = StackBuff[idx2--];

                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SortActivePolygonIndexesSerial", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Фильтрация стека полигонов: скрытие отвернутых от сцены односторонних полигонов
        /// </summary>
        /// <param name="side">Скрываемая сторона. AllSides скрывает все стороны, Auto - не скрывает ничего.</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int FilterPolygonDirectedAwayFromScene(PolygonSides side=PolygonSides.RearSide)
        {
            try
            {
                int i = 0;
                switch (side)
                {
                    case PolygonSides.AllSides:
                        activePolygonIndexesCount = 0;
                        break;
                    case PolygonSides.Auto:
                        break;
                    case PolygonSides.RearSide:
                        while (i < activePolygonIndexesCount)
                        {
                            if (!polygon[activePolygonIndexes[i]].DoubleSided)
                            {
                                if (polygon[activePolygonIndexes[i]].NormalZ >= 0)
                                    activePolygonIndexes[i--] = activePolygonIndexes[--activePolygonIndexesCount];
                            }
                            i++;
                        }
                        break;
                    case PolygonSides.FrontSide:
                        while (i < activePolygonIndexesCount)
                        {
                            if (!polygon[activePolygonIndexes[i]].DoubleSided)
                            {
                                if (polygon[activePolygonIndexes[i]].NormalZ <= 0)
                                    activePolygonIndexes[i--] = activePolygonIndexes[--activePolygonIndexesCount];
                            }
                            i++;
                        }
                        break;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "FilterPolygonDirectedAwayFromScene", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Фильтрация стека полигонов: скрытие расположенных слишкоми близко к наблюдателю полигонов (или за пределами тумана)
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int FilterPolygonByZPos(int MinZ=0, int MaxZ=-1)
        {
            try
            {
                int i = 0;

                while (i < activePolygonIndexesCount)
                {
                    if ((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z <= MinZ) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z <= MinZ) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z <= MinZ))
                        activePolygonIndexes[i--] = activePolygonIndexes[--activePolygonIndexesCount];
                    i++;
                }

                if (MaxZ > MinZ)
                {
                    i = 0;
                    while (i < activePolygonIndexesCount)
                    {
                        if ((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z >= MaxZ) &
                            (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z >= MaxZ) &
                            (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z >= MaxZ))
                            activePolygonIndexes[i--] = activePolygonIndexes[--activePolygonIndexesCount];
                        i++;
                    }
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "FilterPolygonByZPos", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Фильтрация стека полигонов: скрытие расположенных слишкоми близко к наблюдателю полигонов
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int FilterPolygonByXYPos(Point ScreenSize)
        {
            try
            {
                int i = 0;

                while (i < activePolygonIndexesCount)
                {
                    if (
                        (((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X <= 0) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X <= 0) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X <= 0)))
                        |
                        (((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y <= 0) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y <= 0) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y <= 0)))
                        |
                        (((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X >= ScreenSize.X) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X >= ScreenSize.X) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X >= ScreenSize.X)))
                        |
                        (((ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y >= ScreenSize.Y) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y >= ScreenSize.Y) &
                        (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y >= ScreenSize.Y)))
                        )
                        activePolygonIndexes[i--] = activePolygonIndexes[--activePolygonIndexesCount];
                    i++;
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "FilterPolygonByXYPos", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет центра координат для каждого полигона модели
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int CalculatePolygonCenters()
        {
            try
            {
                Parallel.For(0, activePolygonIndexesCount, (int i) =>
                  {
                      polygon[activePolygonIndexes[i]].Center.X = (CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X) / 3;
                      polygon[activePolygonIndexes[i]].Center.Y = (CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y) / 3;
                      polygon[activePolygonIndexes[i]].Center.Z = (CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z + CameraVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z) / 3;
                  });
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CalculatePolygonCenters", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет экранного центра координат для каждого полигона модели
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int CalculatePolygonScreenCenters()
        {
            try
            {
                for (int i = 0; i < activePolygonIndexesCount; i++)
                {
                    polygon[activePolygonIndexes[i]].Center.X = (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X) / 3;
                    polygon[activePolygonIndexes[i]].Center.Y = (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y) / 3;
                    polygon[activePolygonIndexes[i]].Center.Z = (ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Z + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Z + ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Z) / 3;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CalculatePolygonScreenCenters", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Установка цвета полигонов модели
        /// </summary>
        /// <param name="color">Цвет</param>
        /// <param name="firstPolygonNumber">Первый полигон</param>
        /// <param name="lastPolygonNumber">Последний полигон</param>
        /// <param name="side">Сторона полигона</param>
        /// <returns></returns>
        public int SetColor(Color color, int firstPolygonNumber=0, int lastPolygonNumber = -1,PolygonSides side = PolygonSides.AllSides)
        {
            try
            {
                if (side == PolygonSides.Auto) side = PolygonSides.AllSides;
                if (lastPolygonNumber >= polygon.Length) lastPolygonNumber = -1;
                if (lastPolygonNumber == -1) lastPolygonNumber = polygon.Length - 1;
                for (int i = firstPolygonNumber; i <= lastPolygonNumber; i++)
                {
                    if (side == PolygonSides.AllSides)
                    {
                        polygon[i].color[0] = polygon[i].color[1] = color;
                    }
                    else polygon[i].color[(int)(side)-1] = color;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SetColor", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Установка степени матовости полигонов модели
        /// </summary>
        /// <param name="matte">Степень матовости (0.1)</param>
        /// <param name="firstPolygonNumber">Первый полигон</param>
        /// <param name="lastPolygonNumber">Последний полигон</param>
        /// <param name="side">Сторона полигона</param>
        /// <returns></returns>
        public int SetMatte(float matte= (float)0.5, int firstPolygonNumber = 0, int lastPolygonNumber = -1, PolygonSides side = PolygonSides.AllSides)
        {
            try
            {
                if (side == PolygonSides.Auto) side = PolygonSides.AllSides;
                if (lastPolygonNumber >= polygon.Length) lastPolygonNumber = -1;
                if (lastPolygonNumber == -1) lastPolygonNumber = polygon.Length - 1;
                if (matte < 0) matte = 0;
                if (matte > 1) matte = 1;
                for (int i = firstPolygonNumber; i <= lastPolygonNumber; i++)
                {
                    if (side == PolygonSides.AllSides)
                    {
                        polygon[i].Matte[0] = polygon[i].Matte[1] = matte;
                    }
                    else polygon[i].Matte[(int)(side) - 1] = matte;
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SetMatte", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Установка признака двусторонности полигонов модели
        /// </summary>
        /// <param name="doubleSided">Признак двусторонности</param>
        /// <param name="firstPolygonNumber">Первый полигон</param>
        /// <param name="lastPolygonNumber">Последний полигон</param>
        /// <returns></returns>
        public int SetDoubleSided(bool doubleSided=true, int firstPolygonNumber = 0, int lastPolygonNumber = -1)
        {
            try
            {
                if (lastPolygonNumber >= polygon.Length) lastPolygonNumber = -1;
                if (lastPolygonNumber == -1) lastPolygonNumber = polygon.Length - 1;
                for (int i = firstPolygonNumber; i <= lastPolygonNumber; i++)
                    polygon[i].DoubleSided = doubleSided;
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SetDoubleSided", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Установка типа заливки полигонов модели
        /// </summary>
        /// <param name="fill_type">Тип заливки</param>
        /// <param name="firstPolygonNumber">Первый полигон</param>
        /// <param name="lastPolygonNumber">Последний полигон</param>
        /// <returns></returns>
        public int SetFillType(PolygonFillType fill_type, int firstPolygonNumber = 0, int lastPolygonNumber = -1)
        {
            try
            {
                if (lastPolygonNumber >= polygon.Length) lastPolygonNumber = -1;
                if (lastPolygonNumber == -1) lastPolygonNumber = polygon.Length - 1;
                for (int i = firstPolygonNumber; i <= lastPolygonNumber; i++)
                    polygon[i].FillType = fill_type;
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "SetFillType", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Сброс расчетного освещения (для обоих сторон полигонов)
        /// </summary>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ResetLighting()
        {
            try
            {
                Parallel.For(0, activePolygonIndexesCount, (int i) =>
                 {
                     polygon[activePolygonIndexes[i]].LightingColor[0] = Color.Black;//polygon[activePolygonIndexes[i]].color[0];
                    polygon[activePolygonIndexes[i]].LightingColor[1] = Color.Black; //polygon[activePolygonIndexes[i]].color[1];
                });
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ResetLighting", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет освещения сцены от указанного источника света
        /// </summary>
        /// <param name="LightVector">Вектор направления на источник света</param>
        /// <param name="LightColor">Цвет источника света</param>
        /// <param name="LightPower">Сила источника света</param>
        /// <param name="LightType">Тип источника света</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int AddLight(Point3D LightVector, Color LightColor, float LightPower, LightTypes LightType= LightTypes.Directional)
        {
            try
            {
                return Engine3D.AddLight(ref polygon, ref CameraVertex3D, ref activePolygonIndexes, activePolygonIndexesCount, LightVector, LightColor, LightPower, LightType);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "AddLight(1)", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Расчет освещения сцены от указанного источника света
        /// </summary>
        /// <param name="LightPoint">Положение источника света</param>
        /// <param name="LightVector">Вектор направления на источник света</param>
        /// <param name="InternalConAngle">Косинус угла внутренноего конуса источника света</param>
        /// <param name="ExternalConAngle">Косинус угла внешнего конуса источника света</param>
        /// <param name="LightColor">Цвет источника света</param>
        /// <param name="LightPower">Сила источника света</param>
        /// <param name="LightType">Тип источника света</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int AddLight(Point3D LightPoint, Point3D LightVector, float InternalCosinusAngle, float ExternalCosinusAngle, float LightPower, Color LightColor, LightTypes LightType=LightTypes.SpotSoft)
        {
            try
            {
                return Engine3D.AddLight(ref polygon, ref CameraVertex3D, ref activePolygonIndexes, activePolygonIndexesCount, LightPoint, LightVector, InternalCosinusAngle, ExternalCosinusAngle, LightColor, LightPower, LightType);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "AddLight(2)", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Добавление тумана
        /// </summary>
        /// <param name="FogColor">Цвет тумана</param>
        /// <param name="MinFogDistance">Минимальная дистанция, на которой начинает ощущаться эффект тумана</param>
        /// <param name="FullFogDistance">Дистанция, за которой туман скрывает объекты полностью</param>
        /// <returns></returns>
        public int AddFog(Color FogColor, int MinFogDistance=0, int FullFogDistance=5000)
        {
            try
            {
                return Engine3D.AddFog(ref polygon, ref activePolygonIndexes, activePolygonIndexesCount, FogColor, MinFogDistance, FullFogDistance);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "AddFog", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Отрисовка полигонной модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowPolygonModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.Auto)
        {
            try
            {
                //if ((!DoubleSided) & (drawingSide == PolygonSides.RearSide)) return 0;
                    
                if (drawingSide == PolygonSides.Auto)
                {
                    if (ShowPolygonModel(surface, PolygonSides.RearSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    if (ShowPolygonModel(surface, PolygonSides.FrontSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                }
                else
                {
                    Brush brush = new SolidBrush(Color.White);
                    PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };
                    int sideIdx;

                    int PolygonPartSize = ActivePolygonIndexesCount / surface.BufferCount;
                    Parallel.For(0, surface.BufferCount, (int partIndex) =>
                    {
                        for (int i = partIndex * PolygonPartSize; (i < ActivePolygonIndexesCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                            if ((polygon[activePolygonIndexes[i]].DoubleSided) | (drawingSide != PolygonSides.RearSide))
                            {
                                sideIdx = (polygon[activePolygonIndexes[i]].NormalZ <= 0) ? 0 : 1;

                                if (drawingSide != PolygonSides.AllSides)
                                {
                                    if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                    if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                }

                                if (Engine3D.BuiltStaticPolygon(ref ScreenVertex3D, ref polygon[activePolygonIndexes[i]], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());


                                ((SolidBrush)brush).Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];

                                surface.FillPolygon(brush, poly,partIndex);
                            }
                    });
                    brush.Dispose();
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ShowPolygonModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Отрисовка полигонно-паутинной модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowPolygonWideModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.Auto)
        {
            try
            {                
                if (drawingSide == PolygonSides.Auto)
                {
                    if (ShowPolygonWideModel(surface, PolygonSides.RearSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    if (ShowPolygonWideModel(surface, PolygonSides.FrontSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                }
                else
                {
                    Pen pen = new Pen(Color.White);
                    PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };
                    int sideIdx;

                    int PolygonPartSize = ActivePolygonIndexesCount / surface.BufferCount;
                    Parallel.For(0, surface.BufferCount, (int partIndex) =>
                    {
                        for (int i = partIndex * PolygonPartSize; (i < ActivePolygonIndexesCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                            if ((polygon[activePolygonIndexes[i]].DoubleSided) | (drawingSide != PolygonSides.RearSide))
                            {
                                sideIdx = (polygon[activePolygonIndexes[i]].NormalZ <= 0) ? 0 : 1;

                                if (drawingSide != PolygonSides.AllSides)
                                {
                                    if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                    if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                }

                                if (Engine3D.BuiltStaticPolygon(ref ScreenVertex3D, ref polygon[activePolygonIndexes[i]], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());


                                pen.Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];

                                surface.DrawPolygon(pen, poly,partIndex);
                            }
                    });
                    pen.Dispose();
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ShowPolygonWideModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Отрисовка только ячеек клеток в виде сетки указанной толщиной
        /// </summary>
        /// <param name="lineWidth">Толщина линий</param>
        /// <param name="SpecialColor">Особый цвет отрисовки</param>        
        /// <param name="UseSpecialColor">Признак использования особого цвета отрисовки</param>                
        /// <returns></returns>
        public int ShowWideOnlyCellModel(IDrawingSurface surface, Color SpecialColor, PolygonSides drawingSide = PolygonSides.Auto, int lineWidth = 1, bool UseSpecialColor = false)
        {
            try
            {
                if (drawingSide == PolygonSides.Auto)
                {
                    if (ShowWideOnlyCellModel(surface, SpecialColor, PolygonSides.RearSide, lineWidth, UseSpecialColor) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    if (ShowWideOnlyCellModel(surface, SpecialColor, PolygonSides.FrontSide, lineWidth, UseSpecialColor) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                }
                else
                {
                    if (!UseSpecialColor) SpecialColor = Color.White;
                    Pen pen = new Pen(SpecialColor, lineWidth);
                    PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };
                    int sideIdx;

                    int PolygonPartSize = ActivePolygonIndexesCount / surface.BufferCount;
                    Parallel.For(0, surface.BufferCount, (int partIndex) =>
                    {
                        for (int i = partIndex * PolygonPartSize; (i < ActivePolygonIndexesCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                            if ((polygon[activePolygonIndexes[i]].DoubleSided) | (drawingSide != PolygonSides.RearSide))
                            {
                                sideIdx = (polygon[activePolygonIndexes[i]].NormalZ <= 0) ? 0 : 1;

                                if (drawingSide != PolygonSides.AllSides)
                                {
                                    if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                    if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                }

                                if (!UseSpecialColor) pen.Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];

                                poly[0].X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].X;
                                poly[0].Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[2]].Y;

                                poly[1].X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].X;
                                poly[1].Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[0]].Y;

                                poly[2].X = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].X;
                                poly[2].Y = ScreenVertex3D[polygon[activePolygonIndexes[i]].PointIndex[1]].Y;

                                surface.DrawLines(pen, poly,partIndex);
                            }
                    });
                    pen.Dispose();
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ShowWideOnlyCellModel", er.Message, -1);
                return -1;
            }
        }
        

        /// <summary>
        /// Отрисовка модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.Auto)
        {
            try
            {
                if (drawingSide == PolygonSides.Auto)
                {
                    if (ShowModel(surface, PolygonSides.RearSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    if (ShowModel(surface, PolygonSides.FrontSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                }
                else
                {
                    Pen pen = new Pen(Color.White);
                    SolidBrush brush = new SolidBrush(Color.White);
                    PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };
                    int sideIdx;

                    int PolygonPartSize = ActivePolygonIndexesCount / surface.BufferCount;
                    Parallel.For(0, surface.BufferCount, (int partIndex) =>
                    {
                        for (int i = partIndex * PolygonPartSize; (i < ActivePolygonIndexesCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                            if ((polygon[activePolygonIndexes[i]].DoubleSided) | (drawingSide != PolygonSides.RearSide))
                            {
                                sideIdx = (polygon[activePolygonIndexes[i]].NormalZ <= 0) ? 0 : 1;

                                if (drawingSide != PolygonSides.AllSides)
                                {
                                    if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                    if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                }

                                if (Engine3D.BuiltStaticPolygon(ref ScreenVertex3D, ref polygon[activePolygonIndexes[i]], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                                switch (polygon[activePolygonIndexes[i]].FillType)
                                {
                                    case PolygonFillType.Solid:
                                        brush.Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];
                                        surface.FillPolygon(brush, poly,partIndex);
                                        break;
                                    case PolygonFillType.Wide:
                                        pen.Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];
                                        surface.DrawPolygon(pen, poly,partIndex);
                                        break;
                                    case PolygonFillType.SquareWide:
                                        pen.Color = polygon[activePolygonIndexes[i]].LightingColor[sideIdx];
                                        surface.DrawLine(pen, poly[0], poly[1],partIndex);
                                        surface.DrawLine(pen, poly[2], poly[0],partIndex);
                                        break;
                                }
                            }
                    });
                    pen.Dispose();
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "ShowPolygonWideModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Преобразование координат в соответствии с перспективной проекцией на экран
        /// </summary>
        /// <param name="K">Коэффициент перспективы</param>
        /// <returns>Возвращает 0 - успешно, не 0 - ошибка</returns>
        public int CreateScreenVertex(float K, int screenCenterX, int screenCenterY)
        {
            try
            {
                return Engine3D.Point3DPresentationConversion(CameraVertex3D, ScreenVertex3D, K, screenCenterX, screenCenterY);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("VolumetricModel3D", "CreateScreenVertex", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion     

    #region ShapeModel3D
    /// <summary>
    /// Пространственные плоские фигуры
    /// </summary>
    public class ShapeModel3D:Unit3D
    {
        /// <summary>
        /// Цвет фигуры
        /// </summary>
        public Color ShapeColor
        {
            set { drawColor= Information.UnitFrontColor = value; }
            get { return Information.UnitFrontColor; }
        }
        public ShapeTypes ShapeType;
        protected Color drawColor;

        public ShapeModel3D():base()
        {            
            ShapeType = ShapeTypes.Circle;
            Information.UnitFrontColor = Color.White;
            drawColor = Color.White;
        }

        public void ResetLighting()
        {
            drawColor = Information.UnitFrontColor;
        }

        public int AddFog(Color FogColor, int MinFogDistance = 0, int FullFogDistance = 5000)
        {
            return Engine3D.AddFog(ref drawColor, ScreenPosition.Z, FogColor, MinFogDistance, FullFogDistance);
        }

        public int Show(IDrawingSurface surface, int bufferIndex)
        {
            try
            {
                SolidBrush brush = new SolidBrush(drawColor);
                float screenDiameter = screenRadius + screenRadius;
                switch (ShapeType)
                {
                    case ShapeTypes.Circle:
                        surface.FillEllipse(brush, screenPosition.X - screenRadius, screenPosition.Y - screenRadius, screenDiameter, screenDiameter, bufferIndex);
                        break; 
                    default:
                        throw new Exception("Нет обработки для указанного вида фигуры.");
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ShapeModel3D", "Show(1)", er.Message, -1);
                return -1;
            }
        }

        public int Show(Graphics drawSurface, SolidBrush brush)
        {
            try
            {
                brush.Color = drawColor;
                float screenDiameter = screenRadius + screenRadius;
                switch (ShapeType)
                {
                    case ShapeTypes.Circle:
                        drawSurface.FillEllipse(brush, screenPosition.X - screenRadius, screenPosition.Y - screenRadius, screenDiameter, screenDiameter);
                        break;
                    default:
                        throw new Exception("Нет обработки для указанного вида фигуры.");
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ShapeModel3D", "Show(2)", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion

    #region UnitActions
    /// <summary>
    /// Действия над юнитом
    /// </summary>
    public class UnitActions
    {
        protected int[] intValues = null;
        protected float[] floatValues = null;

        protected int[] memoryIntValues = null;
        protected float[] memoryFloatValues = null;

        /// <summary>
        /// Тип модели для юнита
        /// </summary>
        public ModelTypes UnitModelType
        {
            get { return unitModelType; }
            set
            {
                unitModelType = value;

                floatValues = new float[0];
                CreateBuffer(ref floatValues);

                intValues = new int[0];
                CreateBuffer(ref intValues);

                if (MemoryBufferEnabled)
                {
                    memoryFloatValues = new float[0];
                    CreateBuffer(ref memoryFloatValues);

                    memoryIntValues = new int[0];
                    CreateBuffer(ref memoryIntValues);
                }
            }
        }
        protected ModelTypes unitModelType = ModelTypes.NONE;

        /// <summary>
        /// Тип действия
        /// </summary>
        public UnitActionTypes ActionType
        {
            get { return actionType; }
        }
        protected UnitActionTypes actionType;
        
        /// <summary>
        /// Выключатель дополнительной памяти для динамики параметров действия
        /// </summary>
        public bool MemoryBufferEnabled
        {
            get { return memoryBufferEnabled; }
            set
            {
                memoryBufferEnabled = value;
                if (value)
                {
                    memoryFloatValues = new float[0];
                    CreateBuffer(ref memoryFloatValues);

                    memoryIntValues = new int[0];
                    CreateBuffer(ref memoryIntValues);
                }
                else
                {
                    memoryIntValues = null;
                    memoryFloatValues = null;
                }
            }
        }
        protected bool memoryBufferEnabled = false;

        /// <summary>
        /// Буфер памяти целых значений
        /// </summary>
        public int[] MemoryIntValues
        {
            get { return memoryIntValues; }
        }
        
        /// <summary>
        /// Буфер памяти вещественных значений
        /// </summary>
        public float[] MemoryFloatValues
        {
            get { return memoryFloatValues; }
        }
        
        /// <summary>
        /// Действия над юнитом
        /// </summary>
        /// <param name="Type"></param>
        public UnitActions(UnitActionTypes Type, ModelTypes ModelType)
        {
            actionType = Type;
            UnitModelType = ModelType;
        }

        /// <summary>
        /// Создание буфера
        /// </summary>
        /// <param name="buffer">Создаваемый буфер</param>
        /// <returns></returns>
        public int CreateBuffer(ref float[] buffer)
        {
            switch (actionType)
            {
                case UnitActionTypes.Move:
                    buffer = new float[] { 0, 0, 0 };
                    break;
                case UnitActionTypes.Rotate:
                    buffer = new float[] { 0 };
                    break;
                case UnitActionTypes.Zoom:
                    buffer = new float[] { 1 };
                    break;
                case UnitActionTypes.MovePoly3DSection:
                    buffer = new float[] { 0, 0, 0 };
                    break;
                case UnitActionTypes.RotatePoly3DSection:
                    buffer = new float[] { 0 };
                    break;
                case UnitActionTypes.Transform:
                    {
                        switch (unitModelType)
                        {
                            case ModelTypes.Plane3D:
                                buffer = new float[6];
                                break;                            
                            case ModelTypes.Cylinder3D:
                                buffer = new float[11];
                                break;
                            case ModelTypes.Tor3D:
                                buffer = new float[12];
                                break;
                            case ModelTypes.Ellipse3D:
                                buffer = new float[10];
                                break;
                            case ModelTypes.Poly3D:
                                buffer = new float[4];
                                break;
                        }
                    }
                    break;
            }
            return 0;
        }

        /// <summary>
        /// Создание буфера
        /// </summary>
        /// <param name="buffer">Создаваемый буфер</param>
        /// <returns></returns>
        public int CreateBuffer(ref int[] buffer)
        {
            switch (actionType)
            {
                case UnitActionTypes.Rotate:
                    buffer = new int[] { 0 };
                    break;
                case UnitActionTypes.MovePoly3DSection:
                    buffer = new int[] { 0 };
                    break;
                case UnitActionTypes.RotatePoly3DSection:
                    buffer = new int[] { 0, 0 };
                    break;
                case UnitActionTypes.Transform:
                    {
                        switch (unitModelType)
                        {
                            case ModelTypes.CellPlane3D:
                                buffer = new int[2];
                                break;
                            case ModelTypes.Prism3D:
                                buffer = new int[3];
                                break;
                        }
                    }
                    break;
            }
            return 0;
        }

        /// <summary>
        /// Установка параметров трансформации
        /// </summary>
        /// <param name="param"></param>
        public void SetTransformation(float[] param)
        {
            for (int i = 0; (i < floatValues.Length)& (i < param.Length); i++) floatValues[i] = param[i];
        }

        /// <summary>
        /// Установка параметров трансформации
        /// </summary>
        /// <param name="param"></param>
        public void SetTransformation(int[] param)
        {
            for (int i = 0; (i < intValues.Length)& (i < param.Length); i++) intValues[i] = param[i];
        }

        /// <summary>
        /// Выдает параметры трансформации
        /// </summary>
        /// <param name="param"></param>
        /// <returns></returns>
        public void GetTransformation(ref float[] param)
        {
            for (int i = 0; (i < floatValues.Length)&(i<param.Length); i++) param[i] = floatValues[i];
        }

        /// <summary>
        /// Выдает параметры трансформации
        /// </summary>
        /// <param name="param"></param>
        /// <returns></returns>
        public void GetTransformation(ref int[] param)
        {
            for (int i = 0; (i < intValues.Length) & (i < param.Length); i++) param[i] = intValues[i];
        }
    }
    #endregion
    
}
