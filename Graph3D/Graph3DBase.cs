using System;
using System.Drawing;

namespace Graph3DLibrary
{    
    #region Axis3D
    /// <summary>
    /// Оси вращения
    /// </summary>
    public enum Axis3D
    {
        OXyz = 0, // поворот вокруг оси OX
        OYxz = 1, // поворот вокруг оси OY
        OZyx = 2  // поворот вокруг оси OZ        
    };
    #endregion
    
    #region LightTypes
    /// <summary>
    /// Типы источников света
    /// </summary>
    public enum LightTypes
    {
        Ambient = 0, // фоновое освещение (отраженный свет)
        Directional = 1,// бесконечно удаленный источник света (небо, звезды)
        Point = 2,      // точечный источник света (лампа)
        Spot = 3, // направленный ограниченный конусом источник света (фонарь)
        SpotSoft = 4 // направленный ограниченный конусом источник света (фонарь) с мягкой обработкой полигонов
    };
    #endregion

    #region ModelTypes
    /// <summary>
    /// Типы моделей
    /// </summary>
    public enum ModelTypes
    {
        Plane3D = 0,
        CellPlane3D = 1,
        Cylinder3D = 2,
        Ellipse3D = 3,        
        Tor3D = 4,
        Prism3D = 5,
        Poly3D = 6,
        SurfaceHole3D = 7,

        CellBorder3D = 8,

        PolyLine3D = 100,
        RailLine3D = 101,

        
        ComplexModel3D = 1000,
        NONE = 99999
    }
    #endregion
    
    #region Angle3D
    /// <summary>
    /// Класс для хранения значений углов поворота
    /// </summary>
    public class Angle3D
    {
        public float Alpha = 0;
        public float Betta = 0;
        public float Gamma = 0;

        public Angle3D(float a = 0, float b = 0, float g = 0)
        {
            CopyFrom(a, b, g);
        }

        public Angle3D(Angle3D source)
        {
            CopyFrom(source);
        }

        public void CopyFrom(float a = 0, float b = 0, float g = 0)
        {
            Alpha = a;
            Betta = b;
            Gamma = g;
        }

        public void CopyFrom(Angle3D source)
        {
            CopyFrom(source.Alpha, source.Betta, source.Gamma);
        }

        public void CopyTo(ref Angle3D dest)
        {
            dest.Alpha = Alpha;
            dest.Betta = Betta;
            dest.Gamma = Gamma;
        }

        /// <summary>
        /// Возвращает округленный результат
        /// </summary>
        /// <returns></returns>
        public Angle3D GetRound()
        {
            return new Angle3D((int)Math.Round(Alpha), (int)Math.Round(Betta), (int)Math.Round(Gamma));
        }

        /// <summary>
        /// Округление значений всех углов
        /// </summary>
        public void MadeRound()
        {
            Alpha=(int)Math.Round(Alpha); 
            Betta=(int)Math.Round(Betta); 
            Gamma=(int)Math.Round(Gamma);
        }

        public void ToRadian()
        {
            Alpha = Alpha * Engine3D.RadianDegrees;
            Betta = Betta * Engine3D.RadianDegrees;
            Gamma = Gamma * Engine3D.RadianDegrees;
        }

        public void ToGradus()
        {
            Alpha = Alpha / Engine3D.RadianDegrees;
            Betta = Betta / Engine3D.RadianDegrees;
            Gamma = Gamma / Engine3D.RadianDegrees;
        }

    }
    #endregion

    #region Point3D
    /// <summary>
    /// Класс для хранения трехмерных координат точки
    /// </summary>
    public class Point3D
    {
        public float X = 0;
        public float Y = 0;
        public float Z = 0;

        public Point3D(float x = 0, float y = 0, float z = 0)
        {
            CopyFrom(x, y, z);
        }

        public Point3D(Point3D source)
        {
            CopyFrom(source);
        }

        public void CopyFrom(float x = 0, float y = 0, float z = 0)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public void CopyFrom(Point3D source)
        {
            CopyFrom(source.X, source.Y, source.Z);
        }

        public void CopyTo(ref Point3D dest)
        {
            dest.X = X;
            dest.Y = Y;
            dest.Z = Z;
        }

        public Point3D GetRound()
        {
            return new Point3D((int)Math.Round(X), (int)Math.Round(Y), (int)Math.Round(Z));
        }
    }
    #endregion

    #region PolygonFillType
    public enum PolygonFillType
    {
        /// <summary>
        /// Сплошной цвет
        /// </summary>
        Solid = 0,
        /// <summary>
        /// Только контуры
        /// </summary>
        Wide = 1,
        /// <summary>
        /// Только квадратные контуры (2 стороны полигона)
        /// </summary>
        SquareWide = 2
    }
    #endregion

    #region Polygon3D
    /// <summary>
    /// Класс для хранения полигона
    /// </summary>
    public class Polygon3D
    {
        /// <summary>
        /// Индексы вершин полигона
        /// </summary>
        public int[] PointIndex;

        /// <summary>
        /// Собственные цвета полигона
        /// </summary>
        public Color[] color;

        /// <summary>
        /// Расчетные цвета освещения полигона
        /// </summary>
        public Color[] LightingColor;

        /// <summary>
        /// Нормаль к плоскости полигона
        /// </summary>
        public Point3D Normal;

        /// <summary>
        /// Z-нормаль к плоскости полигона после перспективных преобразований
        /// </summary>
        public float NormalZ;

        /// <summary>
        /// Матовость полигона (от 0 до 1) для каждой стороны
        /// </summary>
        public float[] Matte;

        /// <summary>
        /// Координата центра полигона
        /// </summary>
        public Point3D Center;

        /// <summary>
        /// Признак двусторонности полигона
        /// </summary>
        public bool DoubleSided;

        /// <summary>
        /// Тип заливки
        /// </summary>
        public PolygonFillType FillType;

        /// <summary>
        /// Создание полигона
        /// </summary>
        /// <param name="pointNum1">Индекс первой вершины</param>
        /// <param name="pointNum2">Индекс второй вершины</param>
        /// <param name="pointNum3">Индекс третей вершины</param>
        public Polygon3D(int pointNum1, int pointNum2, int pointNum3)
        {
            PointIndex = new int[3];
            color = new Color[2];
            LightingColor = new Color[2];
            Matte = new float[2];

            PointIndex[0] = pointNum1;
            PointIndex[1] = pointNum2;
            PointIndex[2] = pointNum3;

            Normal = new Point3D();
            NormalZ = 0;
            Center = new Point3D();
            DoubleSided = false;
            FillType = PolygonFillType.Solid;

            Matte[0] = Matte[1] = (float)0.5;
            color[0] = color[1] = Color.White;
        }

        public Polygon3D(Polygon3D source)
        {
            PointIndex = new int[3];
            color = new Color[2];
            LightingColor = new Color[2];
            Matte = new float[2];

            PointIndex[0] = source.PointIndex[0];
            PointIndex[1] = source.PointIndex[1];
            PointIndex[2] = source.PointIndex[2];

            Normal = new Point3D();
            NormalZ = 0;
            Center = new Point3D(source.Center);
            DoubleSided = source.DoubleSided;
            FillType = source.FillType;

            Matte[0] = source.Matte[0];
            Matte[1] = source.Matte[1];
            color[0] = source.color[0];
            color[1] = source.color[1];
        }

        public void CopyPropertiesTo(Polygon3D destanation)
        {
            destanation.DoubleSided = DoubleSided;
            destanation.color[0] = color[0];
            destanation.color[1] = color[1];
            destanation.Matte[0] = Matte[0];
            destanation.Matte[1] = Matte[1];
            destanation.FillType = FillType;
        }
    }
    #endregion

    #region PolygonDrawingSides
    /// <summary>
    /// Перечисление отображения сторон полигонов
    /// </summary>
    public enum PolygonSides
    {
        AllSides = 0,
        FrontSide = 1,
        RearSide = 2,
        Auto = 3
    }
    #endregion

    #region ShapeTypes
    /// <summary>
    /// Типы 2D-фигур
    /// </summary>
    public enum ShapeTypes
    {
        Circle = 0
    };
    #endregion

    #region UnitActionTypes
    /// <summary>
    /// Виды действий над юнитом
    /// </summary>
    public enum UnitActionTypes
    {
        Move = 0,
        Rotate = 1,
        Zoom = 2,
        Transform=3,
        MovePoly3DSection=4,
        RotatePoly3DSection = 5
    };
    #endregion

    #region UnitInfo
    public struct UnitInfo
    {
        public ModelTypes UnitType;
        public Color UnitFrontColor;
        public Color UnitRearColor;
        public float UnitFrontMatte;
        public float UnitRearMatte;
        public float GlobalRadius;
        public bool DoubleSidedPolygons;
    }
    #endregion
}