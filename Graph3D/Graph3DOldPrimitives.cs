using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;

namespace Graph3DLibrary
{
    #region Poly3DSection
    /// <summary>
    /// Секция полинома
    /// </summary>
    public class Poly3DSection
    {
        public float StartAngle;
        public float FinishAngle;
        public PointF PositiveRadius;
        public PointF NegativeRadius;
        public Point3D Center;

        public Poly3DSection()
        {
            StartAngle = 0;
            FinishAngle = Engine3D.Radian360;
            PositiveRadius = new PointF();
            NegativeRadius = new PointF();
            Center = new Point3D();
        }

        public Poly3DSection(Poly3DSection Template)
        {
            StartAngle = Template.StartAngle;
            FinishAngle = Template.FinishAngle;
            PositiveRadius = new PointF();
            PositiveRadius.X = Template.PositiveRadius.X;
            PositiveRadius.Y = Template.PositiveRadius.Y;
            NegativeRadius = new PointF();
            NegativeRadius.X = Template.NegativeRadius.X;
            NegativeRadius.Y = Template.NegativeRadius.Y;
            Center = new Point3D(Template.Center);
        }

        public Poly3DSection(float startAngle, float finishAngle, PointF positiveRadius, PointF negativeRadius, Point3D center)
        {
            StartAngle = startAngle;
            FinishAngle = finishAngle;
            PositiveRadius = new PointF(positiveRadius.X, positiveRadius.Y);
            NegativeRadius = new PointF(negativeRadius.X, negativeRadius.Y);
            Center = new Point3D(center);
        }
    }
    #endregion

    #region Poly3D
    /// <summary>
    /// Цилиндрический 3D-полином
    /// </summary>
    public class Poly3D : VolumetricModel3D
    {
        public int sectionCount;
        public int N;

        public Poly3DSection[] SectionParameter
        {
            get { return section; }
        }
        protected Poly3DSection[] section;

        /// <summary>
        /// Цилиндрический 3D-полином
        /// </summary>
        /// <param name="n">Количество опорных точек на секцию</param>
        /// <param name="nSectionCount">Количество секций</param>
        /// <param name="DefaultStartAngle">Стартовый угол по умолчанию</param>
        /// <param name="DefaultFinishAngle">Конечный угол по умолчанию</param>
        /// <param name="DefaultRadius">Радиус секции по умолчанию</param>
        /// <param name="DefaultSectionHeight">Высота секции по умолчанию</param>
        public Poly3D(int n = 2, int nSectionCount = 1, float DefaultStartAngle = 0, float DefaultFinishAngle = Engine3D.Radian360, float DefaultRadius = 30, float DefaultSectionHeight = 10)
        {
            N = 0;
            sectionCount = 0;
            ClosedSurface = false;
            if (BuildModel(n, nSectionCount, DefaultStartAngle, DefaultFinishAngle, DefaultRadius, DefaultSectionHeight) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Poly3D(Poly3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            sectionCount = Template.sectionCount;
            N = Template.N;
            ClosedSurface = Template.ClosedSurface;
            section = new Poly3DSection[Template.section.Length];
            for (int i = 0; i < section.Length; i++) section[i] = new Poly3DSection(Template.section[i]);
        }

        /// <summary>
        /// Трансформация модели
        /// </summary>
        /// <param name="parameter">DefaultStartAngle, DefaultFinishAngle(360), DefaultRadius, DefaultSectionHeight</param>
        /// <returns></returns>
        public int Transform(float[] parameter)
        {
            return BuildModel(N, sectionCount, parameter[0], parameter[1], parameter[2], parameter[3]);
        }

        /// <summary>
        /// Построение модели цилиндрического 3D-полинома
        /// </summary>
        /// <param name="n">Количество опорных точек на секцию</param>
        /// <param name="nSectionCount">Количество секций</param>
        /// <param name="DefaultStartAngle">Стартовый угол по умолчанию</param>
        /// <param name="DefaultFinishAngle">Конечный угол по умолчанию</param>
        /// <param name="DefaultRadius">Радиус секции по умолчанию</param>
        /// <param name="DefaultSectionHeight">Высота секции по умолчанию</param>
        public int BuildModel(int n = 2, int nSectionCount = 1,
                              float DefaultStartAngle = 0, float DefaultFinishAngle = Engine3D.Radian360, float DefaultRadius = 30, float DefaultSectionHeight = 10)
        {
            try
            {
                if (n < 2) throw new Exception("Количество опорных вершин не может быть менее 2.");
                if (nSectionCount < 1) throw new Exception("Количество секций не может быть менее 1.");

                bool newMemory = (N != n) | (sectionCount != nSectionCount);

                N = n;
                sectionCount = nSectionCount;

                float Z = (sectionCount == 1) ? 0 : (-DefaultSectionHeight * (sectionCount - 1) / 2);
                if (newMemory)
                {
                    mainVertex3D = new Point3D[N * sectionCount];
                    section = new Poly3DSection[sectionCount];
                    for (int i = 0; i < sectionCount; i++)
                    {
                        section[i] = new Poly3DSection(DefaultStartAngle, DefaultFinishAngle,
                                                        new PointF(DefaultRadius, DefaultRadius), new PointF(DefaultRadius, DefaultRadius),
                                                        new Point3D(0, 0, Z));
                        Z += DefaultSectionHeight;
                    }
                }
                else
                {
                    for (int i = 0; i < sectionCount; i++)
                    {
                        section[i].StartAngle = DefaultStartAngle;
                        section[i].FinishAngle = DefaultFinishAngle;
                        section[i].Center = new Point3D(0, 0, Z);
                        section[i].PositiveRadius.X = DefaultRadius;
                        section[i].PositiveRadius.Y = DefaultRadius;
                        section[i].NegativeRadius.X = DefaultRadius;
                        section[i].NegativeRadius.Y = DefaultRadius;
                        Z += DefaultSectionHeight;
                    }
                }

                for (int i = 0; i < sectionCount; i++) CalculatePoly3DSection(i, newMemory);

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "BuildModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Расчет координат вершин секции по её параметрам
        /// </summary>
        /// <param name="sectionNumber">Номер секции (нумерация с 0)</param>
        /// <param name="newMemory">Признак, следует ли выделять новую память при формировании координат</param>
        /// <returns></returns>
        public int CalculatePoly3DSection(int sectionNumber, bool newMemory = false)
        {
            try
            {
                if ((sectionNumber < 0) | (sectionNumber >= sectionCount)) throw new Exception("Задан неверный номер секции");

                float angle = section[sectionNumber].StartAngle;
                float step = (section[sectionNumber].FinishAngle - section[sectionNumber].StartAngle) / (N - 1);

                int k = sectionNumber * N;
                for (int i = 0; i < N; i++)
                {
                    Engine3D.CopyPoint3D((((Engine3D.GetNormalizeDegrees(angle) > Engine3D.Radian270) | (Engine3D.GetNormalizeDegrees(angle) < Engine3D.Radian90)) ? section[sectionNumber].PositiveRadius.X : section[sectionNumber].NegativeRadius.X) * (float)Math.Cos(angle) + section[sectionNumber].Center.X,
                          ((Engine3D.GetNormalizeDegrees(angle) < Engine3D.Radian180) ? section[sectionNumber].PositiveRadius.Y : section[sectionNumber].NegativeRadius.Y) * (float)Math.Sin(angle) + section[sectionNumber].Center.Y,
                          section[sectionNumber].Center.Z
                          , ref mainVertex3D[k++], newMemory);
                    angle += step;
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "CalculatePoly3DSection", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Выдет крайние индексы вершин для указанной секции в буфере вершин
        /// </summary>
        /// <param name="sectionNumber">Номер секции (нумерация с 0)</param>
        /// <param name="FirstPointIndex">Первый индекс</param>
        /// <param name="LastPointIndex">Последний индекс</param>
        /// <returns></returns>
        public int GetPoly3DSectionIndexes(int sectionNumber, ref int FirstPointIndex, ref int LastPointIndex)
        {
            try
            {
                if ((sectionNumber < 0) | (sectionNumber >= sectionCount)) throw new Exception("Задан неверный номер секции");

                FirstPointIndex = sectionNumber * N;
                LastPointIndex = FirstPointIndex + N - 1;

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "GetPoly3DSectionIndexes", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Смещение координат секции
        /// </summary>
        /// <param name="sectionNumber">Номер секции (нумерация с 0)</param>
        /// <param name="dX">Смещение по оси 0X</param>
        /// <param name="dY">Смещение по оси 0Y</param>
        /// <param name="dZ">Смещение по оси 0Z</param>
        /// <returns></returns>
        public int MoveSection(int sectionNumber, int dX = 0, int dY = 0, int dZ = 0)
        {
            try
            {
                if ((sectionNumber < 0) | (sectionNumber >= sectionCount)) throw new Exception("Задан неверный номер секции");
                if (Engine3D.MovePoint3D(dX, dY, dZ, section[sectionNumber].Center) != 0) throw new Exception(ErrorLog.GetLastError());
                return Engine3D.MovePoint3D(dX, dY, dZ, ref CameraVertex3D, sectionNumber * N, sectionNumber * N + N - 1);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "MoveSection", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Перемещение координат секции модели к новому центру 
        /// </summary>
        /// <param name="sectionNumber">Номер секции (нумерация с 0)</param>
        /// <param name="X">Новая X-координата центра секции</param>
        /// <param name="Y">Новая X-координата центра секции</param>
        /// <param name="Z">Новая X-координата центра секции</param>
        /// <returns></returns>
        public int MoveSectionTo(int sectionNumber, int X = 0, int Y = 0, int Z = 0)
        {
            try
            {
                if ((sectionNumber < 0) | (sectionNumber >= sectionCount)) throw new Exception("Задан неверный номер секции");
                X -= (int)section[sectionNumber].Center.X;
                Y -= (int)section[sectionNumber].Center.Y;
                Z -= (int)section[sectionNumber].Center.Z;
                return MoveSection(sectionNumber, X, Y, Z);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "MoveSectionTo", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Вращение секции вокруг оси
        /// </summary>
        /// <param name="sectionNumber">Номер секции (нумерация с 0)</param>
        /// <param name="angle">Угол вращения (в радианах)</param>
        /// <param name="axis">Ось вращения</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int RotateSection(int sectionNumber, float angle, Axis3D axis)
        {
            try
            {
                if ((sectionNumber < 0) | (sectionNumber >= sectionCount)) throw new Exception("Задан неверный номер секции");
                return Engine3D.RotatePoint3D(angle, axis, ref CameraVertex3D, sectionNumber * N, sectionNumber * N + N - 1);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "Rotate", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Перестроить модель по последним заданным параметрам
        /// </summary>
        /// <returns></returns>
        public override int RebuildModel(bool newMemory = false)
        {

            try
            {
                int _n = N;
                if (newMemory) N = 0;
                if (BuildModel(_n, sectionCount) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "RebuildModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Построение карты полигонов
        /// </summary>
        /// <returns></returns>
        public override int CreatePolygonMap()
        {
            try
            {
                int PolygonCount = (sectionCount - 1) * (N - 1) * 2;

                polygon = new Polygon3D[PolygonCount];

                int idx = 0;

                for (int i = 1; i < sectionCount; i++)
                    for (int j = 1; j < N; j++)
                    {
                        polygon[idx++] = new Polygon3D(i * N + j, i * N + j - 1, (i - 1) * N + j);
                        polygon[idx++] = new Polygon3D((i - 1) * N + j - 1, (i - 1) * N + j, i * N + j - 1);
                    }

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Poly3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Poly3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Poly3D;
        }

    }
    #endregion

    #region PolyLine3D

    /// <summary>
    /// 
    /// </summary>
    public class PolyLine3D : VolumetricModel3D
    {
        protected int sectionCount;
        protected int defaultLength;
        public int width;
        public DashStyle drawStyle;
        public Color color;

        /// <summary>
        /// Ломанная 3D-линия 
        /// </summary>
        /// <param name="nSectionCount">Число секций</param>
        /// <param name="DefaultLength">Общая длинна линии по умолчанию</param>
        /// <param name="Width">Ширина линии</param>
        /// <param name="DrawStyle">Стиль линии</param>
        /// <param name="LineColor">Цвет линии</param>
        public PolyLine3D(int nSectionCount, int DefaultLength, int Width, DashStyle DrawStyle, Color LineColor)
        {
            sectionCount = 0;
            if (BuildModel(nSectionCount, DefaultLength, Width, DrawStyle, LineColor) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Построение модели ломанной линии
        /// </summary>
        /// <param name="nSectionCount">Число секций</param>
        /// <param name="DefaultLength">Общая длинна линии по умолчанию</param>
        /// <param name="Width">Ширина линии</param>
        /// <param name="DrawStyle">Стиль линии</param>
        /// <param name="LineColor">Цвет линии</param>
        /// <returns></returns>
        public int BuildModel(int nSectionCount, int DefaultLength, int Width, DashStyle DrawStyle, Color LineColor)
        {
            try
            {
                if (nSectionCount < 1) throw new Exception("Количество секций не может быть менее 1.");

                bool newMemory = (sectionCount != nSectionCount);

                sectionCount = nSectionCount;
                width = Width;
                drawStyle = DrawStyle;
                color = LineColor;
                defaultLength = DefaultLength;

                if (newMemory) mainVertex3D = new Point3D[sectionCount + 1];

                float Y = -(float)defaultLength / 2;
                float step = (float)defaultLength / sectionCount;
                for (int i = 0; i < sectionCount; i++)
                {
                    Engine3D.CopyPoint3D(0, Y, 0, ref mainVertex3D[i], newMemory);
                    Y += step;
                }
                Engine3D.CopyPoint3D(0, Y, 0, ref mainVertex3D[sectionCount], newMemory);

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("PolyLine3D", "BuildModel", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Перестроить модель по последним заданным параметрам
        /// </summary>
        /// <returns></returns>
        public override int RebuildModel(bool newMemory = false)
        {
            return BuildModel(sectionCount, defaultLength, width, drawStyle, color);
        }

        /// <summary>
        /// Построение карты полигонов
        /// </summary>
        /// <returns></returns>
        public override int CreatePolygonMap()
        {
            try
            {
                polygon = new Polygon3D[0];
                activePolygonIndexes = new int[0];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("PolyLine3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "PolyLine3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.PolyLine3D;
        }

        /// <summary>
        /// Отрисовка "паутинной" модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <param name="pen">Перо</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowWideModel(Graphics drawSurface, Pen pen)
        {
            try
            {
                for (int i = 0; i < sectionCount; i++)
                {
                    drawSurface.DrawLine(pen, ScreenVertex3D[i].X, ScreenVertex3D[i].Y, ScreenVertex3D[i + 1].X, ScreenVertex3D[i + 1].Y);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("PolyLine3D", "ShowWideModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Отрисовка "паутинной модели" с заданными параметрами
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <param name="perspectiveWidth">Учитывать перспективу для ширины линии</param>
        /// <param name="K">Коэффициент перспективы</param>
        /// <param name="dZ">Смещение по оси Z</param>
        /// <param name="hideMinZ">Скрывать секции, расположенные ближе -dZ</param>
        /// <returns></returns>
        public int ShowWideModel(Graphics drawSurface, bool perspectiveWidth = false, float K = 1, float dZ = 1, bool hideMinZ = true)
        {
            try
            {
                Pen pen = new Pen(color, width);
                pen.DashStyle = drawStyle;
                float z;
                if (K < 0) K = 0;


                for (int i = 0; i < sectionCount; i++)
                    if ((!hideMinZ) | (ScreenVertex3D[i].Z > -dZ))
                    {
                        if (perspectiveWidth)
                        {
                            z = ScreenVertex3D[i].Z + dZ;
                            if (z <= 0) z = (float)0.1;
                            z = K / z;
                            pen.Width = (int)Math.Round(width * z, 0);
                        }
                        drawSurface.DrawLine(pen, ScreenVertex3D[i].X, ScreenVertex3D[i].Y, ScreenVertex3D[i + 1].X, ScreenVertex3D[i + 1].Y);
                    }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("PolyLine3D", "ShowWideModel", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion

    #region RailLine3D

    /// <summary>
    /// Модель железнодорожной линии
    /// </summary>
    public class RailLine3D : VolumetricModel3D
    {
        protected int SectionCount;
        protected int RailWidth;
        protected int SleeperWidth;
        protected int DefaultSleeperLength;
        protected int DefaultLineLength;

        /// <summary>
        /// Количество шпал на секцию
        /// </summary>
        public int SleeperPerSection
        {
            get
            {
                return _SleeperPerSection;
            }
            set
            {
                _SleeperPerSection = (value < 1) ? 1 : value;
            }
        }
        protected int _SleeperPerSection;

        /// <summary>
        /// Стиль линии рельса
        /// </summary>
        public DashStyle RailStyle;
        /// <summary>
        /// Стиль линии шпал
        /// </summary>
        public DashStyle SleeperStyle;
        /// <summary>
        /// Цвет рельса
        /// </summary>
        public Color RailColor;
        /// <summary>
        /// Цвет шпал
        /// </summary>
        public Color SleeperColor;

        /// <summary>
        /// Модель "Железнодорожная линия"
        /// </summary>
        /// <param name="nSectionCount">Количество секций</param>
        /// <param name="defaultSleeperLength">Длинна шпал по умолчанию</param>
        /// <param name="defaultLineLength">Длинна всей линии по умолчанию</param>
        /// <param name="railWidth">Ширина рельса</param>
        /// <param name="sleeperWidth">Ширина шпал</param>
        /// <param name="sleeperPerSection">Количество шпал на секцию</param>
        /// <param name="railStyle">Стиль линии рельса</param>
        /// <param name="sleeperStyle">Стиль линии шпал</param>
        /// <param name="railColor">Цвет рельса</param>
        /// <param name="sleeperColor">Цвет шпал</param>
        public RailLine3D(int nSectionCount,
                          int defaultSleeperLength, int defaultLineLength,
                          int railWidth, int sleeperWidth,
                          int sleeperPerSection,
                          DashStyle railStyle, DashStyle sleeperStyle,
                          Color railColor, Color sleeperColor)
        {
            SectionCount = 0;
            if (BuildModel(nSectionCount,
                          defaultSleeperLength, defaultLineLength,
                          railWidth, sleeperWidth,
                          sleeperPerSection,
                          railStyle, sleeperStyle,
                          railColor, sleeperColor) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Построение модели
        /// </summary>
        /// <param name="nSectionCount">Количество секций</param>
        /// <param name="defaultSleeperLength">Длинна шпал по умолчанию</param>
        /// <param name="defaultLineLength">Длинна всей линии по умолчанию</param>
        /// <param name="railWidth">Ширина рельса</param>
        /// <param name="sleeperWidth">Ширина шпал</param>
        /// <param name="sleeperPerSection">Количество шпал на секцию</param>
        /// <param name="railStyle">Стиль линии рельса</param>
        /// <param name="sleeperStyle">Стиль линии шпал</param>
        /// <param name="railColor">Цвет рельса</param>
        /// <param name="sleeperColor">Цвет шпал</param>
        /// <returns></returns>
        public int BuildModel(int nSectionCount,
                          int defaultSleeperLength, int defaultLineLength,
                          int railWidth, int sleeperWidth,
                          int sleeperPerSection,
                          DashStyle railStyle, DashStyle sleeperStyle,
                          Color railColor, Color sleeperColor)
        {
            try
            {
                if (nSectionCount < 1) throw new Exception("Количество секций не может быть менее 1.");

                bool newMemory = (SectionCount != nSectionCount);

                SectionCount = nSectionCount;
                DefaultSleeperLength = defaultSleeperLength;
                DefaultLineLength = defaultLineLength;
                RailWidth = railWidth;
                SleeperWidth = sleeperWidth;
                SleeperPerSection = sleeperPerSection;
                RailStyle = railStyle;
                SleeperStyle = sleeperStyle;
                RailColor = railColor;
                SleeperColor = sleeperColor;

                if (newMemory) mainVertex3D = new Point3D[(SectionCount + 1) * 2];

                float Y = -(float)DefaultLineLength / 2;
                float step = (float)DefaultLineLength / SectionCount;
                for (int i = 0; i < SectionCount; i++)
                {
                    Engine3D.CopyPoint3D(-DefaultSleeperLength / 2, Y, 0, ref mainVertex3D[i * 2], newMemory);
                    Engine3D.CopyPoint3D(DefaultSleeperLength / 2, Y, 0, ref mainVertex3D[i * 2 + 1], newMemory);
                    Y += step;
                }
                Engine3D.CopyPoint3D(-DefaultSleeperLength / 2, Y, 0, ref mainVertex3D[SectionCount * 2], newMemory);
                Engine3D.CopyPoint3D(DefaultSleeperLength / 2, Y, 0, ref mainVertex3D[SectionCount * 2 + 1], newMemory);

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("RailLine3D", "BuildModel", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Перестроить модель по последним заданным параметрам
        /// </summary>
        /// <returns></returns>
        public override int RebuildModel(bool newMemory = false)
        {
            return BuildModel(SectionCount,
                          DefaultSleeperLength, DefaultLineLength,
                          RailWidth, SleeperWidth,
                          SleeperPerSection,
                          RailStyle, SleeperStyle,
                          RailColor, SleeperColor);
        }

        /// <summary>
        /// Построение карты полигонов
        /// </summary>
        /// <returns></returns>
        public override int CreatePolygonMap()
        {
            try
            {
                polygon = new Polygon3D[0];
                activePolygonIndexes = new int[0];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("RailLine3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "RailLine3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.RailLine3D;
        }

        /// <summary>
        /// Отрисовка "паутинной" модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <param name="pen">Перо</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowWideModel(Graphics drawSurface, Pen pen)
        {
            try
            {
                int j;
                PointF delta1 = new PointF();
                PointF deltaStep1 = new PointF();
                PointF delta2 = new PointF();
                PointF deltaStep2 = new PointF();
                for (int i = 0; i < SectionCount; i++)
                {
                    deltaStep1.X = (ScreenVertex3D[(i + 1) * 2].X - ScreenVertex3D[i * 2].X) / _SleeperPerSection;
                    deltaStep1.Y = (ScreenVertex3D[(i + 1) * 2].Y - ScreenVertex3D[i * 2].Y) / _SleeperPerSection;

                    deltaStep2.X = (ScreenVertex3D[(i + 1) * 2 + 1].X - ScreenVertex3D[i * 2 + 1].X) / _SleeperPerSection;
                    deltaStep2.Y = (ScreenVertex3D[(i + 1) * 2 + 1].Y - ScreenVertex3D[i * 2 + 1].Y) / _SleeperPerSection;

                    delta1.X = ScreenVertex3D[(i + 1) * 2].X + deltaStep1.X / 2;
                    delta1.Y = ScreenVertex3D[(i + 1) * 2].Y + deltaStep1.Y / 2;

                    delta2.X = ScreenVertex3D[(i + 1) * 2 + 1].X + deltaStep2.X / 2;
                    delta2.Y = ScreenVertex3D[(i + 1) * 2 + 1].Y + deltaStep2.Y / 2;

                    for (j = 0; j < SleeperPerSection; j++)
                    {
                        drawSurface.DrawLine(pen, delta1.X, delta1.Y, delta2.X, delta2.Y);
                        delta1.X += deltaStep1.X;
                        delta1.Y += deltaStep1.Y;
                        delta2.X += deltaStep2.X;
                        delta2.Y += deltaStep2.Y;
                    }

                    drawSurface.DrawLine(pen, ScreenVertex3D[i * 2].X, ScreenVertex3D[i * 2].Y, ScreenVertex3D[(i + 1) * 2].X, ScreenVertex3D[(i + 1) * 2].Y);
                    drawSurface.DrawLine(pen, ScreenVertex3D[i * 2 + 1].X, ScreenVertex3D[i * 2 + 1].Y, ScreenVertex3D[(i + 1) * 2 + 1].X, ScreenVertex3D[(i + 1) * 2 + 1].Y);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("RailLine3D", "ShowWideModel", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Отрисовка "паутинной" модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowWideModel(Graphics drawSurface, bool perspectiveWidth = false, float K = 1, float dZ = 1, bool hideMinZ = true)
        {

            try
            {
                int j;
                PointF delta1 = new PointF();
                PointF deltaStep1 = new PointF();
                PointF delta2 = new PointF();
                PointF deltaStep2 = new PointF();

                float z;
                if (K < 0) K = 0;

                Pen pen = new Pen(SleeperColor, SleeperWidth);

                for (int i = 0; i < SectionCount; i++)
                    if ((!hideMinZ) | (ScreenVertex3D[(i) * 2].Z > -dZ) | (ScreenVertex3D[(i + 1) * 2].Z > -dZ) | (ScreenVertex3D[(i) * 2 + 1].Z > -dZ) | (ScreenVertex3D[(i + 1) * 2 + 1].Z > -dZ))
                    {
                        deltaStep1.X = (ScreenVertex3D[(i + 1) * 2].X - ScreenVertex3D[i * 2].X) / _SleeperPerSection;
                        deltaStep1.Y = (ScreenVertex3D[(i + 1) * 2].Y - ScreenVertex3D[i * 2].Y) / _SleeperPerSection;

                        deltaStep2.X = (ScreenVertex3D[(i + 1) * 2 + 1].X - ScreenVertex3D[i * 2 + 1].X) / _SleeperPerSection;
                        deltaStep2.Y = (ScreenVertex3D[(i + 1) * 2 + 1].Y - ScreenVertex3D[i * 2 + 1].Y) / _SleeperPerSection;

                        delta1.X = ScreenVertex3D[i * 2].X + deltaStep1.X / 2;
                        delta1.Y = ScreenVertex3D[i * 2].Y + deltaStep1.Y / 2;

                        delta2.X = ScreenVertex3D[i * 2 + 1].X + deltaStep2.X / 2;
                        delta2.Y = ScreenVertex3D[i * 2 + 1].Y + deltaStep2.Y / 2;

                        pen.DashStyle = SleeperStyle;
                        pen.Color = SleeperColor;


                        if (perspectiveWidth)
                        {
                            z = (ScreenVertex3D[i * 2].Z + ScreenVertex3D[(i + 1) * 2].Z) / 2 + dZ;
                            if (z <= 0) z = (float)0.1;
                            z = K / z;
                            pen.Width = (int)Math.Round(SleeperWidth * z, 0);
                        }
                        else pen.Width = SleeperWidth;

                        for (j = 0; j < SleeperPerSection; j++)
                        {
                            drawSurface.DrawLine(pen, delta1.X, delta1.Y, delta2.X, delta2.Y);

                            delta1.X += deltaStep1.X;
                            delta1.Y += deltaStep1.Y;
                            delta2.X += deltaStep2.X;
                            delta2.Y += deltaStep2.Y;
                        }

                        pen.DashStyle = RailStyle;
                        pen.Color = RailColor;


                        if (perspectiveWidth)
                        {
                            z = (ScreenVertex3D[i * 2].Z + ScreenVertex3D[(i + 1) * 2].Z) / 2 + dZ;
                            if (z <= 0) z = (float)0.1;
                            z = K / z;
                            pen.Width = (int)Math.Round(RailWidth * z, 0);
                        }
                        else pen.Width = RailWidth;

                        drawSurface.DrawLine(pen, ScreenVertex3D[i * 2].X, ScreenVertex3D[i * 2].Y, ScreenVertex3D[(i + 1) * 2].X, ScreenVertex3D[(i + 1) * 2].Y);

                        if (perspectiveWidth)
                        {
                            z = (ScreenVertex3D[i * 2 + 1].Z + ScreenVertex3D[(i + 1) * 2 + 1].Z) / 2 + dZ;
                            if (z <= 0) z = (float)0.1;
                            z = K / z;
                            pen.Width = (int)Math.Round(RailWidth * z, 0);
                        }

                        drawSurface.DrawLine(pen, ScreenVertex3D[i * 2 + 1].X, ScreenVertex3D[i * 2 + 1].Y, ScreenVertex3D[(i + 1) * 2 + 1].X, ScreenVertex3D[(i + 1) * 2 + 1].Y);
                    }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("RailLine3D", "ShowWideModel", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion
}