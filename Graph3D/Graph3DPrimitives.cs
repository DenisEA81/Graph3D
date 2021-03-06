using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Globalization;

namespace Graph3DLibrary
{
    #region ComplexModel

    /// <summary>
    /// Класс для создания сложных моделей из нескольких примитивов
    /// </summary>
    public class ComplexModel3D : VolumetricModel3D
    {        
        /// <summary>
        /// Создание нового пустого экземпляра
        /// </summary>
        public ComplexModel3D()
        {}

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public ComplexModel3D(VolumetricModel3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());            
        }

        /// <summary>
        /// Создание экземпляра из файла
        /// </summary>
        /// <param name="FileName"></param>
        public ComplexModel3D(string FileName)
        {
            ModelTypes mt = ModelTypes.NONE;
            if (OpenModelFromFile(FileName,ref mt) != 0) throw new Exception(ErrorLog.GetLastError());
        }
                       

        /// <summary>
        /// Добавление объекта в модель
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        public int MergeObject(VolumetricModel3D model)
        {
            try
            {
                int k;                
                int m = mainVertex3D.Length;

                ClosedSurface &= model.ClosedSurface;

                Point3D[] tmp = new Point3D[mainVertex3D.Length + model.MainVertex3D.Length];
                for (int i = 0; i < mainVertex3D.Length; i++) tmp[i] = new Point3D(mainVertex3D[i]);

                k = mainVertex3D.Length;
                for (int i=0;i<model.MainVertex3D.Length;i++)
                    tmp[k++] = new Point3D(model.MainVertex3D[i]);
                
                Engine3D.CopyPoint3D(tmp, ref mainVertex3D, true);

                Polygon3D[] p = new Polygon3D[polygon.Length + model.Polygon.Length];
                k = 0;
                for (int i = 0; i < polygon.Length; i++)
                {
                    p[k] = new Polygon3D(polygon[i].PointIndex[0], polygon[i].PointIndex[1], polygon[i].PointIndex[2]);
                    polygon[i].CopyPropertiesTo(p[k]);
                    k++;
                }
                
                for (int i = 0; i < model.Polygon.Length; i++)
                {
                    p[k] = new Polygon3D(model.Polygon[i].PointIndex[0] + m, model.Polygon[i].PointIndex[1] + m, model.Polygon[i].PointIndex[2] + m);
                    model.Polygon[i].CopyPropertiesTo(p[k]);
                    k++;
                }
                polygon = p;
                activePolygonIndexes = new int[polygon.Length];
                ResetActivePolygonIndexes();
                return CreateDeformVertexBuffers(true); 
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ComplexModel3D", "MergeObject", er.Message, -1);
                return -1;
            }
        }

        public int MergeNearVertex(int MinDistance)
        {
            try
            {
                Point3D[] tmpVertex = new Point3D[MainVertex3D.Length];
                int tmpCount = 0;
                MinDistance *= MinDistance;// возводим в квадрат
                #region Слияние вершин
                for (int i=0;i<MainVertex3D.Length;i++)
                {
                    int idxNear = -1;

                    for (int j=0;j< tmpCount; j++)
                        if (Engine3D.GetSquareDistance(tmpVertex[j],MainVertex3D[i])<=MinDistance)
                        {
                            idxNear = j;
                            break;
                        }

                    if (idxNear<0)
                    {
                        idxNear = tmpCount;
                        tmpVertex[tmpCount++] = new Point3D(MainVertex3D[i]);
                    }

                    for (int j=0;j<Polygon.Length;j++)
                    {
                        for (int k = 0; k < Polygon[j].PointIndex.Length; k++)
                            if (Polygon[j].PointIndex[k] == i) Polygon[j].PointIndex[k] = idxNear;
                    }
                }

                mainVertex3D = new Point3D[tmpCount];
                for (int i = 0; i < mainVertex3D.Length; i++) mainVertex3D[i] = new Point3D(tmpVertex[i]);
                #endregion

                #region Фильтрация некорректных (слипшихся) полигонов
                Polygon3D[] tmpPolygon = new Polygon3D[Polygon.Length];
                tmpCount = 0;
                for (int k=0;k<Polygon.Length;k++)
                    if (!((Polygon[k].PointIndex[0] == Polygon[k].PointIndex[1])|(Polygon[k].PointIndex[0] == Polygon[k].PointIndex[2])|(Polygon[k].PointIndex[1] == Polygon[k].PointIndex[2]))) tmpPolygon[tmpCount++] = Polygon[k];

                polygon = new Polygon3D[tmpCount];
                for (int i = 0; i < polygon.Length; i++) polygon[i] = tmpPolygon[i];
                if (tmpCount == 0) throw new Exception("В результате преобразования не осталось ни одного корректного полигона.\nВсе изменения будут сброшены.");
                
                #endregion

                activePolygonIndexes = new int[polygon.Length];
                if (ResetActivePolygonIndexes()<0) throw new Exception(ErrorLog.GetLastError());

                return CreateDeformVertexBuffers(true);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ComplexModel3D", "MergeNearVertex", er.Message, -1);
                if (Polygon.Length <= 0) return -2;
                return -1;
            }
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.ComplexModel3D;
        }

        /// <summary>
        /// Не используется.
        /// </summary>
        /// <returns></returns>
        public override int RebuildModel(bool newMemory = false)
        {
            return 0;
        }

        /// <summary>
        /// Не используется по прямому назначению.
        /// </summary>
        /// <returns></returns>
        public override int CreatePolygonMap()
        {
            if (polygon == null) polygon = new Polygon3D[0];
            activePolygonIndexes = new int[polygon.Length];
            return ResetActivePolygonIndexes();
        }

    }
    #endregion

    #region Plane3D
    /// <summary>
    /// Вершинная модель плоскости
    /// </summary>
    public class Plane3D : VolumetricModel3D
    {
        public int N;
        public float RxPlus;
        public float RxMinus;
        public float RyPlus;
        public float RyMinus;
        public float AngleStart;
        public float AngleFinish;

        /// <summary>
        /// Создание модели плоскости
        /// </summary>
        /// <param name="n">количество углов плоскости</param>
        /// <param name="RxPlus">Радиус по положительной оси ОХ</param>
        /// <param name="RxMinus">Радиус по отрицательной оси ОХ</param>
        /// <param name="RyPlus">Радиус по положительной оси ОY</param>
        /// <param name="RyMinus">Радиус по отрицательной оси ОY</param>
        public Plane3D(int n = 4, float rX_Plus = 100, float rX_Minus = 100, float rY_Plus = 100, float rY_Minus = 100, float angleStart = 0, float angleFinish = Engine3D.Radian360)
        {
            N = 0;
            ClosedSurface = false;
            if (BuildModel(n, rX_Plus, rX_Minus, rY_Plus, rY_Minus, angleStart, angleFinish) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Plane3D(Plane3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            N = Template.N;
            RxPlus=Template.RxPlus;
            RxMinus = Template.RxMinus;
            RyPlus = Template.RyPlus;
            RyMinus = Template.RyMinus;
            AngleStart = Template.AngleStart;
            AngleFinish = Template.AngleFinish;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public Plane3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
            //if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName,this.ModelType())!=0) throw new Exception(ErrorLog.GetLastError());

                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("N=" + N.ToString());
                file.WriteLine("RxPlus=" + RxPlus.ToString("0.0000", culture));
                file.WriteLine("RxMinus=" + RxMinus.ToString("0.0000", culture));
                file.WriteLine("RyPlus=" + RyPlus.ToString("0.0000", culture));
                file.WriteLine("RyMinus=" + RyMinus.ToString("0.0000", culture));
                file.WriteLine("AngleStart=" + AngleStart.ToString("0.0000", culture));
                file.WriteLine("AngleFinish=" + AngleFinish.ToString("0.0000", culture));
                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType=ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;
                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "N":
                                                N = int.Parse(PropertyValue);
                                                break;
                                            case "RxPlus":
                                                RxPlus = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RxMinus":
                                                RxMinus = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyPlus":
                                                RyPlus = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyMinus":
                                                RyMinus = float.Parse(PropertyValue, culture);
                                                break;
                                            case "AngleStart":
                                                AngleStart = float.Parse(PropertyValue, culture);
                                                break;
                                            case "AngleFinish":
                                                AngleFinish = float.Parse(PropertyValue, culture);
                                                break;
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Построение модели по заданным параметрам и сброс модели деформаций к исходной
        /// </summary>
        /// <param name="n">количество углов плоскости</param>
        /// <param name="RxPlus">Радиус по положительной оси ОХ</param>
        /// <param name="RxMinus">Радиус по отрицательной оси ОХ</param>
        /// <param name="RyPlus">Радиус по положительной оси ОY</param>
        /// <param name="RyMinus">Радиус по отрицательной оси ОY</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int n = 4, float rX_Plus = 10, float rX_Minus = 10, float rY_Plus = 10, float rY_Minus = 10, float angleStart = 0, float angleFinish = Engine3D.Radian360)
        {
            try
            {
                if (n < 3) throw new Exception("Модель плоскости не может содержать менее четырех опорных точек (три угла для замкнутого многоугольника).");

                float angle = angleStart;
                float cos_angle;
                float sin_angle;

                bool NewMemory = N != n;
                N = n;
                RxPlus = rX_Plus;
                RxMinus = rX_Minus;
                RyPlus = rY_Plus;
                RyMinus = rY_Minus;
                AngleStart = angleStart;
                AngleFinish = angleFinish;

                float step = (float)(angleFinish - angleStart) / (N - 1);

                if (NewMemory) mainVertex3D = new Point3D[N + 1];

                Engine3D.CopyPoint3D(0, 0, 0, ref mainVertex3D[n], NewMemory);
                for (int i = 0; i < n; i++)
                {
                    cos_angle = (float)Math.Cos(angle);
                    sin_angle = (float)Math.Sin(angle);

                    Engine3D.CopyPoint3D(((cos_angle >= 0) ? rX_Plus * cos_angle : rX_Minus * cos_angle),
                                         ((sin_angle >= 0) ? rY_Plus * sin_angle : rY_Minus * sin_angle),
                                          0, ref mainVertex3D[i], NewMemory);
                    angle += step;
                }

                return CreateDeformVertexBuffers(NewMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "BuildModel", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Перестроить модель по последним заданным параметрам
        /// </summary>
        /// <returns></returns>
        public override int RebuildModel(bool newMemory=false)
        {
            try
            {
                int _n = N;
                if (newMemory) N = 0;
                if (BuildModel(_n, RxPlus, RxMinus, RyPlus, RyMinus,AngleStart, AngleFinish) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());                
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "RebuildModel", er.Message, -1);
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
                polygon = new Polygon3D[N];

                for (int i = 0; i < N; i++) polygon[i] = new Polygon3D(i, N, i + 1);

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Plane3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Plane3D;
        }

    }
    #endregion

    #region CellPlane3D
    /// <summary>
    /// Модель сетчатой плоскости
    /// </summary>
    public class CellPlane3D : VolumetricModel3D
    {
        public float _WidthX;
        public float _WidthZ;
        public int _Nx;
        public int _Nz;

        /// <summary>
        /// Создание модели сетчатой плоскости
        /// </summary>
        /// <param name="nX">Количество линий сетки по оси 0X</param>
        /// <param name="nZ">Количество линий сетки по оси 0Z</param>
        /// <param name="widthX">ширина по оси 0X</param>
        /// <param name="widthZ">ширина по оси 0Z</param>
        public CellPlane3D(int nX = 2, int nZ = 2, int widthX = 100, int widthZ = 100)
        {
            _Nx = 0;
            _Nz = 0;
            if (BuildModel(nX, nZ, widthX, widthZ) != 0) throw new Exception("Ошибка при построении модели объекта.");
            ClosedSurface = false;
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public CellPlane3D(CellPlane3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            _WidthX = Template._WidthX;
            _WidthZ = Template._WidthZ;
            _Nx = Template._Nx;
            _Nz = Template._Nz;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public CellPlane3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
            //if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }

        /// <summary>
        /// Построение модели по заданным параметрам и сброс модели деформаций к исходной
        /// </summary>
        /// <param name="nX">Количество линий сетки по оси 0X</param>
        /// <param name="nZ">Количество линий сетки по оси 0Z</param>
        /// <param name="widthX">ширина по оси 0X</param>
        /// <param name="widthZ">ширина по оси 0Z</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int nX = 2, int nZ = 2, float widthX = 100, float widthZ = 100)
        {
            try
            {
                if (nX < 2) throw new Exception("Сторона модели не может содержать менее двух опорных точек.");
                if (nZ < 2) throw new Exception("Сторона модели не может содержать менее двух опорных точек.");
                bool NewMemory = (_Nx != nX) | (_Nz != nZ);
                _Nx = nX;
                _Nz = nZ;
                _WidthX = widthX;
                _WidthZ = widthZ;

                float stepX = (float)_WidthX / (_Nx - 1);
                float stepZ = (float)_WidthZ / (_Nz - 1);

                if (NewMemory) mainVertex3D = new Point3D[_Nx * _Nz];

                int k = 0;
                int i;

                for (int j = 0; j < _Nz; j++)
                    for (i = 0; i < _Nx; i++)
                        Engine3D.CopyPoint3D((float)i * stepX - _WidthX / 2, 0, (float)j * stepZ - _WidthZ / 2, ref mainVertex3D[k++], NewMemory);

                return CreateDeformVertexBuffers(NewMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "BuildModel", er.Message, -1);
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
                int nX = _Nx;
                int nZ = _Nz;
                if (newMemory) _Nx = 0;
                if (BuildModel(nX, nZ, _WidthX, _WidthZ) != 0) throw new Exception(ErrorLog.GetLastError());
                RebuildPolygonMap();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Plane3D", "RebuildModel", er.Message, -1);
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
                polygon = new Polygon3D[(_Nx - 1) * (_Nz - 1) * 2];

                int k = 0;
                for (int i = 1; i < _Nx; i++)
                    for (int j = 1; j < _Nz; j++)
                    {
                        polygon[k++] = new Polygon3D(j * _Nx + i - 1, j * _Nx + i, (j - 1) * _Nx + i - 1);
                        polygon[k++] = new Polygon3D((j - 1) * _Nx + i, (j - 1) * _Nx + i - 1, j * _Nx + i);
                    }

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "CellPlane3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.CellPlane3D;
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("_Nx=" + _Nx.ToString());
                file.WriteLine("_Nz=" + _Nz.ToString());
                file.WriteLine("_WidthX=" + _WidthX.ToString("0.0000", culture));
                file.WriteLine("_WidthZ=" + _WidthZ.ToString("0.0000", culture));
                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);
                                        
                                        switch (PropertyName)
                                        {
                                            case "_Nx":
                                                _Nx = int.Parse(PropertyValue);
                                                break;
                                            case "_Nz":
                                                _Nz = int.Parse(PropertyValue);
                                                break;
                                            case "_WidthX":
                                                _WidthX = float.Parse(PropertyValue,culture);
                                                break;
                                            case "_WidthZ":
                                                _WidthZ = float.Parse(PropertyValue, culture);
                                                break;
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Установка цвета клеток. Нумерация начинается с 0.
        /// </summary>
        /// <param name="color">Цвет</param>
        /// <param name="CellNumber">Номер клетки</param>
        /// <param name="PolygonSide">Окрашиваемые стороны полигонов</param>
        /// <returns></returns>
        public int SetCellColor(Color color, int CellNumber = 0, PolygonSides PolygonSide = PolygonSides.AllSides)
        {
            try
            {
                if (CellNumber < 0) throw new Exception("Номер стороны модели не может быть менее 0.");
                if (CellNumber > ((_Nx - 1) * (_Nz - 1))) throw new Exception("Задан слишком большой номер клетки.");

                return SetColor(color, CellNumber * 2, CellNumber * 2 + 1, PolygonSide);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "SetCellColor", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Цельная отрисовка сетки указанной толщиной
        /// </summary>
        /// <param name="drawSurface">Поверхность рисования</param>        
        /// <param name="SpecialColor">Особый цвет отрисовки</param>        
        /// <param name="lineWidth">Толщина линий</param>
        /// <returns></returns>
        public int ShowWideOnlyCellModel(Graphics drawSurface, Color SpecialColor, int lineWidth)
        {
            try
            {
                Pen pen = new Pen(SpecialColor, lineWidth);
                                
                for (int i = 0; i < _Nx; i++)
                {
                    drawSurface.DrawLine(pen, ScreenVertex3D[i * _Nz].X, ScreenVertex3D[i * _Nz].Y, ScreenVertex3D[(i + 1) * _Nz - 1].X, ScreenVertex3D[(i + 1) * _Nz - 1].Y);
                }
                

                lineWidth = (_Nx - 1) * _Nz;
                for (int i = 0; i < _Nz; i++)
                {
                    drawSurface.DrawLine(pen, ScreenVertex3D[i].X, ScreenVertex3D[i].Y, ScreenVertex3D[lineWidth + i].X, ScreenVertex3D[lineWidth + i].Y);
                }

                pen.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "ShowWideOnlyCellModel", er.Message, -1);
                return -1;
            }
        }

    }
    #endregion


    #region CellBorder3D
    /// <summary>
    /// Модель сетчатой плоскости
    /// </summary>
    public class CellBorder3D : VolumetricModel3D
    {
        public float _WidthX;
        public float _WidthZ;
        public int _Nx;
        public int _Nz;

        /// <summary>
        /// Создание модели сетчатой плоской рамки
        /// </summary>
        /// <param name="nX">Количество линий сетки по оси 0X</param>
        /// <param name="nZ">Количество линий сетки по оси 0Z</param>
        /// <param name="widthX">ширина по оси 0X</param>
        /// <param name="widthZ">ширина по оси 0Z</param>
        public CellBorder3D(int nX = 2, int nZ = 2, int widthX = 100, int widthZ = 100)
        {
            _Nx = 0;
            _Nz = 0;
            if (BuildModel(nX, nZ, widthX, widthZ) != 0) throw new Exception("Ошибка при построении модели объекта.");
            ClosedSurface = false;
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public CellBorder3D(CellPlane3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            _WidthX = Template._WidthX;
            _WidthZ = Template._WidthZ;
            _Nx = Template._Nx;
            _Nz = Template._Nz;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public CellBorder3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());            
        }

        /// <summary>
        /// Построение модели по заданным параметрам и сброс модели деформаций к исходной
        /// </summary>
        /// <param name="nX">Количество линий сетки по оси 0X</param>
        /// <param name="nZ">Количество линий сетки по оси 0Z</param>
        /// <param name="widthX">ширина по оси 0X</param>
        /// <param name="widthZ">ширина по оси 0Z</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int nX = 2, int nZ = 2, float widthX = 100, float widthZ = 100)
        {
            try
            {
                if (nX < 2) throw new Exception("Сторона модели не может содержать менее двух опорных точек.");
                if (nZ < 2) throw new Exception("Сторона модели не может содержать менее двух опорных точек.");
                bool NewMemory = (_Nx != nX) | (_Nz != nZ);
                _Nx = nX;
                _Nz = nZ;
                _WidthX = widthX;
                _WidthZ = widthZ;

                float stepX = (float)_WidthX / (_Nx - 1);
                float stepZ = (float)_WidthZ / (_Nz - 1);

                if (NewMemory) mainVertex3D = new Point3D[(_Nx + _Nz)*2];

                int k = 0;
                int i;

                for (i = 0; i < _Nx; i++)
                    Engine3D.CopyPoint3D((float)i * stepX - _WidthX / 2, 0, -_WidthZ / 2, ref mainVertex3D[k++], NewMemory);

                for (i = 0; i < _Nx; i++)
                    Engine3D.CopyPoint3D((float)i * stepX - _WidthX / 2, 0, _WidthZ/2, ref mainVertex3D[k++], NewMemory);

                for (i = 0; i < _Nz; i++)
                    Engine3D.CopyPoint3D(-_WidthX / 2, 0, (float)i * stepZ - _WidthZ / 2, ref mainVertex3D[k++], NewMemory);

                for (i = 0; i < _Nz; i++)
                    Engine3D.CopyPoint3D(_WidthX/2, 0, (float)i * stepZ - _WidthZ / 2, ref mainVertex3D[k++], NewMemory);

                return CreateDeformVertexBuffers(NewMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellBorder3D", "BuildModel", er.Message, -1);
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
                int nX = _Nx;
                int nZ = _Nz;
                if (newMemory) _Nx = 0;
                if (BuildModel(nX, nZ, _WidthX, _WidthZ) != 0) throw new Exception(ErrorLog.GetLastError());
                RebuildPolygonMap();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellBorder3D", "RebuildModel", er.Message, -1);
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
                polygon = new Polygon3D[0];
                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellBorder3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "CellBorder3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.CellPlane3D;
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("_Nx=" + _Nx.ToString());
                file.WriteLine("_Nz=" + _Nz.ToString());
                file.WriteLine("_WidthX=" + _WidthX.ToString("0.0000", culture));
                file.WriteLine("_WidthZ=" + _WidthZ.ToString("0.0000", culture));
                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellBorder3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "_Nx":
                                                _Nx = int.Parse(PropertyValue);
                                                break;
                                            case "_Nz":
                                                _Nz = int.Parse(PropertyValue);
                                                break;
                                            case "_WidthX":
                                                _WidthX = float.Parse(PropertyValue, culture);
                                                break;
                                            case "_WidthZ":
                                                _WidthZ = float.Parse(PropertyValue, culture);
                                                break;
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellBorder3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Цельная отрисовка сетки указанной толщиной
        /// </summary>
        /// <param name="drawSurface">Поверхность рисования</param>        
        /// <param name="SpecialColor">Особый цвет отрисовки</param>        
        /// <param name="lineWidth">Толщина линий</param>
        /// <returns></returns>
        public int ShowWideOnlyCellModel(Graphics drawSurface, Color SpecialColor, int lineWidth, int stepX = 1, int stepZ = 1)
        {
            try
            {
                Pen pen = new Pen(SpecialColor, lineWidth);

                for (int i = 0; i < _Nx; i+=stepX)
                    drawSurface.DrawLine(pen, ScreenVertex3D[i].X, ScreenVertex3D[i].Y, ScreenVertex3D[i + _Nx].X, ScreenVertex3D[i + _Nx].Y);

                lineWidth = _Nx * 2;
                for (int i = 0; i < _Nz; i+=stepZ)
                    drawSurface.DrawLine(pen, ScreenVertex3D[lineWidth+i].X, ScreenVertex3D[lineWidth+i].Y, ScreenVertex3D[lineWidth+i + _Nz].X, ScreenVertex3D[lineWidth+i + _Nz].Y);

                pen.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("CellPlane3D", "ShowWideOnlyCellModel", er.Message, -1);
                return -1;
            }
        }

    }
    #endregion    


    #region Cylinder3D
    /// <summary>
    /// Вершинная модель цилиндра с секциями
    /// </summary>
    public class Cylinder3D : VolumetricModel3D
    {
        public float Height;
        public float RBottomXpositive;
        public float RBottomXnegative;
        public float RBottomZpositive;
        public float RBottomZnegative;
        public float RTopXpositive;
        public float RTopXnegative;
        public float RTopZpositive;
        public float RTopZnegative;
        public float AngleStart;
        public float AngleFinish;
        public bool TopVisible;
        public bool BottomVisible;
        public int N;
        public int sectionN;

        /// <summary>
        /// Создание модели цилиндра
        /// </summary>
        /// <param name="n">Количество опорных вершин</param>
        /// <param name="height">Высота цилиндра</param>
        /// <param name="rBottomXpositive">Радиус в нижней части цилиндра по положительной части оси 0X</param>
        /// <param name="rBottomXnegative">Радиус в нижней части цилиндра по отрицательной части оси 0X</param>
        /// <param name="rBottomZpositive">Радиус в нижней части цилиндра по положительной части оси 0Z</param>
        /// <param name="rBottomZnegative">Радиус в нижней части цилиндра по отрицательной части оси 0Z</param>
        /// <param name="rTopXpositive">Радиус в верхней части цилиндра по положительной части оси 0X</param>
        /// <param name="rTopXnegative">Радиус в верхней части цилиндра по отрицательной части оси 0X</param>
        /// <param name="rTopZpositive">Радиус в верхней части цилиндра по положительной части оси 0Z</param>
        /// <param name="rTopZnegative">Радиус в верхней части цилиндра по отрицательной части оси 0Z</param>
        /// /// <param name="topVisible">Признак наличия верхней крышки</param>
        /// <param name="bottomVisible">Признак наличия нижней крышки</param>

        public Cylinder3D(int n = 4, int sn = 2, float height = (float)14.1,
                          float rBottomXpositive = 10, float rBottomXnegative = 10,
                          float rBottomZpositive = 10, float rBottomZnegative = 10,
                          float rTopXpositive = 10, float rTopXnegative = 10,
                          float rTopZpositive = 10, float rTopZnegative = 10,
                          bool topVisible = true, bool bottomVisible = true,
                          float angleStart = 0, float angleFinish = Engine3D.Radian360)
        {
            N = 0;
            sectionN = 0;
            TopVisible = true;
            BottomVisible = true;
            ClosedSurface = TopVisible & BottomVisible & (Math.Abs(angleFinish - angleStart - Engine3D.Radian360) < Engine3D.RadianDegrees);
            if (BuildModel(n, sn, topVisible, bottomVisible,
                rBottomXpositive, rBottomXnegative, rBottomZpositive, rBottomZnegative,
                rTopXpositive, rTopXnegative, rTopZpositive, rTopZnegative,
                angleStart, angleFinish,
                height) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Cylinder3D(Cylinder3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            Height = Template.Height;
            RBottomXpositive = Template.RBottomXpositive;
            RBottomXnegative = Template.RBottomXnegative;
            RBottomZpositive = Template.RBottomZpositive;
            RBottomZnegative = Template.RBottomZnegative;
            RTopXpositive = Template.RTopXpositive;
            RTopXnegative = Template.RTopXnegative;
            RTopZpositive = Template.RTopZpositive;
            RTopZnegative = Template.RTopZnegative;
            AngleStart = Template.AngleStart;
            AngleFinish = Template.AngleFinish;
            TopVisible = Template.TopVisible;
            BottomVisible = Template.BottomVisible;
            N = Template.N;
            sectionN = Template.sectionN;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public Cylinder3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
           // if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("N=" + N.ToString());
                file.WriteLine("sectionN=" + sectionN.ToString());
                file.WriteLine("Height=" + Height.ToString("0.0000", culture));
                file.WriteLine("RBottomXpositive=" + RBottomXpositive.ToString("0.0000", culture));
                file.WriteLine("RBottomXnegative=" + RBottomXnegative.ToString("0.0000", culture));
                file.WriteLine("RBottomZpositive=" + RBottomZpositive.ToString("0.0000", culture));
                file.WriteLine("RBottomZnegative=" + RBottomZnegative.ToString("0.0000", culture));
                file.WriteLine("RTopXpositive=" + RTopXpositive.ToString("0.0000", culture));
                file.WriteLine("RTopXnegative=" + RTopXnegative.ToString("0.0000", culture));
                file.WriteLine("RTopZpositive=" + RTopZpositive.ToString("0.0000", culture));
                file.WriteLine("RTopZnegative=" + RTopZnegative.ToString("0.0000", culture));
                file.WriteLine("AngleStart=" + AngleStart.ToString("0.0000", culture));
                file.WriteLine("AngleFinish=" + AngleFinish.ToString("0.0000", culture));
                file.WriteLine("TopVisible=" + (TopVisible?"1":"0"));
                file.WriteLine("BottomVisible=" + (BottomVisible ? "1" : "0"));

                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Cylinder3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;
                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "N":
                                                N = int.Parse(PropertyValue);
                                                break;
                                            case "sectionN":
                                                sectionN = int.Parse(PropertyValue);
                                                break;
                                            case "Height":
                                                Height = float.Parse(PropertyValue,culture);
                                                break;
                                            case "RBottomXpositive":
                                                RBottomXpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RBottomXnegative":
                                                RBottomXnegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RBottomZpositive":
                                                RBottomZpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RBottomZnegative":
                                                RBottomZnegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RTopXpositive":
                                                RTopXpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RTopXnegative":
                                                RTopXnegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RTopZpositive":
                                                RTopZpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RTopZnegative":
                                                RTopZnegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "AngleStart":
                                                AngleStart = float.Parse(PropertyValue, culture);
                                                break;
                                            case "AngleFinish":
                                                AngleFinish = float.Parse(PropertyValue, culture);
                                                break;
                                            case "TopVisible":
                                                TopVisible = (PropertyValue=="1");
                                                break;
                                            case "BottomVisible":
                                                BottomVisible = (PropertyValue == "1");
                                                break;
                                        }                                                                                                                       
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Cylinder3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Построение модели цилиндра по заданным параметрам.
        /// </summary>
        /// <param name="n">Количество опорных вершин</param>
        /// <param name="topVisible">Признак наличия верхней крышки</param>
        /// <param name="bottomVisible">Признак наличия нижней крышки</param>
        /// <param name="rBottomXpositive">Радиус в нижней части цилиндра по положительной части оси 0X</param>
        /// <param name="rBottomXnegative">Радиус в нижней части цилиндра по отрицательной части оси 0X</param>
        /// <param name="rBottomZpositive">Радиус в нижней части цилиндра по положительной части оси 0Z</param>
        /// <param name="rBottomZnegative">Радиус в нижней части цилиндра по отрицательной части оси 0Z</param>
        /// <param name="rTopXpositive">Радиус в верхней части цилиндра по положительной части оси 0X</param>
        /// <param name="rTopXnegative">Радиус в верхней части цилиндра по отрицательной части оси 0X</param>
        /// <param name="rTopZpositive">Радиус в верхней части цилиндра по положительной части оси 0Z</param>
        /// <param name="rTopZnegative">Радиус в верхней части цилиндра по отрицательной части оси 0Z</param>
        /// <param name="height">Высота цилиндра</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int n = 4, int sn = 2, bool topVisible = true, bool bottomVisible = true,
                          float rBottomXpositive = 10, float rBottomXnegative = 10,
                          float rBottomZpositive = 10, float rBottomZnegative = 10,
                          float rTopXpositive = 10, float rTopXnegative = 10,
                          float rTopZpositive = 10, float rTopZnegative = 10,
                          float angleStart = 0, float angleFinish = Engine3D.Radian360,
                          float height = (float)14.1)
        {
            try
            {
                if (n < 3) throw new Exception("Верхняя и нижняя грани цилиндра не могут содержать менее чем по 3 вершины (2 боковые грани).");
                if (sn < 1) throw new Exception("Цилиндр должен состоять как минимум из одной секции");

                bool newMemory = (N != n) | (sectionN != sn) | (TopVisible != topVisible) | (BottomVisible != bottomVisible);

                float angle = angleStart;
                float cosAngle;
                float sinAngle;

                N = n;
                sectionN = sn;
                Height = height;
                RBottomXpositive = rBottomXpositive;
                RBottomXnegative = rBottomXnegative;
                RBottomZpositive = rBottomZpositive;
                RBottomZnegative = rBottomZnegative;
                RTopXpositive = rTopXpositive;
                RTopXnegative = rTopXnegative;
                RTopZpositive = rTopZpositive;
                RTopZnegative = rTopZnegative;
                AngleStart = angleStart;
                AngleFinish = angleFinish;
                TopVisible = topVisible;
                BottomVisible = bottomVisible;

                if (newMemory) mainVertex3D = new Point3D[N * (sectionN + 1) + (TopVisible ? 1 : 0) + (BottomVisible ? 1 : 0)];

                bool xp;
                bool zp;
                float hStep = Height / sectionN;
                float h = 0;
                float x = 0;
                float z = 0;
                int k = 0;
                float xPStep;
                float zPStep;
                float xNStep;
                float zNStep;
                float dx;
                float dz;

                xPStep = (RTopXpositive - RBottomXpositive) / sectionN;
                zPStep = (RTopZpositive - RBottomZpositive) / sectionN;
                xNStep = (RTopXnegative - RBottomXnegative) / sectionN;
                zNStep = (RTopZnegative - RBottomZnegative) / sectionN;

                float step = (AngleFinish - AngleStart) / (N - 1);
                for (int i = 0; i < N; i++)
                {
                    cosAngle = (float)Math.Cos(angle);
                    sinAngle = (float)Math.Sin(angle);

                    xp = (Engine3D.GetNormalizeDegrees(angle) < Engine3D.Radian90) | (Engine3D.GetNormalizeDegrees(angle) > Engine3D.Radian270);
                    zp = (Engine3D.GetNormalizeDegrees(angle) < Engine3D.Radian180);

                    x = (xp ? RBottomXpositive: RBottomXnegative) * cosAngle;
                    z = (zp ? RBottomZpositive: RBottomZnegative) * sinAngle;

                    h = -Height / 2;

                    dx = (xp ? xPStep : xNStep) * cosAngle;
                    dz = (zp ? zPStep : zNStep) * sinAngle;

                    for (int j = 0; j <= sectionN; j++)
                    {
                        Engine3D.CopyPoint3D(x, h, z, ref mainVertex3D[k++], newMemory);
                        x += dx;
                        z += dz;
                        h += hStep;
                    }

                    angle += step;
                }

                if (bottomVisible) Engine3D.CopyPoint3D(0, -Height / 2, 0, ref mainVertex3D[k++], newMemory);
                if (topVisible) Engine3D.CopyPoint3D(0, Height / 2, 0, ref mainVertex3D[k++], newMemory);

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Cylinder3D", "BuildModel", er.Message, -1);
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
                if (BuildModel(_n, sectionN, TopVisible, BottomVisible,
                                RBottomXpositive, RBottomXnegative, RBottomZpositive, RBottomZnegative,
                                RTopXpositive, RTopXnegative, RTopZpositive, RTopZnegative,
                                AngleStart, AngleFinish,
                                Height) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Cylinder3D", "RebuildModel", er.Message, -1);
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
                polygon = new Polygon3D[2 * (N - 1) * sectionN + (TopVisible ? N - 1 : 0) + (BottomVisible ? N - 1 : 0)];
                int k = 0;

                // боковые грани
                for (int i = 0; i < N-1; i++)
                    for (int j = 1; j <= sectionN; j++)
                    {
                        polygon[k++] = new Polygon3D(i * (sectionN + 1) + j - 1, i * (sectionN + 1) + j, (i + 1) * (sectionN + 1) + j - 1);
                        polygon[k++] = new Polygon3D((i + 1) * (sectionN + 1) + j - 1, i * (sectionN + 1) + j, (i + 1) * (sectionN + 1) + j);
                    }

                //нижняя и верхняя грани
                int cB = (BottomVisible ? N * (sectionN + 1) : -1);
                int cT = (TopVisible ? N * (sectionN + 1) + (BottomVisible ? 1 : 0) : -1);

                if (cB >= 0)
                    for (int i = 0; i < N - 1; i++)
                        polygon[k++] = new Polygon3D(i * (sectionN + 1), (i + 1) * (sectionN + 1), cB);

                if (cT >= 0) 
                    for (int i = 0; i < N - 1; i++)
                        polygon[k++] = new Polygon3D((i + 1) * (sectionN + 1) + sectionN, i * (sectionN + 1) + sectionN, cT);

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Cylinder3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Cylinder3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Cylinder3D;
        }
    }
    #endregion

    #region Tor3D
    /// <summary>
    /// Вершинная модель тора
    /// </summary>
    public class Tor3D : VolumetricModel3D
    {
        /// <summary>
        /// Основной радиус по положительной оси OX
        /// </summary>
        public float RxPositive;
        /// <summary>
        /// Основной радиус по отрицательной оси OX
        /// </summary>
        public float RxNegative;
        /// <summary>
        /// Основной радиус по положительной оси OY
        /// </summary>
        public float RyPositive;
        /// <summary>
        /// Основной радиус по отрицательной оси OY
        /// </summary>
        public float RyNegative;
        /// <summary>
        /// Внутренний радиус по положительной оси OZ
        /// </summary>
        public float RADzPositive;
        /// <summary>
        /// Внутренний радиус по отрицательной оси OZ
        /// </summary>
        public float RADzNegative;

        /// <summary>
        /// Внутренний радиус по положительной части плоскости X
        /// </summary>
        public float RADxPositive;
        /// <summary>
        /// Внутренний радиус по отрицательной части плоскости X
        /// </summary>
        public float RADxNegative;

        /// <summary>
        /// Внутренний радиус по положительной части плоскости Y
        /// </summary>
        public float RADyPositive;
        /// <summary>
        /// Внутренний радиус по отрицательной части плоскости Y
        /// </summary>
        public float RADyNegative;

        /// <summary>
        /// Начальный угол расчета в плоскости XY
        /// </summary>
        public float StartAngleXY;
        /// <summary>
        /// Конечный угол расчета в плоскости XY
        /// </summary>
        public float FinishAngleXY;
        /// <summary>
        /// Начальный угол расчета в плоскости YZ
        /// </summary>
        public float StartAngleYZ;
        /// <summary>
        /// Конечный угол расчета в плоскости YZ
        /// </summary>
        public float FinishAngleYZ;

        /// <summary>
        /// Количество опорных вершин в плоскости 0YX
        /// </summary>
        public int Nx;

        /// <summary>
        /// Количество опорных вершин в плоскости 0YZ
        /// </summary>
        public int Nz;

        /// <summary>
        /// Создание модели тора
        /// </summary>
        /// <param name="nXY">Количество опорных вершин в плоскости XY</param>
        /// <param name="nYZ">Количество опорных вершин в плоскости YZ</param>
        /// <param name="rXpositive">Основной радиус по положительной оси OX</param>
        /// <param name="rXnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="rYpositive">Основной радиус по положительной оси OY</param>
        /// <param name="rYnegative">Основной радиус по отрицательной оси OY</param>
        /// <param name="radZpositive">Внутренний радиус по положительной оси OZ</param>
        /// <param name="radZnegative">Внутренний радиус по отрицательной оси OZ</param>
        /// <param name="radXYpositive">Внутренний радиус по положительной части плоскости XY</param>
        /// <param name="radXYnegative">Внутренний радиус по отрицательной части плоскости XY</param>
        /// <param name="startAngleXY">Начальный угол по плоскости 0XY</param>
        /// <param name="finishAngleXY">Конечный угол по плоскости 0XY</param>
        /// <param name="startAngleYZ">Начальный угол по плоскости 0YZ</param>
        /// <param name="finishAngleYZ">Конечный угол по плоскости 0YZ</param>
        public Tor3D(int nXY = 8, int nYZ = 8,
                     float rXpositive = 100, float rXnegative = 100,
                     float rYpositive = 100, float rYnegative = 100,
                     float radZpositive = 20, float radZnegative = 20,
                     float radXpositive = 20, float radXnegative = 20,
                     float radYpositive = 20, float radYnegative = 20,
                     float startAngleXY = 0, float finishAngleXY = Engine3D.Radian360,
                     float startAngleYZ = 0, float finishAngleYZ = Engine3D.Radian360)
        {
            Nx = 0;
            Nz = 0;
            ClosedSurface = (Math.Abs(finishAngleXY - startAngleXY - Engine3D.Radian360) < Engine3D.RadianDegrees) & (Math.Abs(finishAngleYZ - startAngleYZ - Engine3D.Radian360) < Engine3D.RadianDegrees);
            if (BuildModel(nXY, nYZ, rXpositive, rXnegative, rYpositive, rYnegative, radZpositive, radZnegative, radXpositive, radXnegative, radYpositive, radYnegative, startAngleXY, finishAngleXY, startAngleYZ, finishAngleYZ) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Tor3D(Tor3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            RxPositive = Template.RxPositive;
            RxNegative = Template.RxNegative;
            RyPositive = Template.RyPositive;
            RyNegative = Template.RyNegative;
            RADzPositive = Template.RADzPositive;
            RADzNegative = Template.RADzNegative;
            RADxPositive = Template.RADxPositive;
            RADxNegative = Template.RADxNegative;
            RADyPositive = Template.RADyPositive;
            RADyNegative = Template.RADyNegative;
            StartAngleXY = Template.StartAngleXY;
            FinishAngleXY = Template.FinishAngleXY;
            StartAngleYZ = Template.StartAngleYZ;
            FinishAngleYZ = Template.FinishAngleYZ;
            Nx = Template.Nx;
            Nz = Template.Nz;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public Tor3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
            //if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("Nx=" + Nx.ToString());
                file.WriteLine("Nz=" + Nz.ToString());
                file.WriteLine("RxPositive=" + RxPositive.ToString("0.0000", culture));
                file.WriteLine("RxNegative=" + RxNegative.ToString("0.0000", culture));
                file.WriteLine("RyPositive=" + RyPositive.ToString("0.0000", culture));
                file.WriteLine("RyNegative=" + RyNegative.ToString("0.0000", culture));
                file.WriteLine("RADxPositive=" + RADxPositive.ToString("0.0000", culture));
                file.WriteLine("RADxNegative=" + RADxNegative.ToString("0.0000", culture));
                file.WriteLine("RADyPositive=" + RADyPositive.ToString("0.0000", culture));
                file.WriteLine("RADyNegative=" + RADyNegative.ToString("0.0000", culture));
                file.WriteLine("RADzPositive=" + RADzPositive.ToString("0.0000", culture));
                file.WriteLine("RADzNegative=" + RADzNegative.ToString("0.0000", culture));
                file.WriteLine("StartAngleXY=" + StartAngleXY.ToString("0.0000", culture));
                file.WriteLine("FinishAngleXY=" + FinishAngleXY.ToString("0.0000", culture));
                file.WriteLine("StartAngleYZ=" + StartAngleYZ.ToString("0.0000", culture));
                file.WriteLine("FinishAngleYZ=" + FinishAngleYZ.ToString("0.0000", culture));

                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Tor3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "Nx":
                                                Nx = int.Parse(PropertyValue);
                                                break;
                                            case "Nz":
                                                Nz = int.Parse(PropertyValue);
                                                break;
                                            case "RxPositive":
                                                RxPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RxNegative":
                                                RxNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyPositive":
                                                RyPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyNegative":
                                                RyNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADxPositive":
                                                RADxPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADxNegative":
                                                RADxNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADyPositive":
                                                RADyPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADyNegative":
                                                RADyNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADzPositive":
                                                RADzPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RADzNegative":
                                                RADzNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "StartAngleXY":
                                                StartAngleXY = float.Parse(PropertyValue, culture);
                                                break;
                                            case "FinishAngleXY":
                                                FinishAngleXY = float.Parse(PropertyValue, culture);
                                                break;
                                            case "StartAngleYZ":
                                                StartAngleYZ = float.Parse(PropertyValue, culture);
                                                break;
                                            case "FinishAngleYZ":
                                                FinishAngleYZ = float.Parse(PropertyValue, culture);
                                                break;
                                        }

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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Tor3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Построение модели по заданным параметрам
        /// </summary>
        /// <param name="nXY">Количество опорных вершин в плоскости XY</param>
        /// <param name="nYZ">Количество опорных вершин в плоскости YZ</param>
        /// <param name="rXpositive">Основной радиус по положительной оси OX</param>
        /// <param name="rXnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="rYpositive">Основной радиус по положительной оси OY</param>
        /// <param name="rYnegative">Основной радиус по отрицательной оси OY</param>
        /// <param name="radZpositive">Внутренний радиус по положительной оси OZ</param>
        /// <param name="radZnegative">Внутренний радиус по отрицательной оси OZ</param>
        /// <param name="radXYpositive">Внутренний радиус по положительной части плоскости XY</param>
        /// <param name="radXYnegative">Внутренний радиус по отрицательной части плоскости XY</param>
        /// <param name="startAngleXY">Начальный угол по плоскости 0XY</param>
        /// <param name="finishAngleXY">Конечный угол по плоскости 0XY</param>
        /// <param name="startAngleYZ">Начальный угол по плоскости 0YZ</param>
        /// <param name="finishAngleYZ">Конечный угол по плоскости 0YZ</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int nXY = 8, int nYZ = 8,
                     float rXpositive = 100, float rXnegative = 100,
                     float rYpositive = 100, float rYnegative = 100,
                     float radZpositive = 20, float radZnegative = 20,
                     float radXpositive = 20, float radXnegative = 20,
                     float radYpositive = 20, float radYnegative = 20,
                     float startAngleXY = 0, float finishAngleXY = Engine3D.Radian360,
                     float startAngleYZ = 0, float finishAngleYZ = Engine3D.Radian360)
        {
            try
            {
                if (nXY < 2) throw new Exception("Количество опорных вершин в плоскости XY не может быть менее 2.");
                if (nYZ < 2) throw new Exception("Количество опорных вершин в плоскости YZ не может быть менее 2.");


                float stepXY = (finishAngleXY - startAngleXY) / (nXY - 1);
                float stepYZ = (finishAngleYZ - startAngleYZ) / (nYZ - 1);

                float angleXY;
                float angleYZ;

                bool newMemory = (Nx != nXY) | (Nz != nYZ);

                Nx = nXY;
                Nz = nYZ;
                RxPositive = rXpositive;
                RxNegative = rXnegative;
                RyPositive = rYpositive;
                RyNegative = rYnegative;
                RADzPositive = radZpositive;
                RADzNegative = radZnegative;
                RADxPositive = radXpositive;
                RADxNegative = radXnegative;
                RADyPositive = radYpositive;
                RADyNegative = radYnegative;
                StartAngleXY = startAngleXY;
                StartAngleYZ = startAngleYZ;
                FinishAngleXY = finishAngleXY;
                FinishAngleYZ = finishAngleYZ;

                if (newMemory) mainVertex3D = new Point3D[Nx * Nz];

                float Z;
                float dX;
                float dY;

                angleYZ = startAngleYZ;
                for (int i = 0; i < nYZ; i++)
                {
                    angleXY = startAngleXY;

                    Z  = (((Engine3D.GetNormalizeDegrees(angleYZ) < Engine3D.Radian90) | (Engine3D.GetNormalizeDegrees(angleYZ) > Engine3D.Radian270)) ? RADzPositive : RADzNegative) * (float)Math.Cos(angleYZ);
                    dX = ((Engine3D.GetNormalizeDegrees(angleYZ) < Engine3D.Radian180) ? RADxPositive : RADxNegative) * (float)Math.Sin(angleYZ);
                    dY = ((Engine3D.GetNormalizeDegrees(angleYZ) < Engine3D.Radian180) ? RADyPositive : RADyNegative) * (float)Math.Sin(angleYZ);

                    for (int j = 0; j < nXY; j++)
                    {
                        Engine3D.CopyPoint3D(
                            ((((Engine3D.GetNormalizeDegrees(angleXY) < Engine3D.Radian90) | (Engine3D.GetNormalizeDegrees(angleXY) > Engine3D.Radian270)) ? RxPositive : RxNegative) + dX) * (float)Math.Cos(angleXY),
                            (((Engine3D.GetNormalizeDegrees(angleXY) < Engine3D.Radian180) ? RyPositive : RyNegative) + dY) * (float)Math.Sin(angleXY),
                            Z, 
                            ref mainVertex3D[i * nXY + j], newMemory);
                        angleXY += stepXY;
                    }
                    angleYZ += stepYZ;
                }

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Tor3D", "BuildModel", er.Message, -1);
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
                int _n = Nx;
                if (newMemory) Nx = 0;
                if (BuildModel(_n, Nz, RxPositive, RxNegative, RyPositive, RyNegative, RADzPositive, RADzNegative, RADxPositive, RADxNegative, RADyPositive, RADyNegative, StartAngleXY, FinishAngleXY, StartAngleYZ, FinishAngleYZ) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Tor3D", "RebuildModel", er.Message, -1);
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
                int PolygonCount = (Nx - 1) * (Nz - 1) * 2;

                polygon = new Polygon3D[PolygonCount];
                int idx = 0;

                for (int i = 1; i < Nx; i++)
                    for (int j = 1; j < Nz; j++)
                    {
                        polygon[idx++] = new Polygon3D(i + Nx * j, i + Nx * (j - 1), i - 1 + Nx * (j - 1));
                        polygon[idx++] = new Polygon3D(i - 1 + Nx * (j - 1), i - 1 + Nx * j, i + Nx * j);
                    }

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Tor3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Tor3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Tor3D;
        }

    }
    #endregion

    #region Ellipse3D
    /// <summary>
    /// Вершинная модель эллипсоида
    /// </summary>
    public class Ellipse3D : VolumetricModel3D
    {
        /// <summary>
        /// Радиус по положительной оси OX
        /// </summary>
        public float RxPositive;
        /// <summary>
        /// Радиус по отрицательной оси OX
        /// </summary>
        public float RxNegative;
        /// <summary>
        /// Радиус по положительной оси OY
        /// </summary>
        public float RyPositive;
        /// <summary>
        /// Радиус по отрицательной оси OY
        /// </summary>
        public float RyNegative;
        /// <summary>
        /// Радиус по положительной оси OZ
        /// </summary>
        public float RzPositive;
        /// <summary>
        /// Радиус по отрицательной оси OZ
        /// </summary>
        public float RzNegative;
        /// <summary>
        /// Начальный угол расчета в плоскости XY
        /// </summary>
        public float StartAngleXY;
        /// <summary>
        /// Конечный угол расчета в плоскости XY
        /// </summary>
        public float FinishAngleXY;
        /// <summary>
        /// Начальный угол расчета в плоскости YZ
        /// </summary>
        public float StartAngleYZ;
        /// <summary>
        /// Конечный угол расчета в плоскости YZ
        /// </summary>
        public float FinishAngleYZ;

        /// <summary>
        /// Количество опорных вершин в плоскости 0YX
        /// </summary>
        public int Nx;

        /// <summary>
        /// Количество опорных вершин в плоскости 0YZ
        /// </summary>
        public int Nz;

        /// <summary>
        /// Создание модели эллипсоида
        /// </summary>
        /// <param name="nXY">Количество опорных вершин в плоскости XY</param>
        /// <param name="nYZ">Количество опорных вершин в плоскости YZ</param>
        /// <param name="rXpositive">Основной радиус по положительной оси OX</param>
        /// <param name="rXnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="rYpositive">Основной радиус по положительной оси OY</param>
        /// <param name="rYnegative">Основной радиус по отрицательной оси OY</param>
        /// <param name="rZpositive">Основной радиус по положительной оси OX</param>        
        /// <param name="rZnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="startAngleXY">Начальный угол по плоскости 0XY</param>
        /// <param name="finishAngleXY">Конечный угол по плоскости 0XY</param>
        /// <param name="startAngleYZ">Начальный угол по плоскости 0YZ(от 0 до PI радиан)</param>
        /// <param name="finishAngleYZ">Конечный угол по плоскости 0YZ(от 0 до PI радиан)</param>
        public Ellipse3D(int nXY = 8, int nYZ = 8,
                     float rXpositive = 100, float rXnegative = 100,
                     float rYpositive = 100, float rYnegative = 100,
                     float rZpositive = 100, float rZnegative = 100,
                     float startAngleXY = 0, float finishAngleXY = Engine3D.Radian360,
                     float startAngleYZ = 0, float finishAngleYZ = Engine3D.Radian180)
        {
            Nx = 0;
            Nz = 0;
            ClosedSurface = (Math.Abs(finishAngleXY - startAngleXY - Engine3D.Radian360) < Engine3D.RadianDegrees) & (Math.Abs(finishAngleYZ - startAngleYZ - Engine3D.Radian180) < Engine3D.RadianDegrees);
            if (BuildModel(nXY, nYZ, rXpositive, rXnegative, rYpositive, rYnegative, rZpositive, rZnegative, startAngleXY, finishAngleXY, startAngleYZ, finishAngleYZ) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Ellipse3D(Ellipse3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            RxPositive = Template.RxPositive;
            RxNegative = Template.RxNegative;
            RyPositive = Template.RyPositive;
            RyNegative = Template.RyNegative;
            RzPositive = Template.RzPositive;
            RzNegative = Template.RzNegative;
            StartAngleXY = Template.StartAngleXY;
            FinishAngleXY = Template.FinishAngleXY;
            StartAngleYZ = Template.StartAngleYZ;
            FinishAngleYZ = Template.FinishAngleYZ;
            Nx = Template.Nx;
            Nz = Template.Nz;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public Ellipse3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
            //if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }



        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("Nx=" + Nx.ToString());
                file.WriteLine("Nz=" + Nz.ToString());
                file.WriteLine("RxPositive=" + RxPositive.ToString("0.0000", culture));
                file.WriteLine("RxNegative=" + RxNegative.ToString("0.0000", culture));
                file.WriteLine("RyPositive=" + RyPositive.ToString("0.0000", culture));
                file.WriteLine("RyNegative=" + RyNegative.ToString("0.0000", culture));
                file.WriteLine("RzPositive=" + RzPositive.ToString("0.0000", culture));
                file.WriteLine("RzNegative=" + RzNegative.ToString("0.0000", culture));
                file.WriteLine("StartAngleXY=" + StartAngleXY.ToString("0.0000", culture));
                file.WriteLine("FinishAngleXY=" + FinishAngleXY.ToString("0.0000", culture));
                file.WriteLine("StartAngleYZ=" + StartAngleYZ.ToString("0.0000", culture));
                file.WriteLine("FinishAngleYZ=" + FinishAngleYZ.ToString("0.0000", culture));

                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Ellipse3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "Nx":
                                                Nx = int.Parse(PropertyValue);
                                                break;
                                            case "Nz":
                                                Nz = int.Parse(PropertyValue);
                                                break;
                                            case "RxPositive":
                                                RxPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RxNegative":
                                                RxNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyPositive":
                                                RyPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RyNegative":
                                                RyNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RzPositive":
                                                RzPositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RzNegative":
                                                RzNegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "StartAngleXY":
                                                StartAngleXY = float.Parse(PropertyValue, culture);
                                                break;
                                            case "FinishAngleXY":
                                                FinishAngleXY = float.Parse(PropertyValue, culture);
                                                break;
                                            case "StartAngleYZ":
                                                StartAngleYZ = float.Parse(PropertyValue, culture);
                                                break;
                                            case "FinishAngleYZ":
                                                FinishAngleYZ = float.Parse(PropertyValue, culture);
                                                break;
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Ellipse3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        

        /// <summary>
        /// Построение модели по заданным параметрам
        /// </summary>
        /// <param name="nXY">Количество опорных вершин в плоскости XY</param>
        /// <param name="nYZ">Количество опорных вершин в плоскости YZ</param>
        /// <param name="rXpositive">Основной радиус по положительной оси OX</param>
        /// <param name="rXnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="rYpositive">Основной радиус по положительной оси OY</param>
        /// <param name="rYnegative">Основной радиус по отрицательной оси OY</param>
        /// <param name="rZpositive">Основной радиус по положительной оси OX</param>        
        /// <param name="rZnegative">Основной радиус по отрицательной оси OX</param>
        /// <param name="startAngleXY">Начальный угол по плоскости 0XY</param>
        /// <param name="finishAngleXY">Конечный угол по плоскости 0XY</param>
        /// <param name="startAngleYZ">Начальный угол по плоскости 0YZ</param>
        /// <param name="finishAngleYZ">Конечный угол по плоскости 0YZ</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int nXY = 8, int nYZ = 8,
                     float rXpositive = 100, float rXnegative = 100,
                     float rYpositive = 100, float rYnegative = 100,
                     float rZpositive = 100, float rZnegative = 100,
                     float startAngleXY = 0, float finishAngleXY = Engine3D.Radian360,
                     float startAngleYZ = 0, float finishAngleYZ = Engine3D.Radian180)
        {
            try
            {
                if (nXY < 3) throw new Exception("Количество опорных вершин в плоскости XY не может быть менее 3.");
                if (nYZ < 3) throw new Exception("Количество опорных вершин в плоскости YZ не может быть менее 3.");


                float stepXY = (finishAngleXY - startAngleXY) / (nXY - 1);
                float stepYZ = (finishAngleYZ - startAngleYZ) / (nYZ - 1);

                float angleXY;
                float angleYZ;

                bool newMemory = (Nx != nXY) | (Nz != nYZ);

                Nx = nXY;
                Nz = nYZ;
                RxPositive = rXpositive;
                RxNegative = rXnegative;
                RyPositive = rYpositive;
                RyNegative = rYnegative;
                RzPositive = rZpositive;
                RzNegative = rZnegative;
                StartAngleXY = startAngleXY;
                StartAngleYZ = startAngleYZ;
                FinishAngleXY = finishAngleXY;
                FinishAngleYZ = finishAngleYZ;

                if (newMemory) mainVertex3D = new Point3D[Nx * Nz];

                float Z;
                float rSin;
                angleYZ = startAngleYZ;
                int j;

                for (int i = 0; i < nYZ; i++)
                {
                    angleXY = startAngleXY;

                    Z = (((Engine3D.GetNormalizeDegrees(angleYZ) < Engine3D.Radian90) | (Engine3D.GetNormalizeDegrees(angleYZ) > Engine3D.Radian270)) ? RzPositive : RzNegative) * (float)Math.Cos(angleYZ);
                    rSin = (float)Math.Sin(angleYZ);

                    for (j = 0; j < nXY; j++)
                    {
                        Engine3D.CopyPoint3D(((((Engine3D.GetNormalizeDegrees(angleXY) < Engine3D.Radian90) | (Engine3D.GetNormalizeDegrees(angleXY) > Engine3D.Radian270)) ? RxPositive : RxNegative)) * (float)Math.Cos(angleXY) * rSin,
                            (((Engine3D.GetNormalizeDegrees(angleXY) < Engine3D.Radian180) ? RyPositive : RyNegative)) * (float)Math.Sin(angleXY) * rSin,
                            Z,
                            ref mainVertex3D[i * nXY + j], newMemory);
                        angleXY += stepXY;
                    }
                    angleYZ += stepYZ;
                }

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Ellipse3D", "BuildModel", er.Message, -1);
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
                int _n = Nx;
                if (newMemory) Nx = 0;
                if (BuildModel(_n, Nz, RxPositive, RxNegative, RyPositive, RyNegative, RzPositive, RzNegative, StartAngleXY, FinishAngleXY, StartAngleYZ, FinishAngleYZ) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Ellipse3D", "RebuildModel", er.Message, -1);
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
                int PolygonCount = (Nx - 1) * (Nz - 1) * 2;

                polygon = new Polygon3D[PolygonCount];
                int idx = 0;

                for (int i = 1; i < Nx; i++)
                    for (int j = 1; j < Nz; j++)
                    {
                        polygon[idx++] = new Polygon3D(i + Nx * j, i + Nx * (j - 1), i - 1 + Nx * (j - 1));
                        polygon[idx++] = new Polygon3D(i - 1 + Nx * (j - 1), i - 1 + Nx * j, i + Nx * j);
                    }

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Ellipse3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Ellipse3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Ellipse3D;
        }
    }
    #endregion    

    #region Prism3D
    /// <summary>
    /// Вершинная модель составной призмы
    /// </summary>
    public class Prism3D : VolumetricModel3D
    {
        public float Height;
        public float Width;
        public float Depth;
        public bool TopVisible;
        public bool BottomVisible;
        public bool LeftVisible;
        public bool RightVisible;
        public bool FrontVisible;
        public bool RearVisible;
        public int nHeight;
        public int nWidth;
        public int nDepth;

        /// <summary>
        /// Создание модели составной призмы
        /// </summary>
        /// <param name="height">Высота</param>
        /// <param name="width">Ширина</param>
        /// <param name="depth">Глубина</param>
        /// <param name="topVisible">Признак наличия верхней крышки</param>
        /// <param name="bottomVisible">Признак наличия нижней крышки</param>
        /// <param name="leftVisible">Признак наличия левой крышки</param>
        /// <param name="rightVisible">Признак наличия правой крышки</param>
        /// <param name="frontVisible">Признак наличия передней крышки</param>
        /// <param name="rearVisible">Признак наличия задней крышки</param>        
        public Prism3D(int N_Height = 1, int N_Width = 1, int N_Depth = 1, float height = 100, float width = 100, float depth = 100, bool topVisible = true, bool bottomVisible = true, bool leftVisible = true, bool rightVisible = true, bool frontVisible = true, bool rearVisible = true)
        {
            TopVisible = true;
            BottomVisible = true;
            LeftVisible = true;
            RightVisible = true;
            FrontVisible = true;
            RearVisible = true;
            nHeight = 0;
            nWidth = 0;
            nDepth = 0;
            ClosedSurface = TopVisible & BottomVisible & LeftVisible & RightVisible & FrontVisible & RearVisible;
            if (BuildModel(N_Height, N_Width, N_Depth, height, width, depth, topVisible, bottomVisible, leftVisible, rightVisible, frontVisible, rearVisible) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public Prism3D(Prism3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            Height = Template.Height;
            Width = Template.Width;
            Depth = Template.Depth;
            TopVisible = Template.TopVisible;
            BottomVisible = Template.BottomVisible;
            LeftVisible = Template.LeftVisible;
            RightVisible = Template.RightVisible;
            FrontVisible = Template.FrontVisible;
            RearVisible = Template.RearVisible;
            nHeight = Template.nHeight;
            nWidth = Template.nWidth;
            nDepth = Template.nDepth;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public Prism3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
            //if (RebuildModel() != 0) throw new Exception(ErrorLog.GetLastError());
        }


        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("nHeight=" + nHeight.ToString());
                file.WriteLine("nWidth=" + nWidth.ToString());
                file.WriteLine("nDepth=" + nDepth.ToString());
                file.WriteLine("Height=" + Height.ToString("0.0000", culture));
                file.WriteLine("Width=" + Width.ToString("0.0000", culture));
                file.WriteLine("Depth=" + Depth.ToString("0.0000", culture));
                file.WriteLine("TopVisible=" + (TopVisible ? "1" : "0"));
                file.WriteLine("BottomVisible=" + (BottomVisible ? "1" : "0"));
                file.WriteLine("LeftVisible=" + (LeftVisible ? "1" : "0"));
                file.WriteLine("RightVisible=" + (RightVisible ? "1" : "0"));
                file.WriteLine("FrontVisible=" + (FrontVisible ? "1" : "0"));
                file.WriteLine("RearVisible=" + (RearVisible ? "1" : "0"));
                
                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;

                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "nHeight":
                                                nHeight = int.Parse(PropertyValue);
                                                break;
                                            case "nWidth":
                                                nWidth = int.Parse(PropertyValue);
                                                break;
                                            case "nDepth":
                                                nDepth = int.Parse(PropertyValue);
                                                break;
                                            case "Height":
                                                Height = float.Parse(PropertyValue, culture);
                                                break;
                                            case "Width":
                                                Width = float.Parse(PropertyValue, culture);
                                                break;
                                            case "Depth":
                                                Depth = float.Parse(PropertyValue, culture);
                                                break;

                                            case "TopVisible":
                                                TopVisible = (PropertyValue == "1");
                                                break;
                                            case "BottomVisible":
                                                BottomVisible = (PropertyValue == "1");
                                                break;
                                            case "LeftVisible":
                                                LeftVisible = (PropertyValue == "1");
                                                break;
                                            case "RightVisible":
                                                RightVisible = (PropertyValue == "1");
                                                break;
                                            case "FrontVisible":
                                                FrontVisible = (PropertyValue == "1");
                                                break;
                                            case "RearVisible":
                                                RearVisible = (PropertyValue == "1");
                                                break;
                                            
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Построение модели призмы по заданным параметрам.
        /// </summary>
        /// <param name="height">Высота</param>
        /// <param name="width">Ширина</param>
        /// <param name="depth">Глубина</param>
        /// <param name="topVisible">Признак наличия верхней крышки</param>
        /// <param name="bottomVisible">Признак наличия нижней крышки</param>
        /// <param name="leftVisible">Признак наличия левой крышки</param>
        /// <param name="rightVisible">Признак наличия правой крышки</param>
        /// <param name="frontVisible">Признак наличия передней крышки</param>
        /// <param name="rearVisible">Признак наличия задней крышки</param>    
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int N_Height = 1, int N_Width = 1, int N_Depth = 1,
                              float height = 100, float width = 100, float depth = 100, 
                              bool topVisible = true, bool bottomVisible = true, bool leftVisible = true, bool rightVisible = true, bool frontVisible = true, bool rearVisible = true)
        {
            try
            {
                if ((N_Depth < 1) | (N_Height < 1) | (N_Width < 1)) throw new Exception("Размерность по каждой грани должна быть не менее 1");

                bool newMemory = true;

                TopVisible = topVisible;
                BottomVisible = bottomVisible;
                LeftVisible = leftVisible;
                RightVisible = rightVisible;
                FrontVisible = frontVisible;
                RearVisible = rearVisible;
                Height = height;
                Width = width;
                Depth = depth;
                nHeight = N_Height;
                nWidth = N_Width;
                nDepth = N_Depth;

                if (newMemory) mainVertex3D = new Point3D[(nWidth + 1) * (nHeight + 1) * ((FrontVisible ? 1 : 0) + (RearVisible ? 1 : 0)) +
                                                          (nDepth + 1) * (nHeight + 1) * ((LeftVisible ? 1 : 0) + (RightVisible ? 1 : 0)) +
                                                          (nWidth + 1) * (nDepth + 1) * ((BottomVisible ? 1 : 0) + (TopVisible ? 1 : 0))];

                float dW = ((float)Width / nWidth);
                float dH = ((float)Height / nHeight);
                float dD = ((float)Depth / nDepth);
                int i;
                int j;

                int k = 0;

                //передняя грань
                float W = -(float)Width / 2;
                float H = -(float)Height / 2;
                float D = -(float)Depth / 2;

                if (FrontVisible)
                    for (i = 0; i <= nWidth; i++)
                    {
                        for (j = 0; j <= nHeight; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            H += dH;
                        }
                        H = -Height / 2;
                        W += dW;
                    }

                // задняя грань
                if (RearVisible)
                {
                    W = Width / 2;
                    H = -Height / 2;
                    D = Depth / 2;
                    for (i = 0; i <= nWidth; i++)
                    {
                        for (j = 0; j <= nHeight; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            H += dH;
                        }
                        H = -Height / 2;
                        W -= dW;
                    }
                }

                //левая грань
                if (LeftVisible)
                {
                    W = -Width / 2;
                    H = -Height / 2;
                    D = -Depth / 2;
                    for (i = 0; i <= nDepth; i++)
                    {
                        for (j = 0; j <= nHeight; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            H += dH;
                        }
                        H = -Height / 2;
                        D += dD;
                    }
                }

                //правая грань
                if (RightVisible)
                {
                    W = Width / 2;
                    H = -Height / 2;
                    D = Depth / 2;
                    for (i = 0; i <= nDepth; i++)
                    {
                        for (j = 0; j <= nHeight; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            H += dH;
                        }
                        H = -Height / 2;
                        D -= dD;
                    }
                }

                //нижняя грань
                if (BottomVisible)
                {
                    W = -Width / 2;
                    H = -Height / 2;
                    D = Depth / 2;
                    for (i = 0; i <= nWidth; i++)
                    {
                        for (j = 0; j <= nDepth; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            D -= dD;
                        }
                        D = Depth / 2;
                        W += dW;
                    }
                }

                //верхняя грань
                if (TopVisible)
                {
                    W = -Width / 2;
                    H = Height / 2;
                    D = -Depth / 2;
                    for (i = 0; i <= nWidth; i++)
                    {
                        for (j = 0; j <= nDepth; j++)
                        {
                            Engine3D.CopyPoint3D(W, H, D, ref mainVertex3D[k++], newMemory);
                            D += dD;
                        }
                        D = -Depth / 2;
                        W += dW;
                    }
                }

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "BuildModel", er.Message, -1);
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
                int _n = nHeight;
                if (newMemory) nHeight = 0;
                if (BuildModel(_n, nWidth, nDepth, Height, Width, Depth, TopVisible, BottomVisible, LeftVisible, RightVisible, FrontVisible, RearVisible) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "RebuildModel", er.Message, -1);
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
                int N = (TopVisible ? (nDepth * nWidth * 2) : 0) + (BottomVisible ? (nDepth * nWidth * 2) : 0) +
                        (LeftVisible ? (nDepth * nHeight * 2) : 0) + (RightVisible ? (nDepth * nHeight * 2) : 0) +
                        (FrontVisible ? (nWidth * nHeight * 2) : 0) + (RearVisible ? (nWidth * nHeight * 2) : 0);
                if (N == 0) throw new Exception("Модель должна содержать хотя бы одну видимую грань.");

                polygon = new Polygon3D[N];

                int k = 0;
                int firstIndex = 0;
                int i;
                int j;

                if (FrontVisible)
                {
                    for (j = 0; j < nWidth; j++)
                    {
                        for (i = 0; i < nHeight; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i + nHeight + 1, firstIndex + i, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + 1, firstIndex + i + nHeight + 2, firstIndex + i + nHeight + 1);
                        }
                        firstIndex += (nHeight + 1);
                    }
                    firstIndex += (nHeight + 1);
                }

                if (RearVisible)
                {
                    for (j = 0; j < nWidth; j++)
                    {
                        for (i = 0; i < nHeight; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i + nHeight + 1, firstIndex + i, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + 1, firstIndex + i + nHeight + 2, firstIndex + i + nHeight + 1);
                        }
                        firstIndex += (nHeight + 1);
                    }
                    firstIndex += (nHeight + 1);
                }

                if (LeftVisible)
                {
                    for (j = 0; j < nDepth; j++)
                    {
                        for (i = 0; i < nHeight; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i, firstIndex + i + nHeight + 1, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + nHeight + 2, firstIndex + i + 1, firstIndex + i + nHeight + 1);
                        }
                        firstIndex += (nHeight + 1);
                    }
                    firstIndex += (nHeight + 1);
                }

                if (RightVisible)
                {
                    for (j = 0; j < nDepth; j++)
                    {
                        for (i = 0; i < nHeight; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i, firstIndex + i + nHeight + 1, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + nHeight + 2, firstIndex + i + 1, firstIndex + i + nHeight + 1);
                        }
                        firstIndex += (nHeight + 1);
                    }
                    firstIndex += (nHeight + 1);
                }

                if (BottomVisible)
                {
                    for (j = 0; j < nWidth; j++)
                    {
                        for (i = 0; i < nDepth; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i + nDepth + 1, firstIndex + i, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + 1, firstIndex + i + nDepth + 2, firstIndex + i + nDepth + 1);
                        }
                        firstIndex += (nDepth + 1);
                    }
                    firstIndex += (nDepth + 1);
                }

                if (TopVisible)
                {
                    for (j = 0; j < nWidth; j++)
                    {
                        for (i = 0; i < nDepth; i++)
                        {
                            polygon[k++] = new Polygon3D(firstIndex + i + nDepth + 1, firstIndex + i, firstIndex + i + 1);
                            polygon[k++] = new Polygon3D(firstIndex + i + 1, firstIndex + i + nDepth + 2, firstIndex + i + nDepth + 1);
                        }
                        firstIndex += (nDepth + 1);
                    }
                    firstIndex += (nDepth + 1);
                }

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "Prism3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.Prism3D;
        }


        /// <summary>
        /// Установка цвета граней призмы. Нумерация начинается с 0.
        /// </summary>
        /// <param name="color">Цвет</param>
        /// <param name="CylinderSideNumber">Номер грани</param>
        /// <param name="PolygonSide">Окрашиваемые стороны полигонов</param>
        /// <returns></returns>
        public int SetPrismSideColor(Color color, int PrismSideNumber = 0, PolygonSides PolygonSide = PolygonSides.AllSides)
        {
            try
            {
                int N = (FrontVisible ? 1 : 0) + (RearVisible ? 1 : 0) +
                        (LeftVisible ? 1 : 0) + (RightVisible ? 1 : 0) +
                        (TopVisible ? 1 : 0) + (BottomVisible ? 1 : 0);
                if ((PrismSideNumber < 0) | (PrismSideNumber >= N)) throw new Exception("Указан некорректный номер грани.");

                int sideNumber = 0;
                int startIdx = 0;

                if (FrontVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nWidth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (RearVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nWidth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (LeftVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nDepth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (RightVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nDepth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (BottomVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nWidth * nDepth * 2) - 1, PolygonSide);
                    startIdx += nWidth * nDepth * 2;
                    sideNumber++;
                }

                if (TopVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx, startIdx + (nWidth * nDepth * 2) - 1, PolygonSide);
                    //startIdx += nWidth * nDepth * 2;
                    //sideNumber++;
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "SetPrismSideColor", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Установка цвета граней призмы. Нумерация начинается с 0.
        /// </summary>
        /// <param name="matte">Матовость</param>
        /// <param name="PrismSideNumber">Номер грани</param>
        /// <param name="PolygonSide">Окрашиваемые стороны полигонов</param>
        /// <returns></returns>
        public int SetPrismSideMatte(float matte, int PrismSideNumber = 0, PolygonSides PolygonSide = PolygonSides.AllSides)
        {
            try
            {
                int N = (FrontVisible ? 1 : 0) + (RearVisible ? 1 : 0) +
                        (LeftVisible ? 1 : 0) + (RightVisible ? 1 : 0) +
                        (TopVisible ? 1 : 0) + (BottomVisible ? 1 : 0);
                if ((PrismSideNumber < 0) | (PrismSideNumber >= N)) throw new Exception("Указан некорректный номер грани.");

                int sideNumber = 0;
                int startIdx = 0;

                if (FrontVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nWidth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (RearVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nWidth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (LeftVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nDepth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (RightVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nDepth * nHeight * 2) - 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (BottomVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nWidth * nDepth * 2) - 1, PolygonSide);
                    startIdx += nWidth * nDepth * 2;
                    sideNumber++;
                }

                if (TopVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx, startIdx + (nWidth * nDepth * 2) - 1, PolygonSide);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "SetPrismSideColor", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Установка цвета клетки на грани призмы. Нумерация начинается с 0.
        /// </summary>
        /// <param name="color">Цвет</param>
        /// <param name="CylinderSideNumber">Номер грани</param>
        /// <param name="PolygonSide">Окрашиваемые стороны полигонов</param>
        /// <returns></returns>
        public int SetPrismCellColor(Color color, int PrismSideNumber = 0, int CellCol = 0, int CellRow = 0, PolygonSides PolygonSide = PolygonSides.AllSides)
        {
            try
            {
                int N = (FrontVisible ? 1 : 0) + (RearVisible ? 1 : 0) +
                        (LeftVisible ? 1 : 0) + (RightVisible ? 1 : 0) +
                        (TopVisible ? 1 : 0) + (BottomVisible ? 1 : 0);
                if ((PrismSideNumber < 0) | (PrismSideNumber >= N)) throw new Exception("Указан некорректный номер грани.");

                int sideNumber = 0;
                int startIdx = 0;

                if (FrontVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (RearVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (LeftVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (RightVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (BottomVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nDepth * CellCol * 2 + CellRow * 2, startIdx + nDepth * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nDepth * 2;
                    sideNumber++;
                }

                if (TopVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetColor(color, startIdx + nDepth * CellCol * 2 + CellRow * 2, startIdx + nDepth * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "SetPrismCellColor", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Установка матовости клетки на грани призмы. Нумерация начинается с 0.
        /// </summary>
        /// <param name="matte">матовость</param>
        /// <param name="CylinderSideNumber">Номер грани</param>
        /// <param name="PolygonSide">Окрашиваемые стороны полигонов</param>
        /// <returns></returns>
        public int SetPrismCellMatte(float matte, int PrismSideNumber = 0, int CellCol = 0, int CellRow = 0, PolygonSides PolygonSide = PolygonSides.AllSides)
        {
            try
            {
                int N = (FrontVisible ? 1 : 0) + (RearVisible ? 1 : 0) +
                        (LeftVisible ? 1 : 0) + (RightVisible ? 1 : 0) +
                        (TopVisible ? 1 : 0) + (BottomVisible ? 1 : 0);
                if ((PrismSideNumber < 0) | (PrismSideNumber >= N)) throw new Exception("Указан некорректный номер грани.");

                int sideNumber = 0;
                int startIdx = 0;

                if (FrontVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (RearVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nHeight * 2;
                    sideNumber++;
                }

                if (LeftVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (RightVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nHeight * CellCol * 2 + CellRow * 2, startIdx + nHeight * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nDepth * nHeight * 2;
                    sideNumber++;
                }

                if (BottomVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nDepth * CellCol * 2 + CellRow * 2, startIdx + nDepth * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                    startIdx += nWidth * nDepth * 2;
                    sideNumber++;
                }

                if (TopVisible)
                {
                    if (sideNumber == PrismSideNumber) return SetMatte(matte, startIdx + nDepth * CellCol * 2 + CellRow * 2, startIdx + nDepth * CellCol * 2 + CellRow * 2 + 1, PolygonSide);
                }

                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("Prism3D", "SetPrismCellMatte", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion

    #region SurfaceHole3D
    /// <summary>
    /// Вершинная модель поверхности с эллиптической дыркой
    /// </summary>
    public class SurfaceHole3D : VolumetricModel3D
    {
        public float RXpositive;
        public float RXnegative;
        public float RZpositive;
        public float RZnegative;
        public int Nx;
        public int Nz;

        /// <summary>
        /// Создание модели поверхности с эллиптической дыркой
        /// </summary>
        /// <param name="nX">Количество опорных вершин по оси OX</param>
        /// <param name="nZ">Количество опорных вершин по оси OZ</param>
        /// <param name="rXpositive">Радиус по положительной части оси 0X</param>
        /// <param name="rXnegative">Радиус по отрицательной части оси 0X</param>
        /// <param name="rZpositive">Радиус по положительной части оси 0Z</param>
        /// <param name="rZnegative">Радиус по отрицательной части оси 0Z</param>
        public SurfaceHole3D(int nX = 2, int nZ = 2, 
                            float rXpositive = 10, float rXnegative = 10,
                            float rZpositive = 10, float rZnegative = 10)
        {
            Nx = 0;
            Nz = 0;
            ClosedSurface = false;
            if (BuildModel(nX,nZ,rXpositive,rXnegative,rZpositive,rZnegative) != 0) throw new Exception("Ошибка при построении модели объекта.");
        }

        /// <summary>
        /// Создание экземпляра по шаблону
        /// </summary>
        public SurfaceHole3D(SurfaceHole3D Template)
        {
            if (CreateModelFrom(Template) != 0) throw new Exception(ErrorLog.GetLastError());
            Nx = Template.Nx;
            Nz = Template.Nz;
            RXpositive = Template.RXpositive;
            RXnegative = Template.RXnegative;
            RZpositive = Template.RZpositive;
            RZnegative = Template.RZnegative;
            ClosedSurface = Template.ClosedSurface;
        }

        /// <summary>
        /// Создание из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        public SurfaceHole3D(string FileName)
        {
            if (OpenModelFromFile(FileName) != 0) throw new Exception(ErrorLog.GetLastError());
        }

        /// <summary>
        /// Сохранение модели в файл
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int SaveModelToFile(string FileName)
        {
            try
            {
                if (base.SaveModelToFile(FileName, this.ModelType()) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamWriter file = new StreamWriter(FileName, true, System.Text.Encoding.Default);
                CultureInfo culture = CultureInfo.InvariantCulture;

                file.WriteLine("Nx=" + Nx.ToString());
                file.WriteLine("Nz=" + Nz.ToString());
                file.WriteLine("RXpositive=" + RXpositive.ToString("0.0000", culture));
                file.WriteLine("RXnegative=" + RXnegative.ToString("0.0000", culture));
                file.WriteLine("RZpositive=" + RZpositive.ToString("0.0000", culture));
                file.WriteLine("RZnegative=" + RZnegative.ToString("0.0000", culture));

                file.Close();
                file.Dispose();
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("SurfaceHole3D", "SaveModelToFile", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Загрузка модели из файла
        /// </summary>
        /// <param name="FileName">Имя файла</param>
        /// <returns></returns>
        public virtual int OpenModelFromFile(string FileName)
        {
            try
            {
                ModelTypes fileModelType = ModelTypes.NONE;
                if (base.OpenModelFromFile(FileName, ref fileModelType) != 0) throw new Exception(ErrorLog.GetLastError());
                StreamReader file = null;
                CultureInfo culture = CultureInfo.InvariantCulture;
                try
                {
                    string sLine = "";
                    int currentMode = -1;
                    file = new StreamReader(FileName, System.Text.Encoding.Default);

                    while (!file.EndOfStream)
                    {
                        sLine = file.ReadLine().Trim();
                        if (sLine == "") continue;
                        if (sLine.Substring(0, 1) == "*") continue;

                        switch (sLine)
                        {
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

                        switch (currentMode)
                        {
                            case 3:
                                {
                                    int semIdx = sLine.IndexOf("=");
                                    if (semIdx > 0)
                                    {
                                        string PropertyName = sLine.Substring(0, semIdx).Trim();
                                        string PropertyValue = sLine.Substring(semIdx + 1, sLine.Length - semIdx - 1).Trim();
                                        if ((PropertyName == "") | (PropertyValue == "")) throw new Exception("Неизвестный формата файла: некорретный формат свойства: " + sLine);

                                        switch (PropertyName)
                                        {
                                            case "Nx":
                                                Nx = int.Parse(PropertyValue);
                                                break;
                                            case "Nz":
                                                Nz = int.Parse(PropertyValue);
                                                break;
                                            case "RXpositive":
                                                RXpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RXnegative":
                                                RXnegative = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RZpositive":
                                                RZpositive = float.Parse(PropertyValue, culture);
                                                break;
                                            case "RZnegative":
                                                RZnegative = float.Parse(PropertyValue, culture);
                                                break;
                                        }
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
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("SurfaceHole3D", "OpenModelFromFile", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Построение модели по заданным параметрам.
        /// </summary>
        /// <param name="nX">Количество опорных вершин по оси OX</param>
        /// <param name="nZ">Количество опорных вершин по оси OZ</param>
        /// <param name="rXpositive">Радиус по положительной части оси 0X</param>
        /// <param name="rXnegative">Радиус по отрицательной части оси 0X</param>
        /// <param name="rZpositive">Радиус по положительной части оси 0Z</param>
        /// <param name="rZnegative">Радиус по отрицательной части оси 0Z</param>
        /// <returns>Возвращает 0: успешное построение, не 0 - ошибка.</returns>
        public int BuildModel(int nX = 2, int nZ = 2,
                            float rXpositive = 10, float rXnegative = 10,
                            float rZpositive = 10, float rZnegative = 10)
        {
            try
            {
                if ((nX < 2) | (nZ < 2)) throw new Exception("Модель не может содержать менее 2-х опорных точек на ось.");
                

                bool newMemory = (Nx != nX) | (Nz != nZ);

                float angle = 0;
                float angleStep = 0;
                float cosAngle;
                float sinAngle;
                float step = 0;

                int k = 0;

                Nx = nX;
                Nz = nZ;
                RXpositive = rXpositive;
                RXnegative = rXnegative;
                RZpositive = rZpositive;
                RZnegative = rZnegative;

                float xE = 0;
                float zE = 0;
                float xS = 0;
                float zS = 0;                

                if (newMemory) mainVertex3D = new Point3D[(Nx+Nz-2) * 4];

                #region Правая сторона
                angle = -Engine3D.RadianDegrees * 45;
                step = (RZpositive + RZnegative) / (Nz - 1);
                angleStep = Engine3D.Radian90 / (Nz - 1);
                zS=-RZnegative;
                xS = RXpositive;                    
                for (int i = 0; i < Nz; i++)
                {
                    cosAngle = (float)Math.Cos(angle);
                    sinAngle = (float)Math.Sin(angle);

                    xE = RXpositive * cosAngle;
                    zE = (sinAngle > 0 ? RZpositive : RZnegative) * sinAngle;
                    
                    Engine3D.CopyPoint3D(xE, 0, zE, ref mainVertex3D[k++], newMemory);
                    Engine3D.CopyPoint3D(xS, 0, zS, ref mainVertex3D[k++], newMemory);

                    zS += step;
                    angle += angleStep;
                }
                #endregion

                #region Верхняя сторона за вычетом конечных точек
                angle = Engine3D.RadianDegrees * 45;
                step = (RXpositive + RXnegative) / (Nx - 1);
                angleStep = Engine3D.Radian90 / (Nx - 1);
                zS = RZpositive;
                xS = RXpositive;
                xS -= step;
                angle += angleStep;
                for (int i = 1; i < Nx-1; i++)
                {
                    cosAngle = (float)Math.Cos(angle);
                    sinAngle = (float)Math.Sin(angle);

                    xE = (cosAngle > 0 ? RXpositive : RXnegative) * cosAngle;
                    zE = RZpositive * sinAngle;

                    Engine3D.CopyPoint3D(xE, 0, zE, ref mainVertex3D[k++], newMemory);
                    Engine3D.CopyPoint3D(xS, 0, zS, ref mainVertex3D[k++], newMemory);

                    xS -= step;
                    angle += angleStep;
                }
                #endregion

                #region Левая сторона
                angle = Engine3D.RadianDegrees * 135;
                step = (RZpositive + RZnegative) / (Nz - 1);
                angleStep = Engine3D.Radian90 / (Nz - 1);
                zS = RZpositive;
                xS = -RXnegative;
                for (int i = 0; i < Nz; i++)
                {
                    cosAngle = (float)Math.Cos(angle);
                    sinAngle = (float)Math.Sin(angle);

                    xE = RXnegative * cosAngle;
                    zE = (sinAngle > 0 ? RZpositive : RZnegative) * sinAngle;

                    Engine3D.CopyPoint3D(xE, 0, zE, ref mainVertex3D[k++], newMemory);
                    Engine3D.CopyPoint3D(xS, 0, zS, ref mainVertex3D[k++], newMemory);

                    zS -= step;
                    angle += angleStep;
                }
                #endregion

                #region Нижняя сторона за вычетом конечных точек
                angle = Engine3D.RadianDegrees * 225;
                step = (RXpositive + RXnegative) / (Nx - 1);
                angleStep = Engine3D.Radian90 / (Nx - 1);
                zS = -RZnegative;
                xS = -RXnegative;
                xS += step;
                angle += angleStep;
                for (int i = 1; i < Nx - 1; i++)
                {
                    cosAngle = (float)Math.Cos(angle);
                    sinAngle = (float)Math.Sin(angle);

                    xE = (cosAngle > 0 ? RXpositive : RXnegative) * cosAngle;
                    zE = RZnegative * sinAngle;

                    Engine3D.CopyPoint3D(xE, 0, zE, ref mainVertex3D[k++], newMemory);
                    Engine3D.CopyPoint3D(xS, 0, zS, ref mainVertex3D[k++], newMemory);

                    xS += step;
                    angle += angleStep;
                }
                #endregion

                return CreateDeformVertexBuffers(newMemory);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("SurfaceHole3D", "BuildModel", er.Message, -1);
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
                int _n = Nx;
                if (newMemory) Nx = 0;
                if (BuildModel(_n, Nz, RXpositive, RXnegative, RZpositive, RZnegative) != 0) throw new Exception(ErrorLog.GetLastError());
                if (RebuildPolygonMap() != 0) throw new Exception(ErrorLog.GetLastError());
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("SurfaceHole3D", "RebuildModel", er.Message, -1);
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
                polygon = new Polygon3D[(Nx + Nz-2)*4];
                int k = 0;

                for (int i = 0; i < (Nx + Nz - 2) * 2-1; i++)
                {
                    polygon[k++] = new Polygon3D(i * 2 + 1, i * 2, (i + 1) * 2 + 1);
                    polygon[k++] = new Polygon3D((i + 1) * 2, (i + 1) * 2 + 1, i * 2);
                }
                polygon[k++] = new Polygon3D(mainVertex3D.Length - 1, mainVertex3D.Length - 2, 1);
                polygon[k++] = new Polygon3D(0, 1, mainVertex3D.Length - 2);

                activePolygonIndexes = new int[polygon.Length];
                return ResetActivePolygonIndexes();
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("SurfaceHole3D", "CreatePolygonMap", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Имя класса
        /// </summary>
        /// <returns></returns>
        public new string ToString()
        {
            return "SurfaceHole3D";
        }

        /// <summary>
        /// Тип модели
        /// </summary>
        /// <returns></returns>
        public override ModelTypes ModelType()
        {
            return ModelTypes.SurfaceHole3D;
        }
    }
    #endregion
}