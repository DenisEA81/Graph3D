using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading;

namespace Graph3DLibrary
{
    #region PolygonCollectionInfo
    /// <summary>
    /// Информация о полигоне в коллекции моделей
    /// </summary>
    public struct PolygonCollectionInfo
    {
        public int ModelIndex;
        public int PolygonIndex;
    }
    #endregion

    #region ModelCollectionController
    /// <summary>
    /// Класс для совместной обработки коллекции моделей
    /// </summary>
    public class ModelCollectionController
    {
        /// <summary>
        /// Признак выпуклости модели. Влияет на очередность вывода полигонов.
        /// </summary>
        public bool ClosedSurfaceModel = true;

        /// <summary>
        /// Буфер активных полигонов
        /// </summary>
        public PolygonCollectionInfo[] ActivePolygonBuffer
        { get { return ActivePolygon; } }
        protected PolygonCollectionInfo[] ActivePolygon;

        /// <summary>
        /// Количество активных полигонов в коллекции
        /// </summary>
        public int CollectionActivePolygonCount
        { get { return ActivePolygonCount; } }
        protected int ActivePolygonCount;

        public VolumetricModel3D[] Collection
        {
            get { return collection; }
        }
        protected VolumetricModel3D[] collection;

        public ModelCollectionController(int modelCount)
        {
            if (modelCount < 1) throw new Exception("Контроллер моделей должен содержать хотя бы одну модель.");
            collection = new VolumetricModel3D[modelCount];
        }

        /// <summary>
        /// Выделение памяти под общий буфер индексов полигонов
        /// </summary>
        /// <returns></returns>
        public int CreateActivePolygonBuffer()
        {
            try
            {
                ActivePolygonCount = 0;

                #region если в коллекции только одна модель, то не использовать средства контроллера
                if (collection.Length == 1)
                {
                    if (collection[0]!=null)
                         ActivePolygonCount = collection[0].ActivePolygonIndexesCount;
                    return 0;
                }
                #endregion

                #region создание буфера
                int len = 0;
                for (int i = 0; i < collection.Length; i++)
                    if (collection[i]!=null)
                        len += collection[i].ActivePolygonIndexes.Length;
                ActivePolygon = new PolygonCollectionInfo[len];
                #endregion

                return 0;
            }
            catch (Exception er)
            {
                ErrorLog.AddErrorMessage("ModelCollectionController", "CreateActivePolygonBuffer", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Слияние стека индексов полигонов в общий буфер с сортировкой. Исходные стеки предварительно должны быть отсортированы.
        /// </summary>
        /// <returns></returns>
        public int MergeActivePolygon()
        {
            try
            {
                ActivePolygonCount = 0;

                #region Подсчет общего числа активных полигонов и признак выпуклости
                ClosedSurfaceModel = true;
                for (int i = 0; i < collection.Length; i++)
                {
                    if (collection[i] != null)
                    {
                        ActivePolygonCount += collection[i].ActivePolygonIndexesCount;
                        ClosedSurfaceModel &= collection[i].ClosedSurface;
                    }
                }
                if (collection.Length == 1) return 0;
                #endregion


                #region Слияние с сортировкой
                int idx1;
                int[] groupTop = new int[collection.Length];
                for (int i = 0; i < collection.Length; i++)
                {
                    if (collection[i] != null)
                        groupTop[i] = collection[i].ActivePolygonIndexesCount;
                    else
                        groupTop[i] = 0;
                }

                bool AllEmpty = true;
                { // ищу первый миниимальный столбик
                    idx1 = -1;
                    for (int i = 0; i < collection.Length; i++)                        
                    {
                        if (groupTop[i] <= 0) continue;
                        AllEmpty = false;
                        if (idx1 >= 0)
                        {
                            if (collection[i].Polygon[collection[i].ActivePolygonIndexes[groupTop[i] - 1]].Center.Z < collection[idx1].Polygon[collection[idx1].ActivePolygonIndexes[groupTop[idx1] - 1]].Center.Z) idx1 = i;
                        }
                        else idx1 = i;
                    }
                }

                if (AllEmpty) return 0;

                int groupIndex = ActivePolygonCount - 1;
                int idx2;
                bool setIdx = false;

                while (true)
                {
                    AllEmpty = true;
                    // ищу второй минимальный столбик
                    idx2 = -1;
                    for (int i = 0; i < collection.Length; i++)
                    {
                        if (groupTop[i] <= 0) continue;
                        AllEmpty = false;
                        if (i == idx1) continue;
                        setIdx = (idx2 == -1);
                        if (!setIdx) setIdx = (collection[i].Polygon[collection[i].ActivePolygonIndexes[groupTop[i] - 1]].Center.Z < collection[idx2].Polygon[collection[idx2].ActivePolygonIndexes[groupTop[idx2] - 1]].Center.Z);
                        if (setIdx) idx2 = i;
                    }
                    if (AllEmpty) break;
                    if (idx2 == -1) idx2 = idx1;

                    // Выгружаю часть минимального столбика
                    while (groupTop[idx1] > 0)
                    {
                        if (collection[idx1].Polygon[collection[idx1].ActivePolygonIndexes[groupTop[idx1] - 1]].Center.Z <= collection[idx2].Polygon[collection[idx2].ActivePolygonIndexes[groupTop[idx2] - 1]].Center.Z)
                        {
                            ActivePolygon[groupIndex].PolygonIndex = collection[idx1].ActivePolygonIndexes[groupTop[idx1] - 1];
                            ActivePolygon[groupIndex].ModelIndex = idx1;
                            groupIndex--;
                            groupTop[idx1]--;
                        }
                        else break;
                    }
                    idx1 = idx2;
                }
                #endregion

                return 0;
            }
            catch (Exception er)
            {
                ErrorLog.AddErrorMessage("ModelCollectionController", "MergeActivePolygon", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Отрисовка полигонной модели
        /// </summary>
        /// <param name="drawSurface">Поверхность для рисования</param>
        /// <returns>0 - успешно, не 0 - ошибка</returns>
        public int ShowPolygonModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.AllSides)
        {
            try
            {
                if (collection.Length == 1)
                {
                    if (collection[0] != null)
                        if (collection[0].ShowPolygonModel(surface, drawingSide) != 0) throw new Exception(ErrorLog.GetLastError());
                }
                else
                {
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

                        int PolygonPartSize = ActivePolygonCount / surface.BufferCount;
                        Parallel.For(0, surface.BufferCount, (int partIndex) =>
                        {
                            for (int i = partIndex * PolygonPartSize; (i < ActivePolygonCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                                if (collection[ActivePolygon[i].ModelIndex] != null)
                                {
                                    sideIdx = (collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].NormalZ <= 0) ? 0 : 1;


                                    if (drawingSide != PolygonSides.AllSides)
                                    {
                                        if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                        if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                    }

                                    if (Engine3D.BuiltStaticPolygon(ref collection[ActivePolygon[i].ModelIndex].ScreenVertex3D, ref collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());


                                    ((SolidBrush)brush).Color = collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].LightingColor[sideIdx];

                                    surface.FillPolygon(brush, poly,partIndex);
                                }
                        });
                        brush.Dispose();
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ModelCollectionController", "ShowPolygonModel", er.Message, -1);
                return -1;
            }
        }
        
        /// <summary>
        /// Отрисовка полигонов паутиной
        /// </summary>
        /// <param name="drawSurface"></param>
        /// <param name="drawingSide"></param>
        /// <returns></returns>
        public int ShowWideModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.AllSides)
        {
            try
            {
                
                if (collection.Length == 1)
                {
                    if (collection[0]!=null)
                        if (collection[0].ShowPolygonWideModel(surface, drawingSide) != 0) throw new Exception(ErrorLog.GetLastError());
                }
                else
                {
                    if (drawingSide == PolygonSides.Auto)
                    {
                        if (ShowWideModel(surface, PolygonSides.RearSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                        if (ShowWideModel(surface, PolygonSides.FrontSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    }
                    else
                    {
                        Pen pen = new Pen(Color.White);
                        PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };
                        int sideIdx;

                        int PolygonPartSize = ActivePolygonCount / surface.BufferCount;
                        Parallel.For(0, surface.BufferCount, (int partIndex) =>
                          {
                            for(int i=partIndex*PolygonPartSize; (i < ActivePolygonCount)&&(i<(partIndex+1)*PolygonPartSize); i++)
                            if (collection[ActivePolygon[i].ModelIndex] != null)
                              {
                                  sideIdx = (collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].NormalZ <= 0) ? 0 : 1;

                                  if (drawingSide != PolygonSides.AllSides)
                                  {
                                      if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                      if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                  }

                                  if (Engine3D.BuiltStaticPolygon(ref collection[ActivePolygon[i].ModelIndex].ScreenVertex3D, ref collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                                  pen.Color = collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].LightingColor[sideIdx];

                                  surface.DrawPolygon(pen, poly,partIndex);
                              }
                          });
                        pen.Dispose();
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ModelCollectionController", "ShowWideModel", er.Message, -1);
                return -1;
            }
        }
        

        
        /// <summary>
        /// Отрисовка модели
        /// </summary>
        /// <param name="drawSurface"></param>
        /// <param name="drawingSide"></param>
        /// <returns></returns>
        public int ShowModel(IDrawingSurface surface, PolygonSides drawingSide = PolygonSides.AllSides)
        {
            try
            {
                if (collection.Length == 1)
                {
                    if (collection[0] != null)
                        if (collection[0].ShowModel(surface, drawingSide) != 0) throw new Exception(ErrorLog.GetLastError());
                }
                else
                {
                    if (drawingSide == PolygonSides.Auto)
                    {
                        if (ShowModel(surface, PolygonSides.RearSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                        if (ShowModel(surface, PolygonSides.FrontSide) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());
                    }
                    else
                    {
                        int PolygonPartSize = ActivePolygonCount / surface.BufferCount;
                        Parallel.For(0, surface.BufferCount, (int partIndex) =>
                        {
                            int sideIdx;
                            Pen pen = new Pen(Color.White);
                            SolidBrush brush = new SolidBrush(Color.White);

                            PointF[] poly = new PointF[3] { new PointF(), new PointF(), new PointF() };

                            for (int i = partIndex * PolygonPartSize; (i < ActivePolygonCount) && (i < (partIndex + 1) * PolygonPartSize); i++)
                                if (collection[ActivePolygon[i].ModelIndex] != null)
                                {
                                    sideIdx = (collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].NormalZ <= 0) ? 0 : 1;

                                    if (drawingSide != PolygonSides.AllSides)
                                    {
                                        if ((drawingSide == PolygonSides.FrontSide) & (sideIdx == 1)) continue;
                                        if ((drawingSide == PolygonSides.RearSide) & (sideIdx == 0)) continue;
                                    }

                                    if (Engine3D.BuiltStaticPolygon(ref collection[ActivePolygon[i].ModelIndex].ScreenVertex3D, ref collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex], ref poly) != 0) throw new Exception(Graph3DLibrary.ErrorLog.GetLastError());

                                    switch (collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].FillType)
                                    {
                                        case PolygonFillType.Solid:
                                            brush.Color = collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].LightingColor[sideIdx];
                                            surface.FillPolygon(brush, poly,partIndex);
                                            break;
                                        case PolygonFillType.Wide:
                                            pen.Color = collection[ActivePolygon[i].ModelIndex].Polygon[ActivePolygon[i].PolygonIndex].LightingColor[sideIdx];
                                            surface.DrawPolygon(pen, poly,partIndex);
                                            break;
                                    }
                                }
                            pen.Dispose();
                        });
                    }
                }
                return 0;
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ModelCollectionController", "ShowModel", er.Message, -1);
                return -1;
            }
        }
    }
    #endregion

    #region ModelDetalizationInfo
    /// <summary>
    /// Информация о детализации модели
    /// </summary>
    public struct ModelDetalizationInfo
    {
        public VolumetricModel3D Model;
        public float MinSize;
    }
    #endregion
    
    #region ModelDetalizationSelector
    /// <summary>
    /// Управляет детализацией выводимой модели
    /// </summary>
    public class ModelDetalizationSelector
    {
        /// <summary>
        /// Коллекция детализаций моделей
        /// </summary>
        public ModelDetalizationInfo[] ModelDetalizationCollection
        {
            get { return modelDetalizationCollection; }
        }
        protected ModelDetalizationInfo[] modelDetalizationCollection;

        /// <summary>
        /// Расчетный центр модели
        /// </summary>
        public Point3D PositionPoint;

        /// <summary>
        /// Расчетный исходный размер модели
        /// </summary>
        public float SourceSize;

        /// <summary>
        /// Расчетный размер модели с учетом перспективного искажения
        /// </summary>
        public float PresentationSize
        {
            get { return presentationSize; }
        }
        protected float presentationSize;


        /// <summary>
        /// Управление детализацией моделей
        /// </summary>
        /// <param name="DetalizstionCount">Количество детализаций</param>
        public ModelDetalizationSelector(int DetalizstionCount)
        {
            if (DetalizstionCount < 1) throw new Exception("Селектор детализаций должен иметь ссылку хотя бы на одну модель.");
            modelDetalizationCollection = new ModelDetalizationInfo[DetalizstionCount];
            for (int i = 0; i < modelDetalizationCollection.Length; i++)
            {
                modelDetalizationCollection[i].Model = null;
                modelDetalizationCollection[i].MinSize = 0;
            }
        }

        /// <summary>
        /// Выбор детализации модели по текущим координатам и размерам модели.
        /// </summary>
        /// <param name="detailedModelIndex">Получаемый индекс выбираемой модели</param>
        /// <param name="K">Коэффициент перспективы (масштабный коэффициент)</param>
        /// <returns></returns>
        public int SelectDetalizationIndex(ref int detailedModelIndex, float K)
        {
            try
            {
                if (Engine3D.ValuePresentationConversion(PositionPoint.Z, SourceSize, ref presentationSize, K) != 0) throw new Exception(ErrorLog.GetLastError());
                detailedModelIndex = 0;
                for (int i = 1; i < modelDetalizationCollection.Length; i++)
                    if (presentationSize < modelDetalizationCollection[i].MinSize)
                    {
                        detailedModelIndex = i - 1;
                        return 0;
                    }
                detailedModelIndex = modelDetalizationCollection.Length - 1;
                return 0;
            }
            catch (Exception er)
            {
                ErrorLog.AddErrorMessage("ModelDetalizationSelector", "SelectDetalizationIndex", er.Message, -1);
                return -1;
            }
        }


        /// <summary>
        /// Смещение расчетного центра модели 
        /// </summary>
        /// <param name="dX">Смещение по оси OX</param>
        /// <param name="dY">Смещение по оси OY</param>
        /// <param name="dZ">Смещение по оси OZ</param>
        /// <returns></returns>
        public int Move(float dX, float dY, float dZ)
        {
            try
            {
                PositionPoint.X += dX;
                PositionPoint.Y += dY;
                PositionPoint.Z += dZ;
                return 0;
            }
            catch (Exception er)
            {
                ErrorLog.AddErrorMessage("ModelDetalizationSelector", "Move", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Перемещение расчетного центра модели в указанную точку
        /// </summary>
        /// <param name="X">X-координата</param>
        /// <param name="Y">Y-координата</param>
        /// <param name="Z">Z-координата</param>
        public int MoveTo(float X, float Y, float Z)
        {
            try
            {

                X -= PositionPoint.X;
                Y -= PositionPoint.Y;
                Z -= PositionPoint.Z;
                return Engine3D.MovePoint3D(X, Y, Z, PositionPoint);
            }
            catch (Exception er)
            {
                ErrorLog.AddErrorMessage("ModelDetalizationSelector", "MoveTo", er.Message, -1);
                return -1;
            }
        }

        /// <summary>
        /// Вращение расчетного центра модели вокруг осей
        /// </summary>
        /// <param name="angle">Угол вращения</param>
        /// <param name="axis">Ось вращения</param>
        /// <returns></returns>
        public int Rotate(float angle, Axis3D axis)
        {
            try
            {
                return Engine3D.RotatePoint3D(angle, axis, PositionPoint);
            }
            catch (Exception er)
            {
                Graph3DLibrary.ErrorLog.AddErrorMessage("ModelDetalizationSelector", "Rotate", er.Message, -1);
                return -1;
            }
        }

    }
    #endregion
}