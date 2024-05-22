
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Kitware.VTK;
using Kitware.mummy.Runtime;
using System.Numerics;
using System.Collections;
using static System.Net.Mime.MediaTypeNames;

namespace rrt_star_Connect_CSharp
{

    public struct point
    {
        public double x { get; }
        public double y { get; }
        public double z { get; }
        public point(double X, double Y, double Z)
        {
            x = X;
            y = Y;
            z = Z;
        }

    };
    public class Node3D_star
    {
        public point loc;
        public double cost;
        public Node3D_star parent;

    }

   


    public  class TxRobotRRTStartConnect 
    {
        private int connected = 0;
        private int state = 1;
        private int sub_state = 0;
        private double step_size = 5.0;
        private double circle_radius_1 = 20.0;
        private double circle_radius_2 = 5.0;

        private List<point> path_points_start = new List<point>(500);
        private int pathcount_start = 0;
        private List<point> path_points_end = new List<point>(500);
        private int pathcount_end = 0;

        private List<Node3D_star> start_nodes = new List<Node3D_star>(10000);
        private int nodecount_start = 0;
        private List<Node3D_star> end_nodes = new List<Node3D_star>(10000);
        private int nodecount_end = 0;

        private List<double[]> obstractleList= new List<double[]>(); // List to keep the obstractle data information only for cube and sphere
        private List<vtkActor> findalConnectLines= new List<vtkActor>(); // List to keep the data for path line information once the path is founded and show them in VKT Window
        public double dist(point p1, point p2)  // To calculate the distance between two points
        {

            return Math.Sqrt(Math.Pow(p2.x - p1.x, 2) + Math.Pow(p2.y - p1.y, 2) + Math.Pow(p2.z - p1.z, 2));
        }

        public int Nearest_Node(Node3D_star rand)
        {
            double min = 999.0;
            int index = -1;

            if (state == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {

                    if (dist(rand.loc, start_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, start_nodes[i].loc);
                        index = i;
                    }

                }

            }
            else
            {
                for (int i = 0; i < nodecount_end; i++)
                {

                    if (dist(rand.loc, end_nodes[i].loc) < min)
                    {
                        min = dist(rand.loc, end_nodes[i].loc);
                        index = i;
                    }

                }



            }




            return index;
        }

        public point step_func(point near, point rand, double size_step)
        {
            double dx = rand.x - near.x;
            double dy = rand.y - near.y;
            double dz = rand.z - near.z;

            double d = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2) + Math.Pow(dz, 2));

            point step = new point(near.x + (size_step) * (dx / d)
                , near.y + (size_step) * (dy / d),
                near.z + (size_step) * (dz / d));

            return step;
        }
        public bool isValid(point near, point step, List<double[]> obstractleList)
        {
            //for collision check

            int LineDiv = 30;
            double[] MiddleCheckNode = new double[3];

            for(int i=0;i< obstractleList.Count;i++)
            {
                if (obstractleList[i].Length==6) // cube obstractle
                {
                    for(int j=1;j<=LineDiv; j++)
                    {
                        MiddleCheckNode[0] = near.x + (step.x-near.x) * j / LineDiv;
                        MiddleCheckNode[1] = near.y + (step.y - near.y) * j / LineDiv;
                        MiddleCheckNode[2] = near.z+ (step.z - near.z) * j / LineDiv;


                        if ((Math.Abs(MiddleCheckNode[0] - obstractleList[i][0]) <= obstractleList[i][3] / 2) && (Math.Abs(MiddleCheckNode[1] - obstractleList[i][1]) <= obstractleList[i][4] / 2) && (Math.Abs(MiddleCheckNode[2] - obstractleList[i][2]) <= obstractleList[i][5] / 2))
                        {
                            return false;
                        }
                       

                    }



                }
                else// sphere obstractle
                {

                    for (int j = 1; j <= LineDiv; j++)
                    {
                        point p=new point(near.x + (step.x - near.x) * j / LineDiv, near.y + (step.y - near.y) * j / LineDiv, near.z + (step.z - near.z) * j / LineDiv);
                     
                        point center=new point(obstractleList[i][0], obstractleList[i][1], obstractleList[i][2]);
                        

                        if (dist(p, center)<= obstractleList[i][3])
                        {
                            return false;
                        }
             
                    }


                }

            }



            return true;
        }

        public void minimal_cost(Node3D_star step)
        {

            double new_cost;
            double min_cost = step.cost;
            int index = -1;

            if (state == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_1 && isValid(start_nodes[i].loc, step.loc, obstractleList))
                    {
                        new_cost = dist(start_nodes[i].loc, step.loc) + start_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }

                }
                if (min_cost < step.cost)
                {
                    step.parent = start_nodes[index];
                    step.cost = min_cost;
                }

            }
            else
            {
                for (int i = 0; i < nodecount_end; i++)
                {
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_1 && isValid(end_nodes[i].loc, step.loc, obstractleList))
                    {
                        new_cost = dist(end_nodes[i].loc, step.loc) + end_nodes[i].cost;
                        if (new_cost < min_cost)
                        {
                            min_cost = new_cost;
                            index = i;
                        }
                    }

                }
                if (min_cost < step.cost)
                {
                    step.parent = end_nodes[index];
                    step.cost = min_cost;
                }


            }

        }

        public void path(int index_1, int index_2)// to generate the final red path
        {
            Node3D_star n1, n2;
            n2 = start_nodes[index_1 - 1];
            n1 = (n2.parent);
            while (n1.parent != null)
            {
                
                vtkActor line = DrawLine(new point(n1.loc.x, n1.loc.y,n1.loc.z), new point(n2.loc.x, n2.loc.y,n2.loc.z),1);
                findalConnectLines.Add(line);
                n2 = n1;
                n1 = n2.parent;

            }
            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;
            while (n1.parent != null)
            {
               
                vtkActor line = DrawLine(new point(n1.loc.x, n1.loc.y, n1.loc.z), new point(n2.loc.x, n2.loc.y, n2.loc.z),1);
                findalConnectLines.Add(line);
                n2 = n1;
                n1 = n2.parent;

            }


        }
        public void path_Points(int index_1, int index_2) // final path points will be keep at path_points_start and path_points_end List
        {
            pathcount_start = 0;
            pathcount_end = 0;
            Node3D_star n1, n2;
            float d = 20.0f;

            n2 = start_nodes[index_1 - 1];
            n1 = n2.parent;
            
            path_points_start.Add(n2.loc);
            pathcount_start++;

            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_start[pathcount_start - 1]) < d)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;
                   
                    path_points_start.Add(n2.loc);
                    pathcount_start++;
                }
            }
            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;
           
            path_points_end.Add(n2.loc);
            pathcount_end++;
            while (n1.parent != null)
            {
                if (dist(n1.loc, path_points_end[pathcount_end - 1]) < d)
                {
                    n1 = n1.parent;
                }
                else
                {
                    n2 = n1;
                    n1 = n2.parent;
                    path_points_end.Add(n2.loc);
                    pathcount_end++;
                }
            }



        }

        public void rrt_connect(point p_start, point p_end)
        {
            connected = 0;
            state = 1;
            sub_state = 0;
            nodecount_start = 0;
            nodecount_end = 0;

            Node3D_star start_node = new Node3D_star();
            Node3D_star end_node = new Node3D_star();
            Node3D_star rand_node = new Node3D_star();
            int index;

           
            Node3D_star step_node;
            Node3D_star sub_step_node;

            Random rd = new Random(unchecked((int)DateTime.Now.Ticks));

            start_node.loc = new point(p_start.x, p_start.y, p_start.z);
            
            start_node.parent = new Node3D_star();
            start_node.cost = 0;

            end_node.loc=new point(p_end.x, p_end.y, p_end.z);
            
            end_node.parent = new Node3D_star();
            end_node.cost = 0;

            start_nodes.Add(start_node);

            nodecount_start++;
            end_nodes.Add(end_node);
            nodecount_end++;

            while (connected != 1)
            {
                sub_state = 0;

                if (state == 1)
                {

                    
                    rand_node.loc = new point(rd.Next() % (Math.Abs((p_end.x - p_start.x) * 2))+ p_start.x, rd.Next() % (Math.Abs((p_end.y - p_start.y) * 2))+ p_start.y, rd.Next() % (Math.Abs((p_end.z - p_start.z )* 2))+ p_start.z);
                    index = Nearest_Node(rand_node);

                    if (dist(start_nodes[index].loc, rand_node.loc) < step_size)
                    {
                        continue;
                    }
                    else
                    {
                        step_node = new Node3D_star();
                        (step_node.loc) = step_func(start_nodes[index].loc, rand_node.loc, step_size);
                    }
                    if (isValid(start_nodes[index].loc, step_node.loc, obstractleList) == false)
                    {
                        continue;
                    }
                    else
                    {
                        step_node.parent = start_nodes[index];
                        step_node.cost = start_nodes[index].cost + step_size;
                        minimal_cost(step_node);
                        start_nodes.Add(step_node);
                        nodecount_start++;
                    }

                    vtkActor line = DrawLine(new point(step_node.loc.x, step_node.loc.y, step_node.loc.z), new point(step_node.parent.loc.x, step_node.parent.loc.y, step_node.parent.loc.z), 0);
                    findalConnectLines.Add(line);
                    state = 2;

                    while (sub_state != 1)
                    {
                        index = Nearest_Node(step_node);

                        if (dist(end_nodes[index].loc, step_node.loc) < 20.0 && isValid( step_node.loc, end_nodes[index].loc, obstractleList))
                        {
                            line = DrawLine(new point(step_node.loc.x, step_node.loc.y, step_node.loc.z), new point(end_nodes[index].loc.x, end_nodes[index].loc.y, end_nodes[index].loc.z),1);
                            findalConnectLines.Add(line);

                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");                         
                            path(nodecount_start, index + 1);
                            path_Points(nodecount_start, index + 1);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_star();
                            (sub_step_node.loc) = step_func(end_nodes[index].loc, step_node.loc, step_size);
                        }
                        if (isValid(end_nodes[index].loc, sub_step_node.loc, obstractleList) == false)
                        {
                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            end_nodes.Add(sub_step_node);
                            sub_step_node.parent = end_nodes[index];
                            sub_step_node.cost = end_nodes[index].cost + step_size;
                            minimal_cost( sub_step_node);
                            
                            nodecount_end++;
                        }

                        line = DrawLine(new point((sub_step_node.loc).x, (sub_step_node.loc).y, (sub_step_node.loc).z), new point((sub_step_node.parent.loc).x, (sub_step_node.parent.loc).y, (sub_step_node.parent.loc).z), 0);
                        findalConnectLines.Add(line);
                       
                    }
                }

                if (state == 2)
                {
                    
                    rand_node.loc = new point(rd.Next() % (Math.Abs((p_end.x - p_start.x * 2))), rd.Next() % (Math.Abs((p_end.y - p_start.y * 2))), rd.Next() % (Math.Abs((p_end.z - p_start.z * 2))));
                    index = Nearest_Node(rand_node);

                    if (dist(end_nodes[index].loc, rand_node.loc) < step_size)
                    {
                        continue;
                    }
                    else
                    {
                        step_node = new Node3D_star();
                        (step_node.loc) = step_func(end_nodes[index].loc, rand_node.loc, step_size);
                    }

                    if (isValid(end_nodes[index].loc, step_node.loc, obstractleList) == false)
                    {
                        continue;
                    }
                    else
                    {
                        step_node.parent = end_nodes[index];
                        step_node.cost = end_nodes[index].cost + step_size;
                        minimal_cost( step_node);                       
                        end_nodes.Add(step_node);
                        nodecount_end++;
                    }


                    vtkActor line = DrawLine(new point(step_node.loc.x, step_node.loc.y, step_node.loc.z), new point(step_node.parent.loc.x, step_node.parent.loc.y, step_node.parent.loc.z), 0);
                    findalConnectLines.Add(line);
                    
                    state = 1;

                    while (sub_state != 1)
                    {

                        index = Nearest_Node(step_node);

                        if (dist(start_nodes[index].loc, step_node.loc) < 10.0 && isValid(step_node.loc, start_nodes[index].loc , obstractleList))
                        {
                            line = DrawLine(new point((step_node.loc).x, (step_node.loc).y, (step_node.loc).z), new point(start_nodes[index].loc.x, start_nodes[index].loc.y, start_nodes[index].loc.z),1);
                            findalConnectLines.Add(line);
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                            path( index + 1, nodecount_end);
                            path_Points(index + 1, nodecount_end);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node3D_star();
                            (sub_step_node.loc) = step_func(start_nodes[index].loc, step_node.loc, step_size);
                        }

                        if (isValid(start_nodes[index].loc, sub_step_node.loc, obstractleList) == false)
                        {
                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            sub_step_node.parent = start_nodes[index];
                            sub_step_node.cost = start_nodes[index].cost + step_size;
                            minimal_cost( sub_step_node);
                            start_nodes.Add(sub_step_node);
                            nodecount_start++;
                        }

                        line = DrawLine(new point((sub_step_node.loc).x, (sub_step_node.loc).y, (sub_step_node.loc).z), new point((sub_step_node.parent.loc).x, (sub_step_node.parent.loc).y, (sub_step_node.parent.loc).z), 0);
                        findalConnectLines.Add(line);
                       
                    }

                }




            }
            
        }

        public vtkAxesActor vtkCartesianCoordinateSystem()
        {
            //创建坐标系成员
            vtkTransform transform = vtkTransform.New();
            transform.Translate(0.0, 0.0, 0.0); //通过translate函数进行坐标系的平移

            vtkAxesActor axesActor = vtkAxesActor.New();
            axesActor.SetUserTransform(transform);
            axesActor.SetTotalLength(100,100,100);
            axesActor.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetColor(255, 0, 0);
            axesActor.SetXAxisLabelText("X");
           
            axesActor.GetYAxisCaptionActor2D().GetCaptionTextProperty().SetColor(255, 0, 0);
            axesActor.SetYAxisLabelText("Y");
           
            axesActor.GetZAxisCaptionActor2D().GetCaptionTextProperty().SetColor(255, 0, 0);
            axesActor.SetZAxisLabelText("Z");
           
            axesActor.GetXAxisShaftProperty().SetLineWidth(3.0f);
            axesActor.GetYAxisShaftProperty().SetLineWidth(3.0f);
            axesActor.GetZAxisShaftProperty().SetLineWidth(3.0f);

            

            return axesActor;
        }

        public vtkActor drawSphere(double[] Center, double radius)
        {
            /*
          * 映射器Mapper 是将数据集(vtkPolyData, vtkImageData等)映射到图形系统中进行可视化的组件
          * 负责将数据转换为图形进行表示，并将数据传递给渲染器(renderer)进行显示
          * 映射器起到了数据和图形之间的桥梁作用；
          * vtkPolyDataMapper 是最常用的映射器之一，
          * 用于将vtkPolyData的数据(包括点，线，面等几何数据)
          * 映射到图形系统当中，将数据转换为图形表示，如点，线面等，
          * 然后传递给渲染器进行显示
          */

            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            vtkActor actor = vtkActor.New();
            vtkSphereSource sphereSource = new vtkSphereSource();
            sphereSource.SetCenter(Center[0], Center[1], Center[2]);
            sphereSource.SetRadius(radius);
            mapper.SetInput(sphereSource.GetOutput());
            actor.SetMapper(mapper);

            return actor;
        }
        public vtkActor drawCube( double[] Center, double[] radius)
        {
            /*
          * 映射器Mapper 是将数据集(vtkPolyData, vtkImageData等)映射到图形系统中进行可视化的组件
          * 负责将数据转换为图形进行表示，并将数据传递给渲染器(renderer)进行显示
          * 映射器起到了数据和图形之间的桥梁作用；
          * vtkPolyDataMapper 是最常用的映射器之一，
          * 用于将vtkPolyData的数据(包括点，线，面等几何数据)
          * 映射到图形系统当中，将数据转换为图形表示，如点，线面等，
          * 然后传递给渲染器进行显示
          * 
          */
            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            vtkActor actor = vtkActor.New();
            vtkCubeSource cubeSource = new vtkCubeSource();
            cubeSource.SetCenter(Center[0], Center[1], Center[2]);
            cubeSource.SetXLength( radius[0]);
            cubeSource.SetYLength(radius[1]);
            cubeSource.SetZLength(radius[2]);

            mapper.SetInput(cubeSource.GetOutput());
            actor.SetMapper(mapper);

            
            return actor;

        }

        public List<vtkActor> randomGenerateObstacle(point start, point end,int number)
        {
            double[] ObstaclePosition = new double[3];
            double[] ObstacleSize = new double[3];
            List<vtkActor> vtkActors = new List<vtkActor>();


            Random obstactleRand = new Random(unchecked((int)DateTime.Now.Ticks));

            /* Random generate the cube and sphere obstractle
             * the cube and sphere center scope is withine 30 to 70
             * The cube size will be withine 10-50;
             * the sphere radius will be withine 10-50
             */
            for (int i=0;i<number;i++)
            {
                double obstacleType = obstactleRand.NextDouble();
                if (obstacleType < 0.5)
                {
                    ObstaclePosition[0] = obstactleRand.Next(10, 90);
                    ObstaclePosition[1] = obstactleRand.Next(10, 90);
                    ObstaclePosition[2] = obstactleRand.Next(10, 90);

                    ObstacleSize[0]= obstactleRand.Next(10, 50);
                    ObstacleSize[1] = obstactleRand.Next(10, 50);
                    ObstacleSize[2] = obstactleRand.Next(10, 50);

                    double[] obstacleInformation = new double[6] { ObstaclePosition[0], ObstaclePosition[1], ObstaclePosition[2], ObstacleSize[0], ObstacleSize[1], ObstacleSize[2] };

                    // To confirm the start and end point is not collision with sphere and cube

                    if ((Math.Abs(start.x- obstacleInformation[0]) <= obstacleInformation[3] / 2) && (Math.Abs(start.y - obstacleInformation[1]) <= obstacleInformation[4] / 2) && (Math.Abs(start.z - obstacleInformation[2]) <= obstacleInformation[5] / 2))
                    {
                        continue;
                    }


                    if ((Math.Abs(end.x - obstacleInformation[0]) <= obstacleInformation[3] / 2) && (Math.Abs(end.y - obstacleInformation[1]) <= obstacleInformation[4] / 2) && (Math.Abs(end.z - obstacleInformation[2]) <= obstacleInformation[5] / 2))
                    {
                        continue;
                    }

                    
                    obstractleList.Add(obstacleInformation);
                    vtkActor actorCube= drawCube(ObstaclePosition, ObstacleSize);
                    vtkActors.Add(actorCube);


                }
                else
                {
                    ObstaclePosition[0] = obstactleRand.Next(10, 90);
                    ObstaclePosition[1] = obstactleRand.Next(10, 90);
                    ObstaclePosition[2] = obstactleRand.Next(10, 90);
                    ObstacleSize[0] = obstactleRand.Next(10, 50);
                    double[] obstacleInformation = new double[4] { ObstaclePosition[0], ObstaclePosition[1], ObstaclePosition[2], ObstacleSize[0] };
                    point center = new point(ObstaclePosition[0], ObstaclePosition[1], ObstaclePosition[2]);
                    if (dist(start, center) <= obstacleInformation[3])
                    {
                        continue;
                    }
                    if (dist(end, center) <= obstacleInformation[3])
                    {
                        continue;
                    }

                    obstractleList.Add(obstacleInformation);
                    vtkActor actorSphere = drawSphere(ObstaclePosition, ObstacleSize[0]);
                    vtkActors.Add(actorSphere);

                }


            }


            return vtkActors;
        }

        public vtkActor DrawLine(point start, point end, int type)
        {
            vtkLineSource vtkLineSource= new vtkLineSource();
            vtkActor vtkLineActor= new vtkActor();
            vtkPolyDataMapper mapper = vtkPolyDataMapper.New();
            vtkLineSource.SetPoint1(start.x, start.y, start.z);
            vtkLineSource.SetPoint2(end.x, end.y, end.z);
            vtkLineSource.Update();
            mapper.SetInputConnection(vtkLineSource.GetOutputPort());
            vtkLineActor.SetMapper(mapper);
            if(type==1)
            {
                vtkLineActor.GetProperty().SetColor(1, 0, 0);
                vtkLineActor.GetProperty().SetLineWidth(3.0f);
            }
               
            else
            {
                vtkLineActor.GetProperty().SetColor(1, 1, 0);
                vtkLineActor.GetProperty().SetLineWidth(1.0f);
            }
               

            return vtkLineActor;

        }

        public void vtkRRTConnectShow()
        {
            point start = new point(0, 0, 0);
            point end = new point(100, 100, 100);
            vtkActor start_actor= drawSphere(new double[] { start.x, start.y, start.z }, 1);
            vtkActor end_actor = drawSphere(new double[] { end.x, end.y, end.z }, 1);
            start_actor.GetProperty().SetColor(0,0,1);
            end_actor.GetProperty().SetColor(0, 0, 1);

            List<vtkActor> vtkActors = randomGenerateObstacle(start, end ,15);
            vtkAxesActor axesActor = vtkCartesianCoordinateSystem();
            vtkRenderWindow renderWindow = vtkRenderWindow.New();
            renderWindow.SetWindowName("VKT TEST");
            renderWindow.SetSize(1000, 1000);
            vtkRenderer renderer = vtkRenderer.New();            
            for(int i=0;i< vtkActors.Count;i++)
            {
                renderer.AddActor(vtkActors[i]);
            }
            renderer.AddActor(start_actor);
            renderer.AddActor(end_actor);

            rrt_connect(start,end);
            for(int i=0;i< findalConnectLines.Count;i++)
            {
               renderer.AddActor(findalConnectLines[i]);

            }
            vtkActor start_pathactor = new vtkActor();
            for (int i=0;i<path_points_start.Count;i++)
            {
                start_pathactor = drawSphere(new double[] { path_points_start[i].x, path_points_start[i].y, path_points_start[i].z }, 1);
                start_pathactor.GetProperty().SetColor(0, 0, 1);
                renderer.AddActor(start_pathactor);

            }
            vtkActor end_pathactor = new vtkActor();
            for (int i = 0; i < path_points_end.Count; i++)
            {
                end_pathactor = drawSphere(new double[] { path_points_end[i].x, path_points_end[i].y, path_points_end[i].z }, 1);
                end_pathactor.GetProperty().SetColor(0, 0, 1);
                renderer.AddActor(end_pathactor);

            }
            renderer.AddActor(axesActor);
            renderer.SetBackground(80, 80, 80);         
            renderer.GetActiveCamera().Azimuth(50);
            renderer.GetActiveCamera().Elevation(50);
            renderer.ResetCamera();  
            renderWindow.AddRenderer(renderer);          
            renderWindow.Render();
            vtkRenderWindowInteractor interactor = new vtkRenderWindowInteractor();
            interactor.SetRenderWindow(renderWindow);
            interactor.Initialize();
            interactor.MiddleButtonPressEvt += Interactor_MiddleButtonPressEvt; //点击鼠标中键，视图恢复至初始水平

            interactor.Start();
        }

        private void Interactor_MiddleButtonPressEvt(vtkObject sender, vtkObjectEventArgs e)
        {
            vtkRenderWindowInteractor iren = sender as vtkRenderWindowInteractor;
            if (iren != null)
            {
                vtkRendererCollection vtkRendererCollection= iren.GetRenderWindow().GetRenderers();
               
                vtkRendererCollection.GetFirstRenderer().ResetCamera();
            }
        }

        private static void rightButtonpress(vtkObject sender, vtkObjectEventArgs e)
        {
            vtkRenderWindowInteractor iren = sender as vtkRenderWindowInteractor;
            if (iren != null)
            {
                System.Console.WriteLine("Mouse moved at: " + iren.GetEventPosition()[0] + ", " + iren.GetEventPosition()[1]);
            }

        }

        private static void leftButtonpress(vtkObject sender, vtkObjectEventArgs e)
        {

            vtkRenderWindowInteractor iren = sender as vtkRenderWindowInteractor;
            if (iren != null)
            {
                System.Console.WriteLine("Mouse moved at: " + iren.GetEventPosition()[0] + ", " + iren.GetEventPosition()[1]);
            }

        }

     
    }
}

