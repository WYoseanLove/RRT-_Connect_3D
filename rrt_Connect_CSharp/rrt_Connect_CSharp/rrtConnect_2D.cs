using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenCvSharp;
using static OpenCvSharp.ML.DTrees;
using static System.Net.Mime.MediaTypeNames;
using System.Xml.Linq;

namespace rrt_star_Connect_CSharp
{
    public struct point2D
    {
        public int x, y;
    };
    public class Node2D_star
    {
        public point2D loc;
        public double cost;
        public Node2D_star parent;

    }

    public class rrtConnect_2D
    {
        private int connected = 0;
        private int state = 1;
        private int sub_state = 0;
        private double step_size = 5.0;
        private double circle_radius_1 = 20.0;
        private double circle_radius_2 = 5.0;

        private List<point2D> path_points_start = new List<point2D>(500);
        private int pathcount_start = 0;
        private List<point2D> path_points_end = new List<point2D>(500);
        private int pathcount_end = 0;

        private List<Node2D_star> start_nodes = new List<Node2D_star>(10000);
        private int nodecount_start = 0;
        private List<Node2D_star> end_nodes = new List<Node2D_star>(10000);
        private int nodecount_end = 0;


        public double dist(point2D p1, point2D p2)
        {

            return Math.Sqrt(Math.Pow(p2.x - p1.x, 2) + Math.Pow(p2.y - p1.y, 2));
        }

        public int Nearest_Node(Node2D_star rand)
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

        public point2D step_func(point2D near, point2D rand, double size_step)
        {
            double dx = rand.x - near.x;
            double dy = rand.y - near.y;
          

            double d = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2));

            point2D step;

            step.x =(int )(near.x + (size_step) * ((float)dx / d));
            step.y = (int)(near.y + (size_step) * ((float)dy / d));
           

            return step;
        }

        public bool isValid(Mat obst, point2D near, point2D step)
        {
            //for collision check
            if(obst.At<byte>(step.y,step.x)==255)
            {
                return false;

            }
            int x1 = near.x, x2 = step.x;
            if (near.x > step.x)
            {
                x1 = step.x;
                x2 = near.x;
            }
            if (x1 != x2)
            {
                float m = (float)(step.y - near.y) / (step.x - near.x);
                float c = (float)(near.y) - (near.x * m);
                int j;
                for (int i = x1; i <= x2; i++)
                {
                    j = (int)((m * i) + c);
                    if (obst.At<byte>(j, i) == 255)
                    {
                        return false;
                    }
                }
            }

            int y1 = near.y, y2 = step.y;
            if (near.y > step.y)
            {
                y1 = step.y;
                y2 = near.y;
            }
            if (y1 != y2)
            {
                float m = (float)(step.x - near.x) / (step.y - near.y);
                float c = (float)(near.x) - (near.y * m);
                int j;
                for (int i = y1; i <= y2; i++)
                {
                    j = (int)((m * i) + c);
                    if (obst.At<byte>(i, j) == 255)
                    {
                        return false;
                    }
                }
            }


            return true;
        }

        public void minimal_cost(Mat obst, Node2D_star step)
        {

            double new_cost;
            double min_cost = step.cost;
            int index = -1;

            if (state == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_1 && isValid(obst, start_nodes[i].loc, step.loc))
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
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_1 && isValid(obst, end_nodes[i].loc, step.loc))
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

        public void rewiring(Mat image, Mat obst, Node2D_star step)
        {
            float new_cost;
            if (state == 1)
            {
                for (int i = 0; i < nodecount_start; i++)
                {
                    if (dist(start_nodes[i].loc, step.loc) < circle_radius_2)
                    {
                        new_cost = (float)(dist(start_nodes[i].loc, step.loc) + (step.cost));
                        if (new_cost < start_nodes[i].cost && isValid(obst, start_nodes[i].loc, step.loc))
                        {
                            start_nodes[i].cost = new_cost;
                            start_nodes[i].parent = step;
                            image.Line(start_nodes[i].loc.x, start_nodes[i].loc.y, (start_nodes[i].parent.loc).x, (start_nodes[i].parent.loc).y, Scalar.Yellow, 2, LineTypes.Link8);
                        }


                    }
                }
             }
            else
            {
                for (int i = 0; i < nodecount_end; i++)
                {
                    if (dist(end_nodes[i].loc, step.loc) < circle_radius_2)
                    {
                        new_cost = (float)(dist(end_nodes[i].loc, step.loc) + (step.cost));
                        if (new_cost < end_nodes[i].cost && isValid(obst, end_nodes[i].loc, step.loc))
                        {
                            end_nodes[i].cost = new_cost;
                            end_nodes[i].parent = step;
                            image.Line(end_nodes[i].loc.x, end_nodes[i].loc.y, (end_nodes[i].parent.loc).x, (end_nodes[i].parent.loc).y, Scalar.Yellow, 2, LineTypes.Link8);
                        }
                    }
                }


            }


        }

        public void path(Mat image, int index_1, int index_2)
        {
            Node2D_star n1, n2;
            n2 = start_nodes[index_1 - 1];
            n1 = (n2.parent);
            while(n1.parent!=null)
            {
                image.Line(n1.loc.x, n1.loc.y,n2.loc.x,n2.loc.y, Scalar.Blue, 2, LineTypes.Link8);
                n2 = n1;
                n1 = n2.parent;

            }
            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;
            while (n1.parent != null)
            {
                image.Line(n1.loc.x, n1.loc.y, n2.loc.x, n2.loc.y, Scalar.Blue, 2, LineTypes.Link8);
                n2 = n1;
                n1 = n2.parent;

            }

        }

        public void path_points(int index_1, int index_2)
        {
            pathcount_start = 0;
            pathcount_end = 0;
            Node2D_star n1, n2;
            float d = 20.0f;

            n2 = start_nodes[index_1 - 1];
            n1 = n2.parent;
            // path_points_start[pathcount_start++] = n2.loc;
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
                    //path_points_start[pathcount_start++] = n2.loc;
                    path_points_start.Add(n2.loc);
                    pathcount_start++;
                }
            }

            n2 = end_nodes[index_2 - 1];
            n1 = n2.parent;
            // path_points_end[pathcount_end++] = n2.loc;
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

        public void rrt_connect(point2D p_start, point2D p_end, Mat img, Mat obst)
        {
            connected = 0;
            state = 1;
            sub_state = 0;
            nodecount_start = 0;
            nodecount_end = 0;

            Node2D_star start_node =new Node2D_star();
            Node2D_star end_node = new Node2D_star();
            Node2D_star rand_node = new Node2D_star();
            int index;

            // Node2D_star temp = new Node2D_star();
            //Node2D_star step_node = temp;
            Node2D_star step_node;

            // Node2D_star sub_temp = new Node2D_star();
            //Node2D_star sub_step_node = sub_temp;
            Node2D_star sub_step_node;

            Random rd = new Random(unchecked((int)DateTime.Now.Ticks));

            start_node.loc.x = p_start.x;
            start_node.loc.y = p_start.y;
            start_node.parent = new Node2D_star();
            start_node.cost = 0;
            end_node.loc.x = p_end.x;
            end_node.loc.y = p_end.y;
            end_node.parent = new Node2D_star();
            end_node.cost = 0;

            //start_nodes[nodecount_start] = start_node;
            start_nodes.Add(start_node);

            nodecount_start++;
            //end_nodes[nodecount_end] = end_node;
            end_nodes.Add(end_node);
            nodecount_end++;

            while (connected != 1)
            {
                sub_state = 0;

                if (state == 1)
                {
                    
                    rand_node.loc.x = rd.Next() % obst.Cols;
                    rand_node.loc.y = rd.Next() % obst.Rows;
                    index = Nearest_Node(rand_node);

                    if (dist(start_nodes[index].loc, rand_node.loc) < step_size)
                    {
                        continue;
                    }
                    else
                    {
                        step_node = new Node2D_star();
                        (step_node.loc) = step_func(start_nodes[index].loc, rand_node.loc, step_size);
                    }
                    if (isValid(obst, start_nodes[index].loc, step_node.loc) == false)
                    {
                        continue;
                    }
                    else
                    {
                        step_node.parent = start_nodes[index];
                        step_node.cost = start_nodes[index].cost + step_size;
                        minimal_cost(obst, step_node);
                        //rewiring(img,obst,step_node);
                        //start_nodes[nodecount_start] = step_node;
                        start_nodes.Add(step_node);
                        nodecount_start++;
                    }

                    img.Line(step_node.loc.x, step_node.loc.y, step_node.parent.loc.x, step_node.parent.loc.y, Scalar.Yellow, 1, LineTypes.Link8);
                    Cv2.ImShow("IMAGE", img);
                    Cv2.WaitKey(50);

                    state = 2;

                    while (sub_state != 1)
                    {
                        index = Nearest_Node(step_node);

                        if (dist(end_nodes[index].loc, step_node.loc) < 20.0 && isValid(obst, step_node.loc, end_nodes[index].loc))
                        {
                            img.Line((step_node.loc).x, (step_node.loc).y, end_nodes[index].loc.x, end_nodes[index].loc.y, Scalar.Blue, 2, LineTypes.Link8);
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                            //console<< "DONE" << '\n';
                            path(img, nodecount_start, index + 1);
                            path_points(nodecount_start, index + 1);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node2D_star();
                            (sub_step_node.loc) = step_func(end_nodes[index].loc, step_node.loc, step_size);
                        }
                        if (isValid(obst, end_nodes[index].loc, sub_step_node.loc) == false)
                        {
                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            end_nodes.Add(sub_step_node);
                            sub_step_node.parent = end_nodes[index];
                            sub_step_node.cost = end_nodes[index].cost + step_size;
                            minimal_cost(obst, sub_step_node);
                            //rewiring(img,obst,sub_step_node);
                            //end_nodes[nodecount_end] = sub_step_node;
                            nodecount_end++;
                        }
                        img.Line((sub_step_node.loc).x, (sub_step_node.loc).y, (sub_step_node.parent.loc).x, (sub_step_node.parent.loc).y, Scalar.Yellow, 1, LineTypes.Link8);
                        Cv2.ImShow("IMAGE", img);
                        Cv2.WaitKey(50);

                    }
                }

                if (state == 2)
                {
                    rand_node.loc.x = rd.Next() % obst.Cols;
                    rand_node.loc.y = rd.Next() % obst.Rows;
                    index = Nearest_Node(rand_node);

                    if (dist(end_nodes[index].loc, rand_node.loc) < step_size)
                    {
                        continue;
                    }
                    else
                    {
                        step_node = new Node2D_star();
                        (step_node.loc) = step_func(end_nodes[index].loc, rand_node.loc, step_size);
                    }

                    if (isValid(obst, end_nodes[index].loc, step_node.loc) == false)
                    {
                        continue;
                    }
                    else
                    {
                        step_node.parent = end_nodes[index];
                        step_node.cost = end_nodes[index].cost + step_size;
                        minimal_cost(obst, step_node);
                        //rewiring(img,obst,step_node);
                        //end_nodes[nodecount_end] = step_node;
                        end_nodes.Add(step_node);
                        nodecount_end++;
                    }
                    img.Line((step_node.loc).x, (step_node.loc).y, (step_node.parent.loc).x, (step_node.parent.loc).y, Scalar.Yellow, 1, LineTypes.Link8);
                    Cv2.ImShow("IMAGE", img);
                    Cv2.WaitKey(50);

                    state = 1;

                    while (sub_state != 1)
                    {

                        index = Nearest_Node(step_node);

                        if (dist(start_nodes[index].loc, step_node.loc) < 10.0 && isValid(obst, step_node.loc, start_nodes[index].loc))
                        {
                            img.Line((step_node.loc).x, (step_node.loc).y, start_nodes[index].loc.x, start_nodes[index].loc.y, Scalar.Blue, 2, LineTypes.Link8);
                            connected = 1;
                            sub_state = 1;
                            Console.WriteLine("DONE");
                           path(img, index + 1, nodecount_end);
                            path_points(index + 1, nodecount_end);
                            continue;
                        }
                        else
                        {
                            sub_step_node = new Node2D_star();
                            (sub_step_node.loc) = step_func(start_nodes[index].loc, step_node.loc, step_size);
                        }

                        if (isValid(obst, start_nodes[index].loc, sub_step_node.loc) == false)
                        {
                            sub_state = 1;
                            continue;
                        }
                        else
                        {
                            sub_step_node.parent = start_nodes[index];
                            sub_step_node.cost = start_nodes[index].cost + step_size;
                            minimal_cost(obst, sub_step_node);
                            //rewiring(img,obst,sub_step_node);
                            //start_nodes[nodecount_start] = sub_step_node;
                            start_nodes.Add(sub_step_node);
                            nodecount_start++;
                        }
                        img.Line((sub_step_node.loc).x, (sub_step_node.loc).y, (sub_step_node.parent.loc).x, (sub_step_node.parent.loc).y, Scalar.Yellow, 1, LineTypes.Link8);
                        Cv2.ImShow("IMAGE", img);
                        Cv2.WaitKey(50);
                    }

                }




            }
            Cv2.ImShow("IMAGE", img);
            Cv2.WaitKey(0);
        }

        public int rrt_connect_Main()
        {
            Cv2.NamedWindow("IMAGE", WindowFlags.AutoSize);
            Mat img = Cv2.ImRead("C:\\Users\\ywang246\\source\\repos\\rrt_Connect_CSharp\\rrt_Connect_CSharp\\rrt_connect_2.png", ImreadModes.Color);
            Mat obst = Cv2.ImRead("C:\\Users\\ywang246\\source\\repos\\rrt_Connect_CSharp\\rrt_Connect_CSharp\\rrt_connect_2.png", ImreadModes.Grayscale);

            Mat start = new Mat(img.Rows, img.Cols, MatType.CV_8UC3,Scalar.Black);
            Mat end = new Mat(img.Rows, img.Cols, MatType.CV_8UC3, Scalar.Black);

            for (int i = 0; i < img.Rows; i++)
            {
                for (int j = 0; j < img.Cols; j++)
                {
               
                    if (img.At<Vec3b>(i, j)[1] > 100 && img.At<Vec3b>(i, j)[2] < 100)
                    {
                        start.At<Vec3b>(i, j)[1] = img.At<Vec3b>(i, j)[1];
                    }
                    if (img.At<Vec3b>(i, j)[2] > 100 && img.At<Vec3b>(i, j)[1] < 100)
                    {
                        end.At<Vec3b>(i, j)[2] = img.At<Vec3b>(i, j)[2];
                    }
                }
            }

            Mat gray_start=new Mat(start.Rows, start.Cols, MatType.CV_8UC3);
            Cv2.CvtColor(start, gray_start, ColorConversionCodes.BGR2GRAY);

            Mat gray_end= new Mat(end.Rows, end.Cols, MatType.CV_8UC3);
            Cv2.CvtColor(end, gray_end, ColorConversionCodes.BGR2GRAY);

            Cv2.MedianBlur(gray_start, gray_start,5);
            Cv2.MedianBlur(gray_end, gray_end, 5);

            List<Vec3f> circle_start = new List<Vec3f>();
            List<Vec3f> circle_end = new List<Vec3f>();


            Cv2.Canny(gray_start,gray_start,100,100*3,3);
            Cv2.Canny(gray_end, gray_end, 100, 100 * 3, 3);

            //List<List<Point>> contours_start = new List<List<Point>>();
            //List<List<Point>> contours_end= new List<List<Point>>();

            //List<Vec4i> hierarchy_start = new List<Vec4i>();
           // List<Vec4i> hierarchy_end = new List<Vec4i>();

            OpenCvSharp.Point[][] contours_start;
            OpenCvSharp.Point[][] contours_end;
            HierarchyIndex[] hierarchy_start;
            HierarchyIndex[] hierarchy_end;

            Cv2.FindContours(gray_start, out contours_start,  out hierarchy_start, RetrievalModes.List, ContourApproximationModes.ApproxSimple, new Point(0,0));
            Cv2.FindContours(gray_end, out contours_end, out hierarchy_end, RetrievalModes.List, ContourApproximationModes.ApproxSimple, new Point(0, 0));

            Cv2.DrawContours(gray_start, contours_start, -1, 255, 1, LineTypes.Link8, hierarchy_start, 2, new Point(0, 0));
            Cv2.DrawContours(gray_end, contours_end, -1, 255, 1, LineTypes.Link8, hierarchy_end, 2, new Point(0, 0));

            Moments m_start = new Moments(gray_start, true);

            Point p_start=new Point(m_start.M10 / m_start.M00, m_start.M01 / m_start.M00);
            Cv2.Circle(img, p_start, 5, Scalar.White, -1);
            Cv2.Circle(obst, p_start, 20, new Scalar(0), -1);

            Moments m_end = new Moments(gray_end, true);

            Point p_end = new Point(m_end.M10 / m_end.M00, m_end.M01 / m_end.M00);
            Cv2.Circle(img, p_end, 5, Scalar.White, -1);
            Cv2.Circle(obst, p_end, 20, new Scalar(0), -1);

            for (int i = 0; i < obst.Rows; i++)
            {
                for (int j = 0; j < obst.Cols; j++)
                {
                    if (obst.At<byte>(i, j) > 100)
                    {
                        obst.At<byte>(i, j) = 255;
                    }
                    else
                    {
                        obst.At<byte>(i, j) = 0;
                    }
                }
            }
            int size = 3;

            Mat element = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(2 * size + 1, 2 * size + 1), new Point(size, size));


            for (int i = 0; i < 3; i++)
                Cv2.Dilate(obst, obst, element);

            /*----------------------------------RRT----------------------------------*/
            point2D s, e;
            s.x = p_start.X;
            s.y = p_start.Y;
            e.x = p_end.X;
            e.y = p_end.Y;


            rrt_connect(s, e, img, obst);


            return 0;
        }

       
    }



}
