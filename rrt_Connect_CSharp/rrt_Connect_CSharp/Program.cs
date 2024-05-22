using rrt_star_Connect_CSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace rrt_Connect_CSharp
{
    class Program
    {
        static void Main(string[] args)
        {
            System.Console.WriteLine("Hello world!!");
            //rrtConnect_2D rrt = new rrtConnect_2D();
            //rrt.rrt_connect_Main();
            TxRobotRRTStartConnect rrt = new TxRobotRRTStartConnect();
            rrt.vtkRRTConnectShow();
            System.Console.ReadKey();
        }
    }
}
