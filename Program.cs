using System;
using System.IO;
using System.Threading;

using Path_Planning_Algorithms.Maps;
using Path_Planning_Algorithms.Algorithms;
using Path_Planning_Algorithms.GUI;

using static Path_Planning_Algorithms.Algorithms.Utility;

using System.Windows.Forms;

namespace Path_Planning_Algorithms
{
    public class Program
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.Run(new TestForm());

            //GatherData();
        }

        //public static void GatherData()
        //{
        //    Map[] maps = new Map[100];
        //    ManualResetEvent[] e = new ManualResetEvent[20];
        //    Stats[] s = new Stats[20];
        //    DateTime now = DateTime.Now;

        //    for (int i = 0; i < e.Length; i++)
        //    {
        //        e[i] = new ManualResetEvent(false);
        //    }

        //    maps = GetMaps(100, .05);

        //    s[0] = new Stats(CloneMapArray(maps), AgentType.ASTAR, e[0], now);
        //    ThreadPool.QueueUserWorkItem(s[0].ThreadPoolCallback, 0);
        //    s[1] = new Stats(CloneMapArray(maps), AgentType.ASTARPS, e[1], now);
        //    ThreadPool.QueueUserWorkItem(s[1].ThreadPoolCallback, 1);
        //    s[2] = new Stats(CloneMapArray(maps), AgentType.THETASTAR, e[2], now);
        //    ThreadPool.QueueUserWorkItem(s[2].ThreadPoolCallback, 2);
        //    s[3] = new Stats(CloneMapArray(maps), AgentType.STHETASTAR, e[3], now);
        //    ThreadPool.QueueUserWorkItem(s[3].ThreadPoolCallback, 3);

        //    maps = GetMaps(100, .10);

        //    s[4] = new Stats(CloneMapArray(maps), AgentType.ASTAR, e[4], now);
        //    ThreadPool.QueueUserWorkItem(s[4].ThreadPoolCallback, 4);
        //    s[5] = new Stats(CloneMapArray(maps), AgentType.ASTARPS, e[5], now);
        //    ThreadPool.QueueUserWorkItem(s[5].ThreadPoolCallback, 5);
        //    s[6] = new Stats(CloneMapArray(maps), AgentType.THETASTAR, e[6], now);
        //    ThreadPool.QueueUserWorkItem(s[6].ThreadPoolCallback, 6);
        //    s[7] = new Stats(CloneMapArray(maps), AgentType.STHETASTAR, e[7], now);
        //    ThreadPool.QueueUserWorkItem(s[7].ThreadPoolCallback, 7);

        //    maps = GetMaps(100, .20);

        //    s[8] = new Stats(CloneMapArray(maps), AgentType.ASTAR, e[8], now);
        //    ThreadPool.QueueUserWorkItem(s[8].ThreadPoolCallback, 8);
        //    s[9] = new Stats(CloneMapArray(maps), AgentType.ASTARPS, e[9], now);
        //    ThreadPool.QueueUserWorkItem(s[9].ThreadPoolCallback, 9);
        //    s[10] = new Stats(CloneMapArray(maps), AgentType.THETASTAR, e[10], now);
        //    ThreadPool.QueueUserWorkItem(s[10].ThreadPoolCallback, 10);
        //    s[11] = new Stats(CloneMapArray(maps), AgentType.STHETASTAR, e[11], now);
        //    ThreadPool.QueueUserWorkItem(s[11].ThreadPoolCallback, 11);

        //    maps = GetMaps(100, .30);

        //    s[12] = new Stats(CloneMapArray(maps), AgentType.ASTAR, e[12], now);
        //    ThreadPool.QueueUserWorkItem(s[12].ThreadPoolCallback, 12);
        //    s[13] = new Stats(CloneMapArray(maps), AgentType.ASTARPS, e[13], now);
        //    ThreadPool.QueueUserWorkItem(s[13].ThreadPoolCallback, 13);
        //    s[14] = new Stats(CloneMapArray(maps), AgentType.THETASTAR, e[14], now);
        //    ThreadPool.QueueUserWorkItem(s[14].ThreadPoolCallback, 14);
        //    s[15] = new Stats(CloneMapArray(maps), AgentType.STHETASTAR, e[15], now);
        //    ThreadPool.QueueUserWorkItem(s[15].ThreadPoolCallback, 15);

        //    maps = GetMaps(100, .40);

        //    s[16] = new Stats(CloneMapArray(maps), AgentType.ASTAR, e[16], now);
        //    ThreadPool.QueueUserWorkItem(s[16].ThreadPoolCallback, 16);
        //    s[17] = new Stats(CloneMapArray(maps), AgentType.ASTARPS, e[17], now);
        //    ThreadPool.QueueUserWorkItem(s[17].ThreadPoolCallback, 17);
        //    s[18] = new Stats(CloneMapArray(maps), AgentType.THETASTAR, e[18], now);
        //    ThreadPool.QueueUserWorkItem(s[18].ThreadPoolCallback, 18);
        //    s[19] = new Stats(CloneMapArray(maps), AgentType.STHETASTAR, e[19], now);
        //    ThreadPool.QueueUserWorkItem(s[19].ThreadPoolCallback, 19);

        //    WaitHandle.WaitAll(e);

        //    File.Delete("StatsData.txt");
        //    File.Delete("StatsData.csv");

        //    File.WriteAllText("StatsData.csv", Stats.GetCSVHeader());

        //    for (int i = 0; i < s.Length; i++)
        //    {
        //        File.AppendAllText("StatsData.csv", s[i].GetCSVBody());
        //        File.AppendAllText("StatData.txt", s[i].ToString());
        //    }

        //    File.Delete("TTEST_THETA_20.txt");
        //    File.Delete("TTEST_THETA_30.txt");
        //    File.Delete("TTEST_THETA_40.txt");
        //    File.Delete("TTEST_STHETA_20.txt");
        //    File.Delete("TTEST_STHETA_30.txt");
        //    File.Delete("TTEST_STHETA_40.txt");

        //    //Write out all data pertaining to the t-test
        //    for (int i = 0; i < s[10].Headings.Count; i++)
        //    {
        //        File.AppendAllText("TTEST_THETA_20.txt", s[10].Headings[i].ToString() + Environment.NewLine);
        //        File.AppendAllText("TTEST_THETA_30.txt", s[14].Headings[i].ToString() + Environment.NewLine);
        //        File.AppendAllText("TTEST_THETA_40.txt", s[18].Headings[i].ToString() + Environment.NewLine);
        //    }

        //    //Write out all data pertaining to the t-test
        //    for (int i = 0; i < s[11].Headings.Count; i++)
        //    {
        //        File.AppendAllText("TTEST_STHETA_20.txt", s[11].Headings[i].ToString() + Environment.NewLine);
        //        File.AppendAllText("TTEST_STHETA_30.txt", s[15].Headings[i].ToString() + Environment.NewLine);
        //        File.AppendAllText("TTEST_STHETA_40.txt", s[19].Headings[i].ToString() + Environment.NewLine);
        //    }
        //}

        public static Map[] GetMaps(int i, double per)
        {
            Map[] maps = new Map[i];

            for (int index = 0; index < maps.Length; index++)
            {
                maps[index] = Map.InstanceOf(500, 500, per, 4, 4);
            }

            return maps;
        }
    }
}
