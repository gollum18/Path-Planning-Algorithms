using System;
using System.IO;
using System.Threading.Tasks;
using System.Linq;

using Path_Planning_Algorithms.Maps;
using Path_Planning_Algorithms.Algorithms;
using static Path_Planning_Algorithms.Algorithms.Utility;
using System.Collections.Generic;

namespace Path_Planning_Algorithms
{
    public class Program
    {
        public static void Main(string[] args)
        {
            if (args.Contains("/h"))
            {
                PrintHelp();
            }
            else if (args.Contains("/ng"))
            {
                //TODO: Launch the GUI
            }
            else
            {
                GatherData(int.Parse(args[0]), double.Parse(args[1]), int.Parse(args[2]), int.Parse(args[3]), int.Parse(args[4]), int.Parse(args[5]));
            }

            //Application.EnableVisualStyles();
            //Application.Run(new TestForm());
        }

        public static void PrintHelp()
        {
            Console.WriteLine("Usage:");
            Console.WriteLine("PPTest [maps] [percentage] [map_columns] [map_rows] [obs_columns] [obs_rows]");
            Console.WriteLine("Where: [required]");
            Console.WriteLine("[maps]: Int, Number of maps to generate.");
            Console.WriteLine("[percentage]: Double, Percentage of obstacles on map, DO NOT EXCEED .50 OR GO BELOW .01");
            Console.WriteLine("[map_columns]: Int, number of columns in each map.");
            Console.WriteLine("[map_rows]: Int, number of rows in each map.");
            Console.WriteLine("[obs_columns]: Int, number of columns per each obstacle.");
            Console.WriteLine("[obs_rows]: Int, number of rows per each obstacle.");
        }

        public static async void GatherData(int amt, double per, int cols,
            int rows, int ocols, int orows)
        {
            if (per < .01 || per > .50)
            {
                throw new Exception($"ERROR: The percentage entered [{per}] is outside the acceptable range!");
            }

            Map[] maps = new Map[amt];
            Stats[] stats = new Stats[4];
            Task[] tasks = new Task[4];
            DateTime now = DateTime.Now;
            TaskFactory factory = new TaskFactory();

            maps = GetMaps(amt, per);

            for (int i = 0; i < stats.Length; i++)
            {
                Stats stat = new Stats((Map[])CopyListOfObjects(maps), (AgentType)i, now);
                stats[i] = stat;
                tasks[i] = new Task(delegate
                {
                    stat.CalcStats();
                });
            }

            foreach (Task task in tasks)
            {
                task.Start();
            }
            
            Task.WaitAll(tasks);

            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine();
            Console.WriteLine(stats[0].ToString());
            Console.WriteLine(stats[1].ToString());
            Console.WriteLine(stats[2].ToString());
            Console.WriteLine(stats[3].ToString());

            //Calculate the T-Scores
            List<double> aList = new List<double>(4);
            List<double> tList = new List<double>(4);

            aList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[0].Lengths, stats[1].Lengths), 2));
            aList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[0].Headings, stats[1].Headings), 2));
            aList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[0].Degrees, stats[1].Degrees), 2));
            aList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[0].Times, stats[1].Times), 2));

            tList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[2].Lengths, stats[3].Lengths), 2));
            tList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[2].Headings, stats[3].Headings), 2));
            tList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[2].Degrees, stats[3].Degrees), 2));
            tList.Add(Math.Round(Stats.RepeatedMeasuresTScore(stats[2].Times, stats[3].Times), 2));

            for (int i = 0; i < 4; i++)
            {
                Console.WriteLine($"{GetTHeading(i)}, A*/A*PS: {aList[i]}");
                Console.WriteLine($"{GetTHeading(i)}, Theta*/S-Theta*: {tList[i]}");
            }

            WriteData(stats, aList, tList, per);

            maps = null;
            stats = null;
            tasks = null;
            GC.Collect();

            Console.ReadLine();
        }

        private static void WriteData(Stats[] stats, List<double> aList, List<double> tList, double per)
        {
            using (StreamWriter writer = new StreamWriter($"TraversalTestResults_{per*100}%.csv"))
            {
                writer.WriteLine(Stats.GetCSVHeader());
                for (int i = 0; i < stats.Count(); i++)
                {
                    writer.WriteLine(stats[i].GetCSV());
                }
            }

            using (StreamWriter writer = new StreamWriter($"T-TestResults_{per * 100}%.txt"))
            {
                for (int i = 0; i < aList.Count; i++) {
                    writer.WriteLine($"{GetTHeading(i)}, A*/A*PS: {aList[i]}");
                    writer.WriteLine($"{GetTHeading(i)}, Theta*/S-Theta*: {tList[i]}");
                }
            }
        }

        private static string GetTHeading(int index)
        {
            switch (index)
            {
                case 0:
                    return "T-Score For Length";
                case 1:
                    return "T-Score For Heading Changes";
                case 2:
                    return "T-Score For Degrees";
                case 3:
                    return "T-Score For Traversal Time";
                default:
                    return "";
            }
        }

        public static Map[] GetMaps(int amt, double per)
        {
            Map[] maps = new Map[amt];

            for (int index = 0; index < maps.Length; index++)
            {
                maps[index] = Map.InstanceOf(500, 500, per, 4, 4);
            }

            return maps;
        }
    }
}
