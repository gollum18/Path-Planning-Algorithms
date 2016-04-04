using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Data;
using System.Data.OleDb;

using System.Windows.Forms;

using Path_Planning_Algorithms.Maps;
using MathNet.Numerics.Statistics;

namespace Path_Planning_Algorithms.Algorithms
{
    public class Stats
    {
        public AgentType Type { get; private set; }
        public Map[] Maps { get; private set; }
        public List<double> Lengths { get; private set; }
        public List<double> Degrees { get; private set; }
        public List<double> Headings { get; private set; }
        public List<double> Times { get; private set; }
        public double CC_DegreesHeadings { get; private set; }
        public double Mean_Length { get; private set; }
        public double Mean_Degree { get; private set; }
        public double Mean_Heading { get; private set; }
        public double Mean_Time { get; private set; }
        public double StdDev_Length { get; private set; }
        public double StdDev_Degree { get; private set; }
        public double StdDev_Heading { get; private set; }
        public double StdDev_Time { get; private set; }
        public double Median_Length { get; private set; }
        public double Median_Degree { get; private set; }
        public double Median_Heading { get; private set; }
        public double Median_Time { get; private set; }
        public Tuple<double, double> CI_Length { get; private set; }
        public Tuple<double, double> CI_Degree { get; private set; }
        public Tuple<double, double> CI_Heading { get; private set; }
        public Tuple<double, double> CI_Time { get; private set; }
        public List<Agent> Agents { get; private set; }

        private DateTime time;

        public Stats(Map[] m, AgentType a, DateTime time)
        {
            if (m == null)
            {
                throw new ArgumentException();
            }

            Maps = m;
            Type = a;
            this.time = time;
        }

        public void CalcStats()
        {
            Lengths = new List<double>(30);
            Degrees = new List<double>(30);
            Headings = new List<double>(30);
            Times = new List<double>(30);
            Agents = new List<Agent>();

            Agent agent = null;
            
            foreach (Map map in Maps)
            {
                agent = new Agent(Type, map);
                agent.TraverseMap();
                Lengths.Add(agent.Length);
                Degrees.Add(agent.Degrees);
                Headings.Add(agent.Headings);
                Times.Add(agent.Time);
                Agents.Add(agent);
            }
            
            //Calculate the correlation coeffiCI_ent between the heading changes and degrees using the pearson method
            CC_DegreesHeadings = Correlation.Spearman(Degrees, Headings);

            Console.WriteLine("Now Calculating Means...");
            //Calculate the means for the data sets
            Mean_Length = Statistics.Mean(Lengths);
            Mean_Degree = Statistics.Mean(Degrees);
            Mean_Heading = Statistics.Mean(Headings);
            Mean_Time = Statistics.Mean(Times);

            Console.WriteLine("Now Calculating Standard Deviations...");
            //Calculate the deviations for the data sets
            StdDev_Length = Statistics.PopulationStandardDeviation(Lengths);
            StdDev_Degree = Statistics.PopulationStandardDeviation(Degrees);
            StdDev_Heading = Statistics.PopulationStandardDeviation(Headings);
            StdDev_Time = Statistics.PopulationStandardDeviation(Times);

            Console.WriteLine("Now Calculating Medians...");
            //Calculate the Median_s for the data sets
            Median_Length = Statistics.Median(Lengths);
            Median_Degree = Statistics.Median(Degrees);
            Median_Heading = Statistics.Median(Headings);
            Median_Time = Statistics.Median(Times);

            Console.WriteLine("Now Calculating Confidence Intervals...");
            //set the confidence intervals
            CI_Length = ConfidenceInterval(Mean_Length, StdDev_Length);
            CI_Degree = ConfidenceInterval(Mean_Degree, StdDev_Degree);
            CI_Heading = ConfidenceInterval(Mean_Heading, StdDev_Heading);
            CI_Time = ConfidenceInterval(Mean_Time, StdDev_Time);

            agent = null;
        }

        public Tuple<double, double> ConfidenceInterval(double mean, double StdDev_) =>
            Tuple.Create(mean - (1.96 * StdDev_), mean + (1.96 * StdDev_));

        public void ToDatabase()
        {
            using (OleDbConnection conn = new OleDbConnection(Properties.Settings.Default.DBConnString))
            {
                OleDbCommand comm = new OleDbCommand();
                comm.CommandType = CommandType.Text;
                comm.Connection = conn;
                comm.CommandText = "INSERT INTO Table_Stats"
                    + " ([DateRan],[Algorithm],[Percentage],"
                    + "[Mean_Length],[MeanHChanges],[Mean_Degree],[MeanTime],"
                    + "[MedianLength],[MedianHChanges],[MedianDegrees],[MedianTime],"
                    + "[DeviantLength],[DeviantHChanges],[DeviantDegrees],[DeviantTime],"
                    + "[LowerLength],[LowerHChanges],[LowerDegrees],[LowerTime],"
                    + "[UpperLength],[UpperHChanges],[UpperDegrees],[UpperTime],[CCHeadingDegree])"
                    + " VALUES "
                    + "(@dat,@alg,@per,"
                    + "@mnl,@mnh,@mnd,@mnt,"
                    + "@mel,@meh,@med,@met,"
                    + "@stl,@sth,@std,@stt,"
                    + "@cll,@clh,@cld,@clt,"
                    + "@cul,@cuh,@cud,@cut,@ccd);";

                //Add the parameters
                comm.Parameters.AddWithValue("@dat", time);
                comm.Parameters.AddWithValue("@alg", Type.ToString());
                comm.Parameters.AddWithValue("@per", Maps[0].Percentage);
                comm.Parameters.AddWithValue("@mnl", Math.Round(Mean_Length, 4));
                comm.Parameters.AddWithValue("@mnh", Math.Round(Mean_Heading, 4));
                comm.Parameters.AddWithValue("@mnd", Math.Round(Mean_Degree, 4));
                comm.Parameters.AddWithValue("@mnt", Math.Round(Mean_Time, 4));
                comm.Parameters.AddWithValue("@mel", Math.Round(Median_Length, 4));
                comm.Parameters.AddWithValue("@meh", Math.Round(Median_Heading, 4));
                comm.Parameters.AddWithValue("@med", Math.Round(Median_Degree, 4));
                comm.Parameters.AddWithValue("@met", Math.Round(Median_Time, 4));
                comm.Parameters.AddWithValue("@stl", Math.Round(StdDev_Length, 4));
                comm.Parameters.AddWithValue("@sth", Math.Round(StdDev_Heading, 4));
                comm.Parameters.AddWithValue("@std", Math.Round(StdDev_Degree, 4));
                comm.Parameters.AddWithValue("@stt", Math.Round(StdDev_Time, 4));
                comm.Parameters.AddWithValue("@cll", Math.Round(CI_Length.Item1));
                comm.Parameters.AddWithValue("@clh", Math.Round(CI_Heading.Item1));
                comm.Parameters.AddWithValue("@cld", Math.Round(CI_Degree.Item1));
                comm.Parameters.AddWithValue("@clt", Math.Round(CI_Time.Item1));
                comm.Parameters.AddWithValue("@cul", Math.Round(CI_Length.Item2));
                comm.Parameters.AddWithValue("@cuh", Math.Round(CI_Heading.Item2));
                comm.Parameters.AddWithValue("@cud", Math.Round(CI_Degree.Item2));
                comm.Parameters.AddWithValue("@cut", Math.Round(CI_Time.Item2));
                comm.Parameters.AddWithValue("@ccd", Math.Round(CC_DegreesHeadings, 4));
                
                try {
                    conn.Open();
                    comm.ExecuteNonQuery();
                }
                catch (OleDbException ex)
                {
                    MessageBox.Show(ex.ToString(),"Exception Encountered!",MessageBoxButtons.OK,MessageBoxIcon.Error);
                }
            }
        }

        public static String GetCSVHeader()
        {
            StringBuilder sb = new StringBuilder();

            //Column Headers
            sb.Append("Algorithm,Percentage,Heading/Degree Correlation,Mean Path Length,Mean Heading Changes, Mean Degrees,Mean Traversal Time,StdDev_ Path Length,StdDev_ Heading Changes,");
            sb.Append("StdDev_ Degrees,StdDev_ Traversal Time,Median Path Length,Median Heading Changes,Median Degrees,Median Traversal Time,CI Path Length,CI Heading Changes,CI Degrees,CI TraversalTime");
            sb.Append(Environment.NewLine);

            return sb.ToString();
        }

        public String GetCSVBody()
        {
            StringBuilder sb = new StringBuilder();

            //Column Data
            sb.Append($"{Type.ToString()},{Maps[0].Percentage * 100}%,{Math.Round(CC_DegreesHeadings, 2)},{Math.Round(Mean_Length, 2)},{Math.Round(Mean_Heading, 2)},{Math.Round(Mean_Degree, 2)},");
            sb.Append($"{Math.Round(Mean_Time, 2)},{Math.Round(StdDev_Length, 2)},{Math.Round(StdDev_Heading, 2)},{Math.Round(StdDev_Degree, 2)},{Math.Round(StdDev_Time, 2)},{Math.Round(Median_Length, 2)},");
            sb.Append($"{Math.Round(Median_Heading, 2)},{Math.Round(Median_Degree, 2)},{Math.Round(Median_Time, 2)},[{Math.Round(CI_Length.Item1)} {Math.Round(CI_Length.Item2)}],[");
            sb.Append($"{Math.Round(CI_Heading.Item1)} {Math.Round(CI_Heading.Item2)}],[{Math.Round(CI_Degree.Item1)} {Math.Round(CI_Degree.Item2)}],[");
            sb.Append($"{Math.Round(CI_Time.Item1)} {Math.Round(CI_Time.Item2)}]" + Environment.NewLine);

            return sb.ToString();
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();

            sb.AppendLine($"Stats Data for {Type.ToString()}: {Maps[0].Percentage * 100}%");
            sb.AppendLine("-Correlations-");
            sb.AppendLine($"\t Heading Changes/Degrees: [{Math.Round(CC_DegreesHeadings, 4)}]");
            sb.AppendLine("-Means-");
            sb.AppendLine($"\t Path Length: [{Math.Round(Mean_Length, 4)}]");
            sb.AppendLine($"\t Heading Changes: [{Math.Round(Mean_Heading, 4)}]");
            sb.AppendLine($"\t Degrees: [{Math.Round(Mean_Degree, 4)}]");
            sb.AppendLine($"\t Traversal Time: [{Math.Round(Mean_Time, 4)}]");
            sb.AppendLine("-Standard Deviations-");
            sb.AppendLine($"\t Path Length: [{Math.Round(StdDev_Length, 4)}]");
            sb.AppendLine($"\t Heading Changes: [{Math.Round(StdDev_Heading, 4)}]");
            sb.AppendLine($"\t Degrees: [{Math.Round(StdDev_Degree, 4)}]");
            sb.AppendLine($"\t Traversal Time: [{Math.Round(StdDev_Time, 4)}]");
            sb.AppendLine("-Medians-");
            sb.AppendLine($"\t Path Length: [{Math.Round(Median_Length, 4)}]");
            sb.AppendLine($"\t Heading Changes: [{Math.Round(Median_Heading, 4)}]");
            sb.AppendLine($"\t Degrees: [{Math.Round(Median_Degree, 4)}]");
            sb.AppendLine($"\t Traversal Time: [{Math.Round(Median_Time, 4)}]");
            sb.AppendLine("-Confidence Intervals-");
            sb.AppendLine($"\t Path Length: [{Math.Round(CI_Length.Item1, 4)} -> {Math.Round(CI_Length.Item2, 4)}]");
            sb.AppendLine($"\t Heading Changes: [{Math.Round(CI_Heading.Item1, 4)} -> {Math.Round(CI_Heading.Item2, 4)}]");
            sb.AppendLine($"\t Degrees: [{Math.Round(CI_Degree.Item1, 4)} -> {Math.Round(CI_Degree.Item2, 4)}]");
            sb.AppendLine($"\t Traversal Time: [{Math.Round(CI_Time.Item1, 4)} -> {Math.Round(CI_Time.Item2, 4)}]");

            return sb.ToString();
        }
    }
}
