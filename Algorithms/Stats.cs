/**Copyright(C) 2016 Christen Ford

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see<http://www.gnu.org/licenses/>.

    For more information regarding this program or the author, please email him at
    <cford15@mail.bw.edu> or by mail to Baldwin-Wallace University, Berea OH.
**/

using System;
using System.Collections.Generic;
using System.Text;
using System.Data;
using System.Data.OleDb;
using System.Threading;
using System.Windows.Forms;

using Path_Planning_Algorithms.Maps;
using static Path_Planning_Algorithms.Algorithms.Utility;
using MathNet.Numerics.Statistics;

namespace Path_Planning_Algorithms.Algorithms
{
    public class Stats : ICloneable, IDisposable
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
        private ManualResetEvent ResetEvent;
        private DateTime Now;

        private Stats()
        {
            //Used for cloning purposes
        }

        public Stats(Map[] m, AgentType a, DateTime time)
        {
            if (m == null)
            {
                throw new ArgumentException();
            }

            Maps = m;
            Type = a;
            Now = time;
        }

        public Stats(Map[] map, AgentType type, ManualResetEvent manualResetEvent, DateTime now)
        {
            Maps = map;
            Type = type;
            ResetEvent = manualResetEvent;
            Now = now;
        }

        #region IDisposable Support
        private bool disposedValue = false; // To detect redundant calls

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    Lengths = null;
                    Headings = null;
                    Degrees = null;
                    Times = null;
                    CI_Degree = null;
                    CI_Heading = null;
                    CI_Length = null;
                    CI_Time = null;
                    ResetEvent = null;
                    Agents = null;
                }
                
                Maps = null;

                disposedValue = true;
            }
        }

        // TODO: override a finalizer only if Dispose(bool disposing) above has code to free unmanaged resources.
        // ~Stats() {
        //   // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
        //   Dispose(false);
        //}

        // This code added to correctly implement the disposable pattern.
        public void Dispose()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(true);
            // TODO: uncomment the following line if the finalizer is overridden above.
            // GC.SuppressFinalize(this);
        }
        #endregion

        public void CalcStats()
        {
            Lengths = new List<double>(30);
            Degrees = new List<double>(30);
            Headings = new List<double>(30);
            Times = new List<double>(30);
            Agents = new List<Agent>();

            Agent agent = null;

            int count = 1;
            foreach (Map map in Maps)
            {
                Console.WriteLine($"Now Processing {Type} {Maps[0].Percentage * 100}%: {count}");
                agent = new Agent(Type, map);
                agent.TraverseMap();
                Lengths.Add(agent.Length);
                Degrees.Add(agent.Degrees);
                Headings.Add(agent.Headings);
                Times.Add(agent.Time);
                Agents.Add(agent);
                count++;
            }
            
            //Calculate the correlation coefficient between the heading changes and degrees
            //This uses the spearman method to adjust for variability in populations
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

        /// <summary>
        /// Calculates the t-score for a repeated measures test.
        /// </summary>
        /// <param name="beforeScores">The scores before administering the test.</param>
        /// <param name="afterScores">The scores after administering the test.</param>
        /// <returns>T-Score for the repeated measure.</returns>
        public static double RepeatedMeasuresTScore(List<double> beforeScores, List<double> afterScores)
        {
            if (beforeScores.Count == 0 || afterScores.Count == 0 || 
                beforeScores.Count != afterScores.Count)
            {
                throw new ArgumentException("ERROR: Statistical lists do not contain the same amount of samples!");
            }

            /**
              Variable Definitions:
                M: Mean of the Difference Scores.
                SS: Sum of Squared Difference Scores.
                S2: The estimated population variance.
                S2M: The estimated variance of the distribution of means.
                SM: The estimated standard deviation of the distribution of means
            **/
            double M = 0, SS = 0, S2 = 0, S2M = 0, SM = 0;
            List<double> diffScores = new List<double>(beforeScores.Count);

            //Step One: Calculate M and SS
            for (int i = 0; i < beforeScores.Count; i++)
            {
                //Add the difference score to the mean
                M += afterScores[i] - beforeScores[i];

                //add the difference score to the list of difference scores
                diffScores.Add(afterScores[i] - beforeScores[i]);
            }

            M /= beforeScores.Count;

            foreach (double diff in diffScores)
            {
                SS += Math.Pow((diff - M), 2);
            }

            //Step Two: Calculate S2
            S2 = SS / (beforeScores.Count - 1);

            //Step Three: Calculate S2M
            S2M = S2 / beforeScores.Count;

            //Step Four: Calculate SM
            SM = Math.Sqrt(S2M);

            //Step Five: Calculate the T-Score
            return (M - 0) / SM;
        }

        public void ToDatabase()
        {
            using (OleDbConnection conn = new OleDbConnection(Properties.Settings.Default.DBConnString))
            {
                OleDbCommand comm = new OleDbCommand();
                comm.CommandType = CommandType.Text;
                comm.Connection = conn;
                comm.CommandText = "INSERT INTO Table_Stats"
                    + "([DateRan],[Algorithm],[Percentage],"
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
                comm.Parameters.AddWithValue("@dat", Now);
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

        public static string GetCSVHeader()
        {
            StringBuilder sb = new StringBuilder();

            //Column Headers
            sb.Append("Algorithm,Percentage,Heading/Degree Correlation,Mean Path Length,Mean Heading Changes, Mean Degrees,Mean Traversal Time,StdDev_ Path Length,StdDev_ Heading Changes,");
            sb.Append("StdDev_ Degrees,StdDev_ Traversal Time,Median Path Length,Median Heading Changes,Median Degrees,Median Traversal Time,CI Path Length,CI Heading Changes,CI Degrees,CI TraversalTime");
            sb.Append(Environment.NewLine);

            return sb.ToString();
        }

        public string GetCSV()
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

        public object Clone()
        {
            Stats stats = new Stats();

            stats.Agents = (List<Agent>)CopyListOfObjects(Agents);
            stats.CC_DegreesHeadings = CC_DegreesHeadings;
            stats.CI_Degree = CopyPair(CI_Degree);
            stats.CI_Heading = CopyPair(CI_Heading);
            stats.CI_Length = CopyPair(CI_Length);
            stats.CI_Time = CopyPair(CI_Time);
            stats.Degrees = (List<double>)CopyListOfPrimitives(Degrees);
            stats.Headings = (List<double>)CopyListOfPrimitives(Headings);
            stats.Lengths = (List<double>)CopyListOfPrimitives(Lengths);
            stats.Maps = (Map[])CopyListOfObjects(Maps);
            stats.Mean_Degree = Mean_Degree;
            stats.Mean_Heading = Mean_Heading;
            stats.Mean_Length = Mean_Length;
            stats.Mean_Time = Mean_Time;
            stats.Median_Degree = Median_Degree;
            stats.Median_Heading = Median_Heading;
            stats.Median_Length = Median_Length;
            stats.Median_Time = Median_Time;
            stats.StdDev_Degree = StdDev_Degree;
            stats.StdDev_Heading = StdDev_Heading;
            stats.StdDev_Length = StdDev_Length;
            stats.StdDev_Time = StdDev_Time;
            stats.Times = (List<double>)CopyListOfPrimitives(Times);
            stats.Type = Type;

            return stats;
        }

        
    }
}
