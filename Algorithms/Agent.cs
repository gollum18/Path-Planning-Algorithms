using System;
using System.Collections.Generic;
using System.Text;
using System.Diagnostics;

using Path_Planning_Algorithms.Maps;
using static Path_Planning_Algorithms.Algorithms.Geometry;
using static Path_Planning_Algorithms.Algorithms.Utility;

namespace Path_Planning_Algorithms.Algorithms
{
    /// <summary>
    /// Agents are used to traverse the maps.
    /// </summary>
    public class Agent : ICloneable
    {
        /// <summary>
        /// Contains all proterties and methods related to interacting with the map for this agent.
        /// </summary>
        public Map Map { get; private set; }

        /// <summary>
        /// The traversal type of this agent: ASTAR, ASTARPS, THETASTAR, STHETASTAR.
        /// </summary>
        public AgentType AgentType { get; private set; }

        /// <summary>
        /// Contains the nodes waiting to be evaluated by this agents traversal method.
        /// </summary>
        private PriorityQueue<Cell> OpenList { get; set; }

        /// <summary>
        /// Contains the nodes already evaluated by this agents traversal method.
        /// </summary>
        private List<Cell> ClosedList { get; set; }

        public List<Cell> Path { get; private set; }

        /// <summary>
        /// Determines whether the path has been generated or not.
        /// </summary>
        /// <remarks>
        /// The path is considered generated even if the count of the items in the path is 0.
        /// </remarks>
        public Boolean IsPathGenerated { get { return (Path.Equals(null) ? false : true); } }

        /// <summary>
        /// The total number of cells in the path, will return 0 if the path is null.
        /// </summary>
        public int Size { get { return (Path.Equals(null) ? 0 : Path.Count); } }

        /// <summary>
        /// The total length of the path, measured from cell to cell using the standard euclidean distance formula.
        /// </summary>
        public double Length { get; private set; }

        /// <summary>
        /// The time it took for the agent to generate a path from the start cell to the finish cell, measured in milliseconds.
        /// </summary>
        public double Time { get; private set; }

        /// <summary>
        /// The total amount of heading changes in the path.
        /// </summary>
        public double Headings { get; private set; }

        /// <summary>
        /// The total degree of the path, measured as the sum of all angles created by heading changes.
        /// </summary>
        public double Degrees { get; private set; }

        private Agent()
        {
            //Used for cloning purposes
        }

        /// <summary>
        /// Creates an agent from an agent type, and a map.
        /// </summary>
        /// <param name="t">The type of the agent: ASTAR, ASTARPS, THETASTAR, STHETASTAR</param>
        /// <param name="m">The map the agent will be traversing.</param>
        public Agent(AgentType t, Map m)
        {
            //Check to make sure the necessary arguments are not null
            if (m == null || !(m is Map))
            {
                throw new ArgumentException();
            }
            else
            {
                //Set the attributes of the agent
                AgentType = t;
                Map = m;
                OpenList = new PriorityQueue<Cell>();
                ClosedList = new List<Cell>();
            }
        }



        /// <summary>
        /// Traverse the map using the method specific to the agent type.
        /// </summary>
        /// <returns></returns>
        public Boolean TraverseMap()
        {
            //Initialize and start the topwatch to count the traversal time
            Stopwatch sw = new Stopwatch();
            sw.Start();

            //Set the initial attributes of the starting cell
            Map.Start.G_Score = 0;
            Map.Start.H_Score = 0;
            Map.Start.F_Score = Map.Start.G_Score + Map.Start.H_Score;
            Map.Start.Parent = Map.Start;

            //Initialize the open list
            OpenList.Add(Map.Start);

            Cell current = null;
            Cell finish = Map.Finish;

            //While the open list is not empty...
            while (OpenList.Count != 0)
            {
                //Get the min and remove it from the list
                current = OpenList.Dequeue();

                //Loop until the current node equals the finish node
                if (current.Equals(finish))
                {
                    //If we are at the finish node, stop the traversal and return true
                    ClosedList.Add(current);
                    Path = Map.generatePath(ClosedList);
                    if (AgentType == AgentType.ASTARPS)
                    {
                        //Only post smooth if we have to
                        PostSmooth();
                    }
                    sw.Stop();
                    Time = sw.ElapsedMilliseconds;
                    GenerateStatistics();
                    return true;
                }

                ///Add the current node to the closed list to indicate we have visited it
                ClosedList.Add(current);

                //For each neighbor of the current cell...
                foreach (Cell neighbor in GenerateNeighbors(current))
                {
                    //If the neighbor is not in the closed list...
                    if (!ClosedList.Contains(neighbor))
                    {
                        //If the neighbor is not in the open list...
                        if (!OpenList.Contains(neighbor))
                        {
                            //Initialize the neighbors values as we have not seen it before
                            neighbor.G_Score = Int32.MaxValue;
                            neighbor.Parent = null;
                        }
                        //Update the neighbors values with respect to the current cells
                        updateVertex(current, neighbor);
                    }
                }
            }

            //The traversal has failed, stop the swatch and return false
            sw.Stop();
            Time = sw.ElapsedMilliseconds;
            return false;
        }

        /// <summary>
        /// Used by ASTARPS to smooth the path after it has been generated.
        /// </summary>
        /// <exception cref="Exception">If the agent type is not ASTARPS, or the finish cell is not found within the path.</exception>
        /// <remarks>This should only be run after the path has been generated.</remarks>
        private void PostSmooth()
        {
            if (AgentType != AgentType.ASTARPS)
            {
                throw new Exception("ERROR: The agent type must be ASTARPS to post smooth the map!");
            }

            Cell current = Path.Find(Map.Finish.Equals);

            if (current == null)
            {
                throw new Exception("ERROR: Unable to find the finish cell in the path!");
            }

            Cell temp = null;

            while (!current.Parent.Equals(Map.Start))
            {
                if (Map.LineOfSight(current, current.Parent.Parent))
                {
                    temp = current.Parent;
                    current.Parent = temp.Parent;
                    Path.Remove(temp);
                }
                else
                {
                    current = current.Parent;
                }
            }

            //Redraw the path
            Map.RedrawPath(Path);
        }

        private void GenerateStatistics()
        {
            //First check if the path is null or empty
            if (Path.Equals(null) || Path.Count == 0)
            {
                //If it is throw an exception telling the user it is
                throw new Exception("ERROR: Cannot generate statistics on a null or empty path!");
            }

            //Otherwise begin generation of the statistics
            //Get the last node in the graph
            Cell current = Path.Find(Map.Finish.Equals);

            double slopeAToC = 0;
            double slopeBToC = 0;

            //Check to see if the node is not null and has a parent
            if (!current.Equals(null))
            {
                if (!current.Parent.Equals(null))
                {
                    /*
                    *   Walks through the path one cell at a time starting at the 
                    *   last cell in the path and ending at the first.
                    */
                    while (!current.Equals(Map.Start))
                    {
                        //Determine the slope of the lines that matter
                        slopeAToC = Slope(current, current.Parent);
                        slopeBToC = Slope(current.Parent.Parent,
                            current.Parent);

                        //If the slopes do not equal then we have a heading change
                        if (slopeAToC != slopeBToC)
                        {
                            //Calculate the angle of the change
                            double angle = LawOfCosines(Angle.C,
                                current.Parent.Parent, current, current.Parent);

                            Degrees += ToDegrees(UnitCircleMeasure(angle));

                            //Increment the heading changes counter
                            Headings++;
                        }

                        //Add the length of the distance from the current cell to the parent cell
                        Length += Distance(current, current.Parent);

                        //Set the current cell to the parent of the current cell
                        current = current.Parent;
                    }
                }
                else
                {
                    throw new Exception("ERROR! The finish node has no traceable parent!");
                }
            }
            else
            {
                throw new Exception("ERROR: Unable to find the finish node in the path!");
            }
        }

        private void updateVertex(Cell current, Cell next)
        {
            double gOld = next.G_Score;
            ComputeCosts(current, next);

            if (next.G_Score < gOld)
            {
                OpenList.RemoveAll(next.Equals);

                if (OpenList.Count != 0)
                {
                    if (AgentType == AgentType.ASTAR || AgentType == AgentType.ASTARPS)
                    {
                        if (OpenList.Peek().F_Score == next.F_Score)
                        {
                            BreakAStarTie(OpenList.Peek(), next);
                        }
                    }
                    else
                    {
                        if (OpenList.Peek().F_Score == next.F_Score)
                        {
                            BreakThetaStarTie(OpenList.Peek(), next);
                        }
                    }
                }

                OpenList.Add(next);
            }
        }

        private void ComputeCosts(Cell current, Cell next)
        {
            if (AgentType == AgentType.ASTAR || AgentType == AgentType.ASTARPS)
            {
                if (current.G_Score + GetMovementCost(current, next) < next.G_Score)
                {
                    next.G_Score = current.G_Score + GetMovementCost(current, next);
                    next.H_Score = OctileDistance(next, Map.Finish);
                    next.F_Score = next.G_Score + next.H_Score;
                    next.Parent = current;
                }
            }
            else
            {
                if (Map.LineOfSight(current.Parent, next))
                {
                    if (current.Parent.G_Score + GetMovementCost(current.Parent, next) < next.G_Score)
                    {
                        next.G_Score = current.Parent.G_Score + GetMovementCost(current.Parent, next);
                        next.H_Score = EuclideanDistance(next, Map.Finish);
                        next.F_Score = next.G_Score + next.H_Score + (AgentType != AgentType.STHETASTAR ? 0 : ComputeBeta(Map.Finish, next, new Cell(next.X, Map.Finish.Y)));
                        next.Parent = current.Parent;
                    }
                }
                else
                {
                    if (current.G_Score + GetMovementCost(current, next) < next.G_Score)
                    {
                        next.G_Score = current.G_Score + GetMovementCost(current, next);
                        next.H_Score = EuclideanDistance(next, Map.Finish);
                        next.F_Score = next.G_Score + next.H_Score + (AgentType != AgentType.STHETASTAR ? 0 : ComputeBeta(Map.Finish, next, new Cell(next.X, Map.Finish.Y)));
                        next.Parent = current;
                    }
                }
            }
        }

        private double ComputeBeta(Cell A, Cell B, Cell C) =>
            Math.Abs(Map.Trajectory - ToDegrees(UnitCircleMeasure(LawOfCosines(Angle.A, A, B, C))));

        private double GetMovementCost(Cell current, Cell next) =>
            Slope(current, next) == 0 ? 1.0d : 1.4d;

        private List<Cell> GenerateNeighbors(Cell current)
        {
            List<Cell> l = new List<Cell>(8);

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                    {
                        continue;
                    }
                    else if (Map.IsValidCell(current.X + x, current.Y + y))
                    {
                        l.Add(new Cell(current.X + x, current.Y + y));
                    }
                }
            }

            return l;
        }

        private void BreakAStarTie(Cell c1, Cell c2)
        {
            if (c1.H_Score < c2.H_Score)
            {
                c1.F_Score = (c1.F_Score * .9999f);
            }
            else if (c2.H_Score < c1.H_Score)
            {
                c2.F_Score = (c2.F_Score * .9999f);
            }
            else {
                Random r = new Random();
                if (r.Next(2) == 0)
                {
                    c1.F_Score = (c1.F_Score * .9999f);
                }
                else {
                    c2.F_Score = (c2.F_Score * .9999f);
                }
                r = null;
            }
        }

        private void BreakThetaStarTie(Cell c1, Cell c2)
        {
            if (c1.G_Score < c2.G_Score)
            {
                c1.F_Score = (c1.F_Score * .9999f);
            }
            else if (c2.G_Score < c1.G_Score)
            {
                c2.F_Score = (c2.F_Score * .9999f);
            }
            else {
                Random r = new Random();
                if (r.Next(2) == 0)
                {
                    c1.F_Score = (c1.F_Score * .9999f);
                }
                else {
                    c2.F_Score = (c2.F_Score * .9999f);
                }
                r = null;
            }
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();

            sb.AppendLine($"Path Data for ({Map.Percentage * 100})% obstacles:");
            sb.AppendLine($"\tPath Size: [{Size}]");
            sb.AppendLine($"\tPath Length: [{Length}]");
            sb.AppendLine($"\tPath Degree: [{Degrees}]");
            sb.AppendLine($"\tPath Heading Changes: [{Headings}]");
            sb.AppendLine($"\tPath Traversal Time: [{Time}ms]");

            return sb.ToString();
        }

        public object Clone()
        {
            Agent clone = new Agent();

            clone.AgentType = AgentType;
            clone.ClosedList = (List<Cell>)CopyListOfObjects(ClosedList);
            clone.Degrees = Degrees;
            clone.Headings = Headings;
            clone.Length = Length;
            clone.Map = (Map)Map.Clone();
            clone.OpenList = new PriorityQueue<Cell>(OpenList);
            clone.Path = (List<Cell>)CopyListOfObjects(Path);
            clone.Time = Time;

            return clone;
        }
    }
}
