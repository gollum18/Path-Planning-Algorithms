using Path_Planning_Algorithms.Algorithms;
using System;
using System.Collections.Generic;
using System.Text;

using static Path_Planning_Algorithms.Algorithms.Geometry;

namespace Path_Planning_Algorithms.Maps
{
    public class Map : ICloneable
    {
        public static readonly double[] PERCENTAGES = {
            .05,
            .10,
            .20,
            .30,
            .50
        };

        private Map(int cols, int rows, double per, 
            int obsCols, int obsRows)
        {
            if (cols > 0 && rows > 0 && per > 0 && per <= .50 && obsCols > 0 && obsRows > 0)
            {
                Percentage = per;
                ObstacleColumns = obsCols;
                ObstacleRows = obsRows;

                Cells = new State[cols, rows];

                intialize();
            }
            else
            {
                throw new ArgumentException();
            }
        }

        private Map(State[,] map)
        {
            Cells = map;
        }



        public State[,] Cells { get; private set; }
        public int Columns { get { return Cells.GetLength(0); } }
        public int Rows { get { return Cells.GetLength(1); } }
        public int ObstacleColumns { get; private set; }
        public int ObstacleRows { get; private set; }
        public double Percentage { get; private set; }
        public double Trajectory { get; private set; }
        public Cell Start { get; set; }
        public Cell Finish { get; set; }

        private void intialize()
        {
            for (int x = 0; x < Columns; x++)
            {
                for (int y = 0; y < Rows; y++)
                {
                    Cells[x, y] = State.EMPTY;
                }
            }
        }

        public static Map InstanceOf(int cols, int rows, double per, 
            int obsCols, int obsRows)
        {
            Map map = new Map(cols, rows, per, obsCols, obsRows);

            int nBlockedCells = (int)(per * cols * rows) / 100;
            int x = 0, y = 0;

            Random r = new Random();

            //Protect Top Row
            for (int i = 0; i < cols - 1; i++)
            {
                map.Set(State.PROTECTED, i, 0);
            }

            //Protect Bottom Row
            for (int i = 0; i < cols - 1; i++)
            {
                map.Set(State.PROTECTED, i, cols - 1);
            }

            //Protect Left Row
            for (int i = 0; i < cols - 1; i++)
            {
                map.Set(State.PROTECTED, 0, i);
            }

            //Protect Right Row
            for (int i = 0; i < cols - 1; i++)
            {
                map.Set(State.PROTECTED, cols - 1, i);
            }

            while (nBlockedCells > 0)
            {
                //Randomly generate the x and y for the obstacle
                x = r.Next(cols);
                y = r.Next(rows);

                //Generate the obstacles
                for (int ox = x; ox <= (x + obsCols); ox++)
                {
                    for (int oy = y; oy <= (y + obsRows); oy++)
                    {
                        if (ox >= 1 && ox < cols - 1 && oy >= 1 && oy < rows - 1)
                        {
                            //As long as we have more obstacles to place and we are not at a protected space
                            if (nBlockedCells > 0 && map.Get(ox, oy) != State.PROTECTED)
                            {
                                //Place the obstacle
                                map.Set(State.OBSTACLE, ox, oy);
                            }
                        }
                    }
                }

                //Generate the protective borders around the obstacles
                for (int sx = x - 1; sx <= (x + obsCols + 1); sx++)
                {
                    for (int sy = y - 1; sy <= (y + obsRows + 1); sy++)
                    {
                        if (sx >= 0 && sx < cols && sy >= 0 && sy < rows)
                        {
                            if (map.Get(sx, sy) == State.EMPTY)
                            {
                                //Place the border
                                map.Set(State.PROTECTED, sx, sy);
                            }
                        }
                    }
                }
                nBlockedCells--;
            }

            //Create the start and finish
            map.Start = new Cell(0, 0);
            map.Finish = new Cell(cols - 1, (cols - 1) - r.Next((int)((rows - 1) * .20)));

            //Set the start and finish nodes
            map.Set(State.START, map.Start.X, map.Start.Y);
            map.Set(State.FINISH, map.Finish.X, map.Finish.Y);

            //Set the trajectory
            map.Trajectory = ToDegrees(UnitCircleMeasure(LawOfCosines(Angle.A, map.Finish, map.Start, new Cell(map.Start.X, map.Finish.Y))));

            return map;
        }

        public List<Cell> generatePath(List<Cell> closed)
        {
            List<Cell> path = new List<Cell>();

            Cell current = closed.Find(Finish.Equals);

            if (current != null)
            {
                while (!current.Equals(Start))
                {
                    if (!current.Equals(Finish))
                    {
                        Cells[current.X, current.Y] = State.PATH;
                    }
                    path.Add(current);
                    current = current.Parent;
                }
            }
            else
            {
                throw new KeyNotFoundException();
            }

            return path;
        }

        public void RedrawPath(List<Cell> path)
        {
            //First reset the nodes on the map that are paths, unfortunately this carries O(n^2)
            for (int x = 0; x < Columns; x++)
            {
                for (int y = 0; y < Rows; y++)
                {
                    if (Cells[x, y] == State.PATH)
                    {
                        Cells[x, y] = State.EMPTY;
                    }
                }
            }

            //Next redraw the path
            foreach (Cell cell in path)
            {
                Cells[cell.X, cell.Y] = State.PATH;
            }
        }

        public bool InBounds(int x, int y) => (x >= 0 && x < Columns && y >= 0 && y < Rows ? true : false);
        public bool IsValidCell(int x, int y) => (InBounds(x, y) && Get(x, y) != State.OBSTACLE ? true : false);
        public State Get(int x, int y) => Cells[x, y];
        public void Set(State s, int x, int y) => Cells[x, y] = s;

        public object Clone()
        {
            State[,] map = new State[Columns, Rows];

            for (int x = 0; x < Columns; x++)
            {
                for (int y = 0; y < Rows; y++)
                {
                    map[x, y] = Cells[x, y];
                }
            }

            Map clone = new Map(map);
            
            clone.ObstacleColumns = ObstacleColumns;
            clone.ObstacleRows = ObstacleRows;
            clone.Percentage = Percentage;
            clone.Trajectory = Trajectory;
            clone.Start = Start;
            clone.Finish = Finish;

            return clone;
        }

        /// <summary>
        /// Determines the line of sight between two cells.
        /// </summary>
        /// <param name="current">The cell currently being evaluated.</param>
        /// <param name="goal">The potential next cell.</param>
        /// <returns>True if there is line of sight from the current cell to the next, false if not.</returns>
        public bool LineOfSight(Cell current, Cell goal)
        {
            int xO = current.X, xT = goal.X;
            int yO = current.Y, yT = goal.Y;
            int dX = xT - xO, dY = yT - yO;
            int sX = 0, sY = 0;
            int f = 0;

            if (dY < 0)
            {
                dY = -dY;
                sY = -1;
            }
            else {
                sY = 1;
            }

            if (dX < 0)
            {
                dX = -dX;
                sX = -1;
            }
            else {
                sX = 1;
            }

            if (dX >= dY)
            {
                while (xO != xT)
                {
                    f = f + dY;
                    //CASE 1:
                    if (f >= dX)
                    {
                        if (!IsValidCell(xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))
                        {
                            return false;
                        }
                        yO = yO + sY;
                        f = f - dX;
                    }

                    //CASE 2:
                    if (f != 0 && !IsValidCell(xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))
                    {
                        return false;
                    }

                    //CASE 3:
                    if (dY == 0 &&
                              !IsValidCell(xO + ((sX - 1) / 2), yO) &&
                              !IsValidCell(xO + ((sX - 1) / 2), yO - 1))
                    {
                        return false;
                    }
                    xO = xO + sX;
                }
            }
            else {
                while (yO != yT)
                {
                    f = f + dX;
                    //CASE 1:
                    if (f >= dY)
                    {
                        if (!IsValidCell(xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))
                        {
                            return false;
                        }
                        xO = xO + sX;
                        f = f - dY;
                    }

                    //CASE 2:
                    if (f != 0 && !IsValidCell(xO + ((sX - 1) / 2), yO + ((sY - 1) / 2)))
                    {
                        return false;
                    }

                    //CASE 3:
                    if (dY == 0 &&
                              !IsValidCell(xO, yO + ((sY - 1) / 2)) &&
                              !IsValidCell(xO - 1, yO + ((sY - 1) / 2)))
                    {
                        return false;
                    }
                    yO = yO + sY;
                }
            }
            return true;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();

            for (int x = 0; x < Columns; x++)
            {
                for (int y = 0; y < Rows; y++)
                {
                    if (Get(x, y) == State.PATH)
                    {
                        sb.Append("P|");
                    }
                    else if (Get(x, y) == State.OBSTACLE)
                    {
                        sb.Append("O|");
                    }
                    else if (Get(x, y) == State.START)
                    {
                        sb.Append("S|");
                    }
                    else if (Get(x, y) == State.FINISH)
                    {
                        sb.Append("F|");
                    }
                    //Otherwise it it is empty, append - and a separator
                    else {
                        sb.Append("-|");
                    }
                }
                //Append a line separator at the end of the row
                sb.AppendLine();
            }

            return sb.ToString();
        }
    }
}
