using System;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Windows.Forms;

using Path_Planning_Algorithms.Maps;
using Path_Planning_Algorithms.Algorithms;

using static Path_Planning_Algorithms.Algorithms.Utility;

namespace Path_Planning_Algorithms.GUI
{
    public partial class TestForm : Form
    {
        private int Count;

        private Map[] Maps;
        private Stats[] Stats;
        private DateTime Now;

        public static readonly Color PATH_ASTAR = Color.Red;
        public static readonly Color PATH_ASTARPS = Color.Green;
        public static readonly Color PATH_THETASTAR = Color.Blue;
        public static readonly Color PATH_STHETASTAR = Color.Purple;

        public TestForm()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            btnRun.Enabled = false;
            btnCancel.Enabled = true;

            progressTraversal.Value = 0;
            numCurrentMap.Enabled = false;

            //set the max and min for map selector
            numCurrentMap.Minimum = 0;
            numCurrentMap.Maximum = numMaps.Value - 1;

            Maps = new Map[(int)numMaps.Value];
            Now = DateTime.Now;
           
            //Generate the maps
            for (int i = 0; i < Maps.Count(); i++)
            {
                Maps[i] = Map.InstanceOf((int)numCols.Value, (int)numRows.Value, (double)numPercentage.Value, 
                    (int)numObsCols.Value, (int)numRows.Value);
            }

            //Initialize the stat containers
            Stats = new Stats[4];

            Stats[0] = new Stats(CloneMapArray(Maps), AgentType.ASTAR, Now);
            Stats[1] = new Stats(CloneMapArray(Maps), AgentType.ASTARPS, Now);
            Stats[2] = new Stats(CloneMapArray(Maps), AgentType.THETASTAR, Now);
            Stats[3] = new Stats(CloneMapArray(Maps), AgentType.STHETASTAR, Now);

            //Trigger the background workers
            workerAStar.RunWorkerAsync();
            workerAStarPS.RunWorkerAsync();
            workerThetaStar.RunWorkerAsync();
            workerSThetaStar.RunWorkerAsync();

            //Load the table in the second tab
            table_StatsTableAdapter.Fill(algorithms_DataDataSet.Table_Stats);
        }

        private void DrawMap (int index)
        {
            Bitmap image = new Bitmap((int)numCols.Value, (int)numRows.Value);

            //Add the astar path
            DrawHelper(image, Stats[0].Agents[index], PATH_ASTAR);

            //Add the astarps path
            DrawHelper(image, Stats[1].Agents[index], PATH_ASTARPS);

            //Add the thetastar path
            DrawHelper(image, Stats[2].Agents[index], PATH_THETASTAR);

            //Add the sthetastar path
            DrawHelper(image, Stats[3].Agents[index], PATH_STHETASTAR);

            pictureCompare.BackgroundImage = image;
        }

        private void DrawHelper (Bitmap image, Agent agent, Color path)
        {
            //Draw the background
            for (int x = 0; x < agent.Map.Columns; x++)
            {
                for (int y = 0; y < agent.Map.Rows; y++)
                {
                    if (agent.Map.Get(x, y) == State.START)
                    {
                        image.SetPixel(x, y, path);
                    }
                    else if (agent.Map.Get(x, y) == State.FINISH)
                    {
                        image.SetPixel(x, y, path);
                    }
                    else if (agent.Map.Get(x, y) == State.OBSTACLE)
                    {
                        image.SetPixel(x, y, Color.Black);
                    }
                    else if (agent.Map.Get(x, y) == State.PATH)
                    {
                        image.SetPixel(x, y, path);
                    }
                }
            }
        }

        private void numCurrentMap_ValueChanged(object sender, EventArgs e)
        {
            DrawMap((int)(sender as NumericUpDown).Value);
        }

        private void TraversalWorkerMethod(object sender, DoWorkEventArgs e)
        {
            int sHash = sender.GetHashCode();

            if (sHash == workerAStar.GetHashCode())
            {
                Stats[0].CalcStats();
            }
            else if (sHash == workerAStarPS.GetHashCode())
            {
                Stats[1].CalcStats();
            }
            else if (sHash == workerThetaStar.GetHashCode())
            {
                Stats[2].CalcStats();
            }
            else if (sHash == workerSThetaStar.GetHashCode())
            {
                Stats[3].CalcStats();
            }

            Invoke((EventHandler) delegate
            {
                progressTraversal.Increment(25);
            });
        }

        private void TraversalWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            Count++;
            if (Count == 4)
            {
                DrawMap(0);
                numCurrentMap.Enabled = true;
                btnCancel.Enabled = false;
                btnRun.Enabled = true;
            }
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            if (workerAStar.IsBusy)
            {
                workerAStar.CancelAsync();
                workerAStar.Dispose();
            }

            if (workerAStarPS.IsBusy)
            {
                workerAStarPS.CancelAsync();
                workerAStarPS.Dispose();
            }

            if (workerThetaStar.IsBusy)
            {
                workerThetaStar.CancelAsync();
                workerThetaStar.Dispose();
            }

            if (workerSThetaStar.IsBusy)
            {
                workerSThetaStar.CancelAsync();
                workerSThetaStar.Dispose();
            }

            Count = 0;
            progressTraversal.Value = 0;
            numCurrentMap.Enabled = false;
            btnCancel.Enabled = false;
            btnRun.Enabled = true;
            System.GC.Collect();
        }
    }
}
