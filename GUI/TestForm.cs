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
            //TODO: Do something
        }

        private void DrawMap (int index)
        {
            Bitmap image = new Bitmap((int)numCols.Value, (int)numRows.Value);

            //TODO: Draw Something

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
            //TODO: Do Work
        }

        private void Increment()
        {
            Invoke((EventHandler)delegate
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
            if (TraversalWorker.IsBusy)
            {
                TraversalWorker.CancelAsync();
                TraversalWorker.Dispose();
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
