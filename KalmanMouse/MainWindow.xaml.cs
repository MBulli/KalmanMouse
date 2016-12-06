using System;
using System.Collections.Generic;
using System.IO.Packaging;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Ink;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using MathNet.Numerics.LinearAlgebra;

namespace WpfApplication1
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        GaussianRandom rnd;

        // [x y x' y']

        private Point curMousePos;

        private static double sigma = 0.01;

        Matrix<double> Pprev = Matrix<double>.Build.DiagonalOfDiagonalArray(new []{ sigma, sigma, sigma / 2, sigma / 2 });
        Vector<double> Xprev = Vector<double>.Build.Dense(4);

        private bool running = true;

        private double dt = 0.016;

        private Point prevMousePos;

        public MainWindow()
        {
            InitializeComponent();

            rnd = new GaussianRandom(mean: 0, standardDeviation: sigma);
            Task.Run(() =>
            {
                while (running)
                {
                    IterateKalman();
                    Thread.Sleep(TimeSpan.FromSeconds(dt));
                }
            });
        }

        public void IterateKalman()
        {
            if (prevMousePos == curMousePos) return;

            var A = CreateMatrix.DenseOfArray<double>(new double[,] {
                { 1, 0, dt, 0},
                { 0, 1, 0, dt},
                { 0, 0, 1, 0 },
                { 0, 0, 0, 1 },
            });

            var B = CreateVector.DenseOfArray<double>(new[] {
                0.5 * dt*dt,
                0.5 * dt*dt,
                0,
                0
            });

            var H = CreateMatrix.DiagonalIdentity<double>(4);
            var C = CreateMatrix.DiagonalIdentity<double>(4);

            var G = CreateVector.DenseOfArray<double>(new[] {0.5*dt*dt, 0.5*dt*dt, dt, dt});

            var Q = G.ToColumnMatrix()*G.ToRowMatrix()*10;

            var mesuredPos = curMousePos;
            mesuredPos.Offset(rnd.NextDouble(), rnd.NextDouble());

            var M = CreateVector.DenseOfArray<double>(new[] {
                mesuredPos.X,
                mesuredPos.Y,
                (curMousePos.X - prevMousePos.X) / dt,
                (curMousePos.Y - prevMousePos.Y) / dt,
            });

            var Z = CreateVector.Dense<double>(4, value: 0);

            var R = CreateMatrix.Diagonal<double>(new[]
            {
                sigma, sigma, sigma/2, sigma/2
            });

            // No acceleration
            double u = 0;
            double w = 0;

            var Xkp = A * Xprev + B * u + w;

            var Pkp = A * (Pprev * A.Transpose()) + Q;

            var Yk = C * M + Z;

            var K = (Pkp * H.Transpose()) * (H * (Pkp * H.Transpose()) + R).Inverse();

            Console.WriteLine($"K = {K}");

            var Xk = Xkp + K * (Yk - H * Xkp);

            var Pk = (CreateMatrix.DiagonalIdentity<double>(4) - K * H) * Pkp;

            //Console.WriteLine($"Pk = {Pk}");

            Xprev = Xk;
            Pprev = Pk;
            
            //inkCanvas.Strokes.Add(new Stroke(new StylusPointCollection(new[] { pos })));
            AddToLine(new Point(Xprev[0], Xprev[1]));
            DrawPoint(mesuredPos, Colors.Red);
            prevMousePos = curMousePos;
        }

        private void DrawPoint(Point p, Color c)
        {
            Dispatcher.Invoke(() =>
            {
                inkCanvasMeasured.Strokes.Add(new Stroke(new StylusPointCollection(new[] {p}), new DrawingAttributes {Color = c}));
                if (inkCanvasMeasured.Strokes.Count > 50)
                {
                    inkCanvasMeasured.Strokes.RemoveAt(0);
                }
            });
        }

        
        private void AddToLine(Point p)
        {

            Dispatcher.Invoke(() =>
            {
                if (inkCanvasEstimated.Strokes.Count == 0)
                {
                    inkCanvasEstimated.Strokes.Add(new Stroke(new StylusPointCollection(new[] { p }),
                        new DrawingAttributes {Color = Colors.Green}));
                }
                else
                {
                    Stroke stroke = inkCanvasEstimated.Strokes[0];
                    stroke.StylusPoints.Add(new StylusPoint(p.X, p.Y));
                    if (stroke.StylusPoints.Count > 50)
                    {
                        stroke.StylusPoints.RemoveAt(0);
                    }
                }
            });
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            curMousePos = e.GetPosition(inkCanvasMeasured);
        }
    }
}
