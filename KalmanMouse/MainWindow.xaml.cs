using System;
using System.Collections.Generic;
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
        Vector<double> Xprev = Vector<double>.Build.Dense(2);
        Matrix<double> Pprev = Matrix<double>.Build.Dense(2,2);

        private Point curMousePos;

        private double sigma = 5;

        private bool running = true;

        private double dt = 0.1;

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
            var A = Matrix<double>.Build.DenseOfArray(new double[,] {
                { 1, 0 },
                { 0, 1 },
            });

            var B = Vector<double>.Build.DenseOfArray(new[] {
                0.5 * dt*dt,
                0.5 * dt*dt,
            });

            var H = Matrix<double>.Build.DiagonalIdentity(2);
            var C = Matrix<double>.Build.DiagonalIdentity(2);

            var Q = Matrix<double>.Build.DenseOfArray(new[,]
            {
                { Math.Pow(sigma, 2), 0},
                { 0, Math.Pow(sigma, 2)},
            });

            var mesuredPos = curMousePos;
            mesuredPos.Offset(rnd.NextDouble(), rnd.NextDouble());

            var M = Vector<double>.Build.DenseOfArray(new[] {
                mesuredPos.X,
                mesuredPos.Y
            });

            var Z = Vector<double>.Build.Dense(2, value: 0);

            var R = Matrix<double>.Build.Diagonal(new[]
            {
                Math.Pow(sigma,2),
                Math.Pow(sigma,2),
            });

            // No acceleration
            double u = 0;
            double w = 0;

            var Xkp = A * Xprev + B * u + w;

            var Pkp = A * (Pprev * A.Transpose()) + Q;

            var Yk = C * M + Z;

            var K = (Pkp * H.Transpose()) * (H * (Pkp * H.Transpose()) + R).Inverse();

            //Console.WriteLine($"K = {K}");

            var Xk = Xkp + K * (Yk - H * Xkp);

            var Pk = (Matrix<double>.Build.DiagonalIdentity(2) - K * H) * Pkp;

            //Console.WriteLine($"Pk = {Pk}");

            Xprev = Xk;
            Pprev = Pk;
            
            //inkCanvas.Strokes.Add(new Stroke(new StylusPointCollection(new[] { pos })));
            AddToLine(new Point(Xprev[0], Xprev[1]));
            DrawPoint(mesuredPos, Colors.Red);
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
