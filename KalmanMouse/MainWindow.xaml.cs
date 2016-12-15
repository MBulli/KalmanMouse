using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO.Packaging;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
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
using System.Windows.Threading;
using MathNet.Filtering.Kalman;

namespace WpfApplication1
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        GaussianRandom rnd;
        DispatcherTimer timer;

        // [x y x' y']

        private Point curMousePos;

        private static double sigma = 5;

        private bool pauseWhileNotMoving = false;

        Matrix<double> Pprev = Matrix<double>.Build.DiagonalOfDiagonalArray(new []{ sigma, sigma, sigma / 2, sigma / 2 });
        Vector<double> Xprev = Vector<double>.Build.Dense(4);

        private double dt = 0.016;

        private Point prevMousePos;

        private DiscreteKalmanFilter kalmanFilter;

        public double XSpeed => Xprev[2];
        public double YSpeed => Xprev[3];

        [DllImport("User32.dll")]
        private static extern bool SetCursorPos(int x, int y);


        public MainWindow()
        {
            InitializeComponent();

            rnd = new GaussianRandom(mean: 0, standardDeviation: sigma);
        }

        private void TimerTick(object sender, EventArgs e) => IterateKalman();

        public void IterateKalman()
        {
            if (pauseWhileNotMoving && prevMousePos == curMousePos)
                return;

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

            var H = CreateMatrix.DiagonalIdentity<double>(2, 4);
            var C = CreateMatrix.DiagonalIdentity<double>(2);

            var G = CreateVector.DenseOfArray<double>(new[] {0.5*dt*dt, 0.5*dt*dt, dt, dt});

            //var Q = G.ToColumnMatrix()*G.ToRowMatrix()*100000;
            var Q = CreateMatrix.DenseOfArray(new double[,] {
                { 0.0001, 0, 0, 0 },
                { 0, 0.0001, 0, 0 },
                { 0, 0, 0.1, 0 },
                { 0, 0, 0, 0.1 },
            });
                

            var mesuredPos = curMousePos;
            mesuredPos.Offset(rnd.NextDouble(), rnd.NextDouble());

            var M = CreateVector.DenseOfArray<double>(new[] {
                mesuredPos.X,
                mesuredPos.Y,
            });

            var Z = CreateVector.Dense<double>(2, value: 0);

            var R = CreateMatrix.Diagonal<double>(new[]
            {
                sigma * dt, sigma * dt
            });

            // No acceleration
            double u = 0;
            double w = 0;

            var Xkp = A * Xprev + B * u + w;

            var Pkp = A * (Pprev * A.Transpose()) + Q;

            var Yk = C * M + Z;

            var K = (Pkp * H.Transpose()) * (H * (Pkp * H.Transpose()) + R).Inverse();
            
            var Xk = Xkp + K * (Yk - H * Xkp);

            var Pk = (CreateMatrix.DiagonalIdentity<double>(4) - K * H) * Pkp;

            Xprev = Xk;
            Pprev = Pk;

            OnPropertyChanged(nameof(XSpeed));
            OnPropertyChanged(nameof(YSpeed));

            kalmanFilter.Predict(A, Q);
            kalmanFilter.Update(M.ToColumnMatrix(), H, R);

            //Console.WriteLine($"CurState: {kalmanFilter.State}");
            //Console.WriteLine($"CurCov: {kalmanFilter.Cov}");

            //inkCanvas.Strokes.Add(new Stroke(new StylusPointCollection(new[] { pos })));
            //AddToLine(inkCanvasEstimated1, new Point(kalmanFilter.State[0, 0], kalmanFilter.State[1, 0]), Colors.Blue);
            AddToLine(inkCanvasEstimated2, new Point(Xprev[0], Xprev[1]), Colors.Green);
            AddToLine(inkCanvasEstimated1, new Point(Xkp[0], Xkp[1]), Colors.Blue);
            DrawPoint(mesuredPos, Colors.Red);
            prevMousePos = curMousePos;
        }

        private void DrawPoint(Point p, Color c)
        {
            inkCanvasMeasured.Strokes.Add(new Stroke(new StylusPointCollection(new[] { p }), new DrawingAttributes { Color = c, Width = 7, Height = 7}));
            if (inkCanvasMeasured.Strokes.Count > 50)
            {
                inkCanvasMeasured.Strokes.RemoveAt(0);
            }
        }

        
        private void AddToLine(InkCanvas inkCanvas, Point p, Color color)
        {
            if (inkCanvas.Strokes.Count == 0)
            {
                inkCanvas.Strokes.Add(new Stroke(new StylusPointCollection(new[] { p }),
                    new DrawingAttributes { Color = color, Width = 4, Height = 4}));
            }
            else
            {
                Stroke stroke = inkCanvas.Strokes[0];
                stroke.StylusPoints.Add(new StylusPoint(p.X, p.Y));
                if (stroke.StylusPoints.Count > 50)
                {
                    stroke.StylusPoints.RemoveAt(0);
                }
            }
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            curMousePos = e.GetPosition(inkCanvasMeasured);
        }

        private void MainWindow_OnLoaded(object sender, RoutedEventArgs e)
        {
            timer = new DispatcherTimer(TimeSpan.FromSeconds(dt), DispatcherPriority.Normal, TimerTick, this.Dispatcher);
            kalmanFilter = new DiscreteKalmanFilter(
                CreateVector.Dense(new[] { inkCanvasMeasured.ActualWidth / 2, inkCanvasMeasured.ActualHeight / 2, 0.0, 0.0 }).ToColumnMatrix(),
                Pprev);
            this.WindowState = WindowState.Maximized;
            Point mid = inkCanvasMeasured.PointToScreen(new Point(inkCanvasMeasured.ActualWidth / 2, inkCanvasMeasured.ActualHeight / 2));
            SetCursorPos((int) mid.X, (int) mid.Y);
        }

        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        protected override void OnPreviewKeyDown(KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.Add:
                case Key.OemPlus:
                    sigma *= 1.5;
                    rnd.StandardDeviation = sigma;
                    break;
                case Key.Subtract:
                case Key.OemMinus:
                    sigma /= 1.5;
                    rnd.StandardDeviation = sigma;
                    break;
                case Key.Space:
                    pauseWhileNotMoving = !pauseWhileNotMoving;
                    break;
            }
        }
    }
}
