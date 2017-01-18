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
        private GaussianRandom rnd = new GaussianRandom(mean: 0, standardDeviation: 1);
        private DispatcherTimer timer;

        private Point curMousePos;
        private Point prevMousePos;

        private bool pauseWhileNotMoving;

        Matrix<double> Pprev;
        Vector<double> Xprev;

        // time between mesurments
        private const double dt = 0.016;

        // system Matrix
        private Matrix<double> A = CreateMatrix.DenseOfArray(new double[,] {
            { 1, 0, dt, 0},
            { 0, 1, 0, dt},
            { 0, 0, 1, 0 },
            { 0, 0, 0, 1 },
        });

        // system modification transformation Matrix
        private Matrix<double> B = CreateMatrix.DenseOfArray(new double[,] {
            { 0.5 * dt* dt },
            { 0.5 * dt* dt },
            { 0.0 },
            { 0.0 }
        });

        // system modification value vector
        private Vector<double> a = CreateVector.DenseOfArray(new double[]
        {
            0.0
        });

        // system covariance Matrix
        private Matrix<double> Q = CreateMatrix.DenseOfArray(new double[,] {
            { 0.0001, 0, 0, 0 },
            { 0, 0.0001, 0, 0 },
            { 0, 0, 0.1, 0 },
            { 0, 0, 0, 0.1 },
        });

        // state to mesurement convertion matrix
        private Matrix<double> H = CreateMatrix.DiagonalIdentity<double>(2, 4);

        // mesurement covariance Matrix
        private Matrix<double> R = CreateMatrix.DiagonalIdentity<double>(2);
        
        public double XSpeed => Xprev[2];
        public double YSpeed => Xprev[3];

        private double minSigma = 5/Math.Pow(1.5, 21);
        private double maxSigma = 5*Math.Pow(1.5, 15);
        // messurment spread
        private double sigma;
        public double Sigma {
            get { return sigma; }
            set
            {
                if (value < minSigma)
                    sigma = minSigma;
                else if (value > maxSigma)
                    sigma = maxSigma;
                else
                    sigma = value;
                R[0, 0] = sigma*dt;
                R[1, 1] = R[0, 0];
                rnd.StandardDeviation = sigma;
                OnPropertyChanged();
            }
        }

        [DllImport("User32.dll")]
        private static extern bool SetCursorPos(int x, int y);
        
        public MainWindow()
        {

            Sigma = 5;

            Pprev = Matrix<double>.Build.DiagonalOfDiagonalArray(new[] { sigma, sigma, sigma / 2, sigma / 2 });
            Xprev = Vector<double>.Build.Dense(4);

            pauseWhileNotMoving = false;

            InitializeComponent();
        }

        private void TimerTick(object sender, EventArgs e) => IterateKalman();

        public void IterateKalman()
        {
            if (pauseWhileNotMoving && prevMousePos == curMousePos)
                return;
            
            // add mesurement spread to the mouse pos
            var mesuredPos = curMousePos;
            mesuredPos.Offset(rnd.NextDouble(), rnd.NextDouble());

            // prepare mesurement Vector
            var M = CreateVector.DenseOfArray(new[] {
                mesuredPos.X,
                mesuredPos.Y,
            });

            // calc state vector prediction
            var Xkp = A * Xprev + B * a;

            // calc covariance matrix prediction
            var Pkp = A * (Pprev * A.Transpose()) + Q;

            // calc kalman gain
            var K = (Pkp * H.Transpose()) * (H * (Pkp * H.Transpose()) + R).Inverse();
            
            // calc new state
            var Xk = Xkp + K * (M - H * Xkp);

            // calc new covariance matrix
            var Pk = (CreateMatrix.DiagonalIdentity<double>(4) - K * H) * Pkp;

            // save current state for next iteration
            Xprev = Xk;
            Pprev = Pk;

            // update UI
            OnPropertyChanged(nameof(XSpeed));
            OnPropertyChanged(nameof(YSpeed));

            // draw predicted line
            AddToLine(inkCanvasEstimated1, new Point(Xkp[0], Xkp[1]), Colors.Blue);
            // draw corrected line
            AddToLine(inkCanvasEstimated2, new Point(Xprev[0], Xprev[1]), Colors.Green);
            // draw mesurment points
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
                    Sigma *= 1.5;
                    break;
                case Key.Subtract:
                case Key.OemMinus:
                    Sigma /= 1.5;
                    break;
                case Key.Space:
                    pauseWhileNotMoving = !pauseWhileNotMoving;
                    break;
            }
        }
    }
}
