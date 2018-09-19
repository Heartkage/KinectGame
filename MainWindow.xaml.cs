//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Threading;
    using System.Runtime.InteropServices;
    using System.Diagnostics;
   


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        //............................................................................................................
        [DllImport("user32.dll", EntryPoint = "keybd_event", SetLastError = true)]
        public static extern void keybd_event(byte bVk, byte bScan, int dwflags, IntPtr dwExtraInfo);

        [DllImport("user32.dll")]
        public static extern int MapVirtualKey(uint Ucode, uint uMapType);

        [System.Runtime.InteropServices.DllImport("user32")]
        private static extern int mouse_event(int dwFlags, int dx, int dy, int cButtons, int dwExtraInfo);

        [DllImport("user32.dll", CharSet = CharSet.Unicode)]
        static extern short VkKeyScan(char ch);

        /*
        [DllImport("user32.dll", SetLastError = true)]
        internal static extern uint SendInput(uint nInput, ref INPUT pInput, int cbSize);

        [DllImport("user32.dll")]
        private static extern IntPtr GetMessageExtraInfo();

        [StructLayout(LayoutKind.Explicit)]
        internal struct INPUT
        {
            [FieldOffset(0)]
            internal int type;//0:mouse event;1:keyboard event;2:hardware event
            [FieldOffset(8)]
            internal MOUSEINPUT mi;
            [FieldOffset(8)]
            public KEYBDINPUT ki;
            [FieldOffset(8)]
            internal HARDWAREINPUT hi;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        internal struct HARDWAREINPUT
        {
            internal uint uMsg;
            internal short wParamL;
            internal short wParamH;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        internal struct KEYBDINPUT
        {
            internal ushort wVk;
            internal ushort wScan;
            internal uint dwFlags;
            internal uint time;
            internal IntPtr dwExtraInfo;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        internal struct MOUSEINPUT
        {
            internal int dx;
            internal int dy;
            internal int mouseData;
            internal int dwFlags;
            internal int time;
            internal IntPtr dwExtraInfo;
        }*/

        [DllImport("user32.dll", EntryPoint = "FindWindow")]
        private static extern int FindWindow(string sClass, string sWindow);
        [DllImport("user32.dll")]
        public static extern void SwitchToThisWindow(IntPtr hWnd, bool turnon);
        [DllImport("user32.dll", EntryPoint = "MapVirtualKeyA")]
        private extern static int MapVirtualKey(int wCode, int wMapType);
        [DllImport("USER32.DLL")]
        public static extern bool SetForegroundWindow(IntPtr hWnd);

        const byte VK_F = 0x46;
        const byte VK_W = 0x57;
        const byte VK_A = 0x41;
        const byte VK_S = 0x53;
        const byte VK_D = 0x44;
        const byte VK_shift = 0xA0;
        const byte VK_space = 0x20;

        static int shoot_count = 0;
        int wheel_count = 0;
        static int D_counter = 0;
        static int A_counter = 0;
        static int W_counter = 0;

        double spine_coord = 0;
        double elbow_coord = 0;
        int shift_on = 0;

        //keybd_event
        private const int INPUT_KEYBOARD = 1;
        private const int KEYEVENTF_DOWN = 0x0000;
        private const int KEYEVENTF_EXTENDEDKEY = 0x0000;
        private const int KEYEVENTF_KEYUP = 0x0002;
        private const int scancode = 0x0008;

        //mouse_event
        const int MOUSEEVENTF_MOVE = 0x0001; //移动鼠标
        const int MOUSEEVENTF_LEFTDOWN = 0x0002; //模拟鼠标左键按下
        const int MOUSEEVENTF_LEFTUP = 0x0004; //模拟鼠标左键抬起
        const int MOUSEEVENTF_RIGHTDOWN = 0x0008; //模拟鼠标右键按下
        const int MOUSEEVENTF_RIGHTUP = 0x0010; //模拟鼠标右键抬起
        const int MOUSEEVENTF_WHEEL = 0x0800; //滾輪
        const int MOUSEEVENTF_ABSOLUTE = 0x8000; //标示是否采用绝对坐标

        static double currentHand_x, currentHand_y;
        static int mouse_state = 0;
        static bool move_check = false;
        static int aim_count = 0;
        static bool mouseRight = false;
        static bool mouseLeft = false;

        Process[] processes;
        IntPtr WindowHandle;
        //................................................................................................................
        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();

            //System.Diagnostics.Process info = new System.Diagnostics.Process();
            //info.StartInfo.FileName = "C:\\Users\\p124l\\source\\repos\\WpfApp1\\hello.txt";
            //info.Start();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            processes = Process.GetProcessesByName("Notepad");
            if (processes.Length == 0)
                throw new Exception("Could not find the process; is the running?");

            WindowHandle = processes[0].MainWindowHandle;
            SwitchToThisWindow(processes[0].MainWindowHandle, false);
            SetForegroundWindow(WindowHandle);
            Thread.Sleep(300);

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                
                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];      

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);     
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                        
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }

            if (skeletons.Length != 0)
            {
              
                //射擊
                Thread shoot = new Thread(() => shooting(skeletons));
                shoot.Start();

                for (int i = 0; i < skeletons.Length; i++)
                {
                    if ((skeletons[i].TrackingState == SkeletonTrackingState.Tracked) || (skeletons[i].TrackingState == SkeletonTrackingState.PositionOnly))
                    {
                        JointCollection joint = skeletons[i].Joints;


                        double Hip_distance = Math.Abs(joint[JointType.HipRight].Position.X - joint[JointType.HipLeft].Position.X);
                        //膝蓋間(X軸)的距離
                        double kneeDistance = Math.Abs(joint[JointType.KneeRight].Position.X - joint[JointType.KneeLeft].Position.X);
                        //屁股到膝蓋間(x,z軸)的距離
                        double kneeHipDistance = Math.Sqrt(Math.Pow(joint[JointType.HipLeft].Position.Z - joint[JointType.KneeLeft].Position.Z, 2) + Math.Pow(joint[JointType.HipLeft].Position.X - joint[JointType.KneeLeft].Position.X, 2));

                        //Console.WriteLine("Distance: " + kneeHipDistance);
                        //Console.WriteLine("Knee: " + kneeDistance);


                        if(mouse_state == 0)
                        {
                            // 模擬開車w(前)
                            if (joint[JointType.KneeRight].Position.Z < (joint[JointType.HipRight].Position.Z - 0.04))
                            {
                                if (shift_on == 1)
                                {
                                    keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    shift_on = 0;
                                }

                                W_counter++;
                                if (W_counter < 12)
                                    keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);

                                Console.WriteLine("Left amount: " + (joint[JointType.AnkleRight].Position.X- joint[JointType.KneeRight].Position.X));
                                Console.WriteLine("Right amount: " + (joint[JointType.KneeRight].Position.X- joint[JointType.HipRight].Position.X));

                                // 模擬A(向左)+(大彎)                            
                                if (joint[JointType.AnkleRight].Position.X > (joint[JointType.KneeRight].Position.X + 0.09))
                                {
                                    Console.WriteLine("Left Big Turn");
                                    keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                }
                                // 模擬D(向右)+(大彎)                           
                                else if (joint[JointType.KneeRight].Position.X > (joint[JointType.HipRight].Position.X + 0.1))
                                {
                                    Console.WriteLine("Right Big Turn");
                                    keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                }
                                // 模擬A(向左)
                                else if (joint[JointType.AnkleRight].Position.X > (joint[JointType.KneeRight].Position.X + 0.02))
                                {
                                    Console.WriteLine("Left Small Turn");
                                    keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    if (A_counter < 1)
                                        keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                    else
                                        keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_KEYUP, (IntPtr)0);

                                    A_counter++;
                                }
                                // 模擬D(向右)
                                else if (joint[JointType.KneeRight].Position.X > (joint[JointType.HipRight].Position.X + 0.04))
                                {
                                    Console.WriteLine("Right Small Turn");
                                    keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    if (D_counter < 1)
                                        keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                    else
                                        keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_KEYUP, (IntPtr)0);

                                    D_counter++;
                                }
                                else
                                {
                                    A_counter = 0;
                                    D_counter = 0;
                                    keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                }
                            }
                            // 模擬W(向前+跑)
                            else if (kneeHipDistance > 0.147)
                            {
                                keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                                shift_on = 1;

                                // 模擬滑鼠(向左滑)
                                if (joint[JointType.HipLeft].Position.X > (joint[JointType.KneeLeft].Position.X + 0.05))
                                {
                                    int moveAmount = (int)((double)(kneeDistance - 0.227) * 1000);
                                    //Console.WriteLine("moveLeft: " + moveAmount);
                                    mouse_event(MOUSEEVENTF_MOVE, (-1) * moveAmount, 0, 0, 0);
                                }
                                // 模擬滑鼠(向右滑)
                                else if (joint[JointType.AnkleLeft].Position.X < (joint[JointType.KneeLeft].Position.X - 0.018))
                                {
                                    int moveAmount = (int)((double)(0.217 - kneeDistance) * 1100);
                                    //Console.WriteLine("moveRight: " + moveAmount);
                                    mouse_event(MOUSEEVENTF_MOVE, moveAmount, 0, 0, 0);
                                }

                            }
                            else if (kneeHipDistance > 0.06)
                            {
                                if (shift_on == 1)
                                {
                                    keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    shift_on = 0;
                                }

                                keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);

                                // 模擬滑鼠(向左滑)
                                if (joint[JointType.HipLeft].Position.X > (joint[JointType.KneeLeft].Position.X + 0.05))
                                {
                                    int moveAmount = (int)((double)(kneeDistance - 0.23) * 1000);
                                    //Console.WriteLine("moveLeft: " + moveAmount);
                                    mouse_event(MOUSEEVENTF_MOVE, (-1) * moveAmount, 0, 0, 0);
                                }
                                // 模擬滑鼠(向右滑)
                                else if (joint[JointType.AnkleLeft].Position.X < (joint[JointType.KneeLeft].Position.X - 0.018))
                                {
                                    int moveAmount = (int)((double)(0.212 - kneeDistance) * 1100);
                                    //Console.WriteLine("moveRight: " + moveAmount);
                                    mouse_event(MOUSEEVENTF_MOVE, moveAmount, 0, 0, 0);
                                }

                            }
                            else
                            {
                                if (shift_on == 1)
                                {
                                    keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                    shift_on = 0;
                                }
                                keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_KEYUP, (IntPtr)0);

                                W_counter = 0;
                                A_counter = 0;
                                D_counter = 0;
                            }
                        }
                        
                        directionCounter();


                        { 
                        /*
                        // 模擬W(向前)
                        if (joint[JointType.KneeLeft].Position.Z < (joint[JointType.HipLeft].Position.Z - 0.03))
                        {
                            keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                        }
                        // 模擬w(前+跑)
                        else if (joint[JointType.KneeRight].Position.Z < (joint[JointType.HipRight].Position.Z - 0.03))
                        {
                            keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                            keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                            shift_on = 1;
                        }
                        else
                        {                                 
                            if (shift_on == 1)
                            {
                                keybd_event(VK_shift, (byte)MapVirtualKey(VK_shift, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                                shift_on = 0;
                            }
                            keybd_event(VK_W, (byte)MapVirtualKey(VK_W, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                        }*/
                        /*
                        // 模擬D(向右)
                         if ((Hip_distance < 0.144) && (joint[JointType.Head].Position.X-0.02 > joint[JointType.ShoulderCenter].Position.X) && (D_counter <= 3))
                         {
                             keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);                          
                          }
                         else
                        {
                           keybd_event(VK_D, (byte)MapVirtualKey(VK_D, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                           D_counter = 0;
                        }
                        // 模擬A(向左)
                        if ((Hip_distance < 0.144)  && (joint[JointType.Head].Position.X+0.02 < joint[JointType.ShoulderCenter].Position.X) && (A_counter <= 3))
                        {
                           keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);                      
                        }
                        else
                        {
                           A_counter = 0;                           
                           keybd_event(VK_A, (byte)MapVirtualKey(VK_A, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                        }*/
                        }


                        // 模擬s(後退)
                        if (joint[JointType.HandRight].Position.Z > joint[JointType.HipCenter].Position.Z + 0.03 && joint[JointType.HandLeft].Position.Z > joint[JointType.HipCenter].Position.Z + 0.03)
                            keybd_event(VK_S, (byte)MapVirtualKey(VK_S, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                        else
                            keybd_event(VK_S, (byte)MapVirtualKey(VK_S, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                    

                        // 模擬空白建(跳躍)
                        if ((joint[JointType.Spine].Position.Y > spine_coord+0.025) && (mouse_state == 0))
                        {
                            keybd_event(VK_space, (byte)MapVirtualKey(VK_space, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                        }
                        else
                            keybd_event(VK_space, (byte)MapVirtualKey(VK_space, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                            
                                                         

                        // 模擬F(左手開門)
                        if ((joint[JointType.HandRight].Position.Y < joint[JointType.ElbowRight].Position.Y) && (joint[JointType.HandLeft].Position.Y > joint[JointType.HipCenter].Position.Y) && (joint[JointType.HandLeft].Position.X < joint[JointType.ElbowLeft].Position.X))
                        {
                            keybd_event(VK_F, (byte)MapVirtualKey(VK_F, 0), KEYEVENTF_EXTENDEDKEY, (IntPtr)0);
                        }
                        else
                            keybd_event(VK_F, (byte)MapVirtualKey(VK_F, 0), KEYEVENTF_KEYUP, (IntPtr)0);
                        // 模擬滑鼠滾輪(選武器)
                        if ((joint[JointType.HandLeft].Position.Y < joint[JointType.ElbowLeft].Position.Y) && (joint[JointType.HandRight].Position.Y > joint[JointType.Spine].Position.Y) && (joint[JointType.HandRight].Position.X > joint[JointType.ElbowRight].Position.X))
                        {
                            wheel_count++;
                            if (wheel_count <= 1)
                            {
                                mouse_event(MOUSEEVENTF_WHEEL, 0, 0, 120, 0);
                            }
                            else if (wheel_count > 20)
                                wheel_count = 0;
                        }
                        else
                            wheel_count = 0;


                        //紀錄目前身體中心的Y軸
                        spine_coord = joint[JointType.Spine].Position.Y;

                        

                    }

                }
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);
            
            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);
 
            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;                    
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;                    
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                drawPen = this.trackedBonePen;
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;                  
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

        private static void shooting(Skeleton[] skeletons)
        {
            for (int i = 0; i < skeletons.Length; i++)
            {
                if ((skeletons[i].TrackingState == SkeletonTrackingState.Tracked) || (skeletons[i].TrackingState == SkeletonTrackingState.PositionOnly))
                {
                    JointCollection joint = skeletons[i].Joints;

                                   
                    // 模擬舉槍滑鼠右鍵(瞄準)
                    if ((joint[JointType.WristLeft].Position.Y > joint[JointType.HipCenter].Position.Y) && (joint[JointType.WristRight].Position.Y > joint[JointType.HipCenter].Position.Y))
                    {                                     
                        mouse_event(MOUSEEVENTF_RIGHTDOWN, 0, 0, 0, 0);
                        mouseRight = true;
                        if (mouse_state == 0)
                        {
                            currentHand_x = joint[JointType.ShoulderCenter].Position.X;
                            currentHand_y = joint[JointType.ShoulderLeft].Position.Y;

                            mouse_state = 1;
                        }


                        //Console.WriteLine("Hand X_Position: " + joint[JointType.WristLeft].Position.X);
                        //Console.WriteLine("Hand Y_Position: " + joint[JointType.WristLeft].Position.Y);


                        int x_movement = 0;
                        int y_movement = 0;

                        if ((Math.Abs(currentHand_x - joint[JointType.WristLeft].Position.X) > 0.02) || (Math.Abs(currentHand_y - joint[JointType.WristLeft].Position.Y) > 0.02))
                        {
                            //Console.WriteLine("ok");                         
                            x_movement = (int)((joint[JointType.WristLeft].Position.X - currentHand_x) * 100);
                            y_movement = (int)((joint[JointType.WristLeft].Position.Y - currentHand_y) * 100)+5;
                            move_check = true;
                        }

                        if (x_movement > 0)
                            x_movement += 4;

                        //Console.WriteLine("X_movement: " + x_movement);
                        //Console.WriteLine("Y_movement: " + y_movement*(-1));

                        if (move_check)
                        {
                            mouse_event(MOUSEEVENTF_MOVE, x_movement , y_movement*(-1), 0, 0);
                            move_check = false;
                        }
                       
                            


                        // 模擬射擊左鍵(射擊)
                        if ((joint[JointType.ElbowRight].Position.Z > joint[JointType.HipCenter].Position.Z+0.14) && (shoot_count <= 1))
                         {
                            mouseLeft = true;
                            mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                         }
                         else
                         {
                            if(mouseLeft)
                            {
                                mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                                mouseLeft = false;
                            }
                                
                         }

                        if (shoot_count > 3)
                            shoot_count = 0;                        
                         if (aim_count > 10)
                             aim_count = 0;

                         aim_count++;
                         shoot_count++;
                     }
                    else
                    {
                        mouse_state = 0;
                        shoot_count = 0;
                        aim_count = 0;
                        move_check = false;

                        if (mouseRight)
                        {
                            mouse_event(MOUSEEVENTF_RIGHTUP, 0, 0, 0, 0);
                            mouseRight = false;
                        }


                        if (mouseLeft)
                        {
                            mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
                            mouseLeft = false;
                        }
                            
                        
                    }
                }
            }
        }

        private static void directionCounter()
        {
            if (A_counter > 8)
                A_counter = 0;

            if (D_counter > 8)
                D_counter = 0;

            if (W_counter > 12)
                W_counter = 0;
        }
    }

    
}

