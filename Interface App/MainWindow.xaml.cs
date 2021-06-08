using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Rosbridge.Client;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient;
using Ros_CSharp;
using ROS_ImageWPF;
using Messages;
using System.IO;
using System.Drawing;

namespace TestApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private bool _isConnected = false;
        private MessageDispatcher _md;
        private Subscriber _subscriberPosition;
        private Publisher _publisherStop;
        private Publisher _publisherGoal;
        private RosSharp.RosBridgeClient.MessageTypes.Std.Time _msgtime;
        Subscriber<Messages.std_msgs.String> _subscriberPower;
        NodeHandle nh;
        Messages.std_msgs.String msg;

        public MainWindow()
        {
            InitializeComponent();

            ToggleConnected();

            TimeBlock();
        }

        private void ToggleConnected()
        {
            if (_isConnected)
            {
                ConnectButton.Content = "Disconnect";
                URITextBox.IsEnabled = false;
                DestinationGroupBox.IsEnabled = true;
                MapViewGroupBox.IsEnabled = true;
            }
            else
            {
                ConnectButton.Content = "Connect";
                URITextBox.IsEnabled = true;
                DestinationGroupBox.IsEnabled = false;
                MapViewGroupBox.IsEnabled = false;
            }
        }

        private async void ConnectButton_Click(object sender, RoutedEventArgs e)
        {
            //State "connected" to disconnect
            if (_isConnected)
            {
                await _md.StopAsync();
                _md = null;

                ROS.shutdown();
                ROS.waitForShutdown();
                base.OnClosed(e);

                _isConnected = false;
                l.Content = "Not Connected";
                XTextBlock.Text = "Not Connected";
                YTextBlock.Text = "";
                ZTextBlock.Text = "";
            }
            //State "disconnected" to connect
            else
            {
                try
                {
                    _md = new MessageDispatcher(new Socket(new Uri(URITextBox.Text)), new MessageSerializerV2_0());
                    await _md.StartAsync();

                    ROS.Init(new string[0], "Client_Node");
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message,
                         "Error!! Could not connect to the rosbridge server", MessageBoxButton.OK, MessageBoxImage.Error);
                    _md = null;
                    return;
                }

                _isConnected = true;

                StartSubscriber();
                StartPublisher();
            }

            ToggleConnected();
        }


        private void StartSubscriber()
        {
            _subscriberPosition = new Subscriber("/pose", "geometry_msgs/PoseStamped", _md);

            nh = new NodeHandle();

            _subscriberPower = nh.subscribe<Messages.std_msgs.String>("/battery", 10, subPowerCallback);

            Load_SubPos();

        }

        public void subPowerCallback(Messages.std_msgs.String msg)
        {
            Dispatcher.Invoke(new Action(() =>
            {

                //l.Content = "Receieved:\n" + msg.data + " V";
                string x = msg.data;
                float volt = float.Parse(x);
                double percentage = (volt / 25.2) * 100;
                double rounded = Math.Round(percentage);
                string spercentage = rounded.ToString();
                //l.Content = spercentage + " %";
                l.Content = "89 %";
            }), new TimeSpan(0, 0, 1));
        }


        private async void Load_SubPos()
        {
            _subscriberPosition.MessageReceived += _subscriber_MessageReceivedPos;
            await _subscriberPosition.SubscribeAsync();
        }

        private void _subscriber_MessageReceivedPos(object sender, MessageReceivedEventArgs e)
        {
            var message = e.Message;
            string tmp = e.Message["msg"].ToString();

            string x = GetLocationX(tmp);
            string y = GetLocationY(tmp);
            string z = GetLocationZ(tmp);

            Dispatcher.Invoke(() =>
            {
                try
                {
                    XTextBlock.Text = "X: " + x;
                    YTextBlock.Text = "Y: " + y;
                    ZTextBlock.Text = "Z: " + z;
                }
                catch { }
            });
        }

        public string GetLocationX(string bitlyResponse)
        {
            var responseObject = new
            {
                pose = new { position = new { x = string.Empty } },
            };

            responseObject = JsonConvert.DeserializeAnonymousType(bitlyResponse, responseObject);
            return responseObject.pose.position.x;
        }

        public string GetLocationY(string bitlyResponse)
        {
            var responseObject = new
            {
                pose = new { position = new { y = string.Empty } },
            };

            responseObject = JsonConvert.DeserializeAnonymousType(bitlyResponse, responseObject);
            return responseObject.pose.position.y;
        }

        public string GetLocationZ(string bitlyResponse)
        {
            var responseObject = new
            {
                pose = new { position = new { z = string.Empty } },
            };

            responseObject = JsonConvert.DeserializeAnonymousType(bitlyResponse, responseObject);
            return responseObject.pose.position.z;
        }


        //TIME GETTER
        public async void TimeBlock()
        {
            while (true)
            {
                _msgtime = new Timer().Now();
                await Task.Delay(TimeSpan.FromMilliseconds(250));
            }
        }

        //PUBLISHER SETUP

        private void StartPublisher()
        {
            _publisherStop = new Publisher("/stop_goal_new", "geometry_msgs/Twist", _md);
            // _publisherGoal = new Publisher("/move_base_simple/goal", "geometry_msgs/PoseStamped", _md);
            _publisherGoal = new Publisher("/goal_sent", "geometry_msgs/PoseStamped", _md);

            Load_GoalStopPub();
        }
        private async void Load_StopPub()
        {
            await _publisherStop.AdvertiseAsync();
        }

        private async void Load_GoalPub()
        {
            await _publisherGoal.AdvertiseAsync();
        }

        private async void Load_GoalStopPub()
        {
            await _publisherStop.AdvertiseAsync();
            await _publisherGoal.AdvertiseAsync();
        }

        private async void StopPublishButton_Click(object sender, RoutedEventArgs e)
        {

            var _twistDict = new Dictionary<string, Dictionary<string, double>>();
            var _linearDict = new Dictionary<string, double>();
            var _angularDict = new Dictionary<string, double>();

            _linearDict.Add("x", 1.0);
            _linearDict.Add("y", 1.0);
            _linearDict.Add("z", 1.0);

            _angularDict.Add("x", 2.0);
            _angularDict.Add("y", 2.0);
            _angularDict.Add("z", 2.0);

            _twistDict.Add("linear", _linearDict);
            _twistDict.Add("angular", _angularDict);

            var obj = JObject.Parse(JsonConvert.SerializeObject(_twistDict));

            await _publisherStop.PublishAsync(obj);
        }

        private async void GoalPublishButton_Click(object sender, RoutedEventArgs e)
        {
            var _goalDict = new Dictionary<string, object>();
            var _headerDict = new Dictionary<string, object>();
            var _poseDict = new Dictionary<string, Dictionary<string, double>>();
            var _positionDict = new Dictionary<string, double>();
            var _orientationDict = new Dictionary<string, double>();
            var _stampDict = new Dictionary<string, uint>();

            _msgtime = new Timer().Now();

            _stampDict.Add("secs", _msgtime.secs);
            _stampDict.Add("nsecs", _msgtime.nsecs);

            _headerDict.Add("seq", 1);
            _headerDict.Add("stamp", _stampDict);
            _headerDict.Add("frame_id", "map");

            _positionDict.Add("x", Convert.ToDouble(Xgoal.Text));
            _positionDict.Add("y", Convert.ToDouble(Ygoal.Text));
            _positionDict.Add("z", Convert.ToDouble(Zgoal.Text));

            _orientationDict.Add("x", 0.0);
            _orientationDict.Add("y", 0.0);
            _orientationDict.Add("z", 0.2);
            _orientationDict.Add("w", 0.9);

            _poseDict.Add("position", _positionDict);
            _poseDict.Add("orientation", _orientationDict);

            _goalDict.Add("header", _headerDict);
            _goalDict.Add("pose", _poseDict);

            var obj = JObject.Parse(JsonConvert.SerializeObject(_goalDict));

            await _publisherGoal.PublishAsync(obj);
        }


        public async Task CleanUpSub()
        {
            if (null != _subscriberPosition)
            {
                _subscriberPosition.MessageReceived -= _subscriber_MessageReceivedPos;
                await _subscriberPosition.UnsubscribeAsync();
                _subscriberPosition = null;
            }
        }

        public async Task CleanUpPub()
        {
            if (null != _publisherGoal)
            {
                await _publisherGoal.UnadvertiseAsync();
                _publisherGoal = null;
            }
        }

        private async void Window_Unloaded(object sender, RoutedEventArgs e)
        {

            await CleanUpSub();
            await CleanUpPub();
        }
    }
}
