using OpenCvSharp;
using OpenCvSharp.UserInterface;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Data.SqlClient;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.Remoting.Messaging;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace stm_control
{
    public partial class Form1 : Form
    {
        private readonly DbControl dbControl = new DbControl();
        private readonly FileControl fileControl = new FileControl();
        private CvCapture capture;
        private IplImage sourceImage;
        private IplImage resizeImage;

        private int tickCount = 0;

        private string motorDirection = "N";
        private int motorSpeedLevel = 0;

        public const int MOTOR_SPEED_MAX = 100;
        public const int MOTOR_SPEED_LEVEL_MAX = 10;

        public const int LED_VALUE_MAX = 100;

        DirectoryInfo logDirectory = null;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            comboBoxStm.DataSource = SerialPort.GetPortNames();

            capture = CvCapture.FromCamera(CaptureDevice.DShow, 0);
            sourceImage = Cv.CreateImage(Cv.Size(capture.FrameWidth, capture.FrameHeight), BitDepth.U8, 3);
            resizeImage = Cv.CreateImage(Cv.Size(pictureBoxIplCamera.Width, pictureBoxIplCamera.Height), BitDepth.U8, 3);
            timer.Start();

            PrintMotorSpeed();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer.Stop();
            if (serialPortStm.IsOpen) serialPortStm.Close();
            Cv.ReleaseImage(resizeImage);
            Cv.ReleaseImage(sourceImage);
            capture.Dispose();
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            tickCount++;
            if (tickCount >= 10)
            {
                labelCurrentTime.Text = DateTime.Now.ToString();
                tickCount = 1;
            }

            sourceImage = capture.QueryFrame();
            Cv.Resize(sourceImage, resizeImage);
            pictureBoxIplCamera.ImageIpl = resizeImage;
        }

        private void ButtonStmConnect_Click(object sender, EventArgs e)
        {
            if (!serialPortStm.IsOpen)
            {
                serialPortStm.PortName = comboBoxStm.SelectedItem.ToString();
                serialPortStm.BaudRate = 9600;
                serialPortStm.DataBits = 8;
                serialPortStm.StopBits = StopBits.One;
                serialPortStm.Parity = Parity.None;
                serialPortStm.Open();
                if (serialPortStm.IsOpen)
                {
                    labelStmConnectionStatus.Text = "연결됨";
                    labelStmConnectionStatus.ForeColor = Color.Green;
                }
            }
            else
            {
                MessageBox.Show("이미 연결되어 있습니다.");
            }
        }

        private void ButtonStmDisconnect_Click(object sender, EventArgs e)
        {
            if (serialPortStm.IsOpen)
            {
                serialPortStm.Close();
                if (!serialPortStm.IsOpen)
                {
                    labelStmConnectionStatus.Text = "연결되지 않음";
                    labelStmConnectionStatus.ForeColor = Color.Red;
                }
            }
            else
            {
                MessageBox.Show("이미 연결이 끊겨 있습니다.");
            }
        }

        private void SerialPortStm_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            string msg = serialPortStm.ReadLine();
            msg = msg.Replace("\r", "").Replace("\n", "");
            string[] datas = msg.Split(':');

            switch (datas[0])
            {
                case "T":
                    string[] temperatureDatas = datas[1].Split(',');
                    try
                    {
                        int adc = Convert.ToInt32(temperatureDatas[0]);
                        double voltage = Convert.ToDouble(temperatureDatas[1]);
                        int temperature = Convert.ToInt32(temperatureDatas[2]);

                        this.Invoke((MethodInvoker)delegate
                        {
                            textBoxTemperature.Text = temperature + "\u2103";
                            listBoxAdc.Items.Add(string.Format("{0}   ADC:{1}   {2}[V]   {3}[{4}]", DateTime.Now.ToString("tt h:mm:ss"), adc, voltage, temperature, "\u2103"));
                            listBoxAdc.SelectedIndex = listBoxAdc.Items.Count - 1;
                            listBoxAdc.SelectedIndex = -1;

                            PrintChart(adc);
                        });

                        dbControl.InsertTemperatureIntoDb(temperature);
                    }
                    catch (FormatException fe)
                    {
                        Debug.WriteLine("Temperature data receive error: " + fe.Message);
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine("Temperature data receive error: " + ex.Message);
                    }
                    break;
                default:
                    break;
            }
        }

        private void PrintChart(int adc)
        {
            chartAdc.Series[0].Points.AddXY(chartAdc.ChartAreas[0].AxisX.Maximum + 1, adc);

            if (chartAdc.Series[0].Points.Count > 10)
            {
                chartAdc.Series[0].Points.RemoveAt(0);
            }    
            chartAdc.ChartAreas[0].AxisX.Minimum = chartAdc.Series[0].Points.First().XValue;
            chartAdc.ChartAreas[0].AxisX.Maximum = chartAdc.Series[0].Points.Last().XValue;

            double chartYMax = 0;

            for (int i = 0; i < chartAdc.Series[0].Points.Count; i++)
            {
                if (chartYMax < chartAdc.Series[0].Points[i].YValues[0])
                {
                    chartYMax = chartAdc.Series[0].Points[i].YValues[0];
                }
            }
            chartAdc.ChartAreas[0].AxisY.Maximum = chartYMax + 100;
        }

        private void ButtonMotorRight_Click(object sender, EventArgs e)
        {
            SetMotorDirection("R");
        }

        private void ButtonMotorLeft_Click(object sender, EventArgs e)
        {
            SetMotorDirection("L");
        }

        private void ButtonMotorStop_Click(object sender, EventArgs e)
        {
            SetMotorDirection("N");
        }

        private void ButtonMotorSpeedUp_Click(object sender, EventArgs e)
        {
            ChangeMotorSpeedLevel(true);
        }

        private void ButtonMotorSpeedDown_Click(object sender, EventArgs e)
        {
            ChangeMotorSpeedLevel(false);
        }

        private void SetMotorDirection(string inputDirection)
        {
            if (!serialPortStm.IsOpen)
            {
                MessageBox.Show(Messages.PORT_NOT_OPEN);
            }
            else
            {
                if (motorDirection == OppositeDirectionOf(inputDirection))
                {
                    MessageBox.Show(Messages.PREVENT_RAPID_TURN);
                }
                else
                {
                    motorDirection = inputDirection;
                    switch (inputDirection)
                    {
                        case "R":
                            buttonMotorRight.BackColor = Color.Yellow;
                            buttonMotorLeft.BackColor = SystemColors.ControlLight;
                            break;
                        case "L":
                            buttonMotorRight.BackColor = SystemColors.ControlLight;
                            buttonMotorLeft.BackColor = Color.Yellow;
                            break;
                        case "N":
                            buttonMotorRight.BackColor = SystemColors.ControlLight;
                            buttonMotorLeft.BackColor = SystemColors.ControlLight;
                            motorSpeedLevel = 0;
                            SendMotorData();
                            PrintMotorSpeed();
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        private string OppositeDirectionOf(string direction)
        {
            switch (direction)
            {
                case "R": return "L";
                case "L": return "R";
                default: return null;
            }
        }

        private void ChangeMotorSpeedLevel(bool isIncrease)
        {
            if (!serialPortStm.IsOpen)
            {
                MessageBox.Show(Messages.PORT_NOT_OPEN);
            }
            else
            {
                if (motorDirection == "N")
                {
                    MessageBox.Show(Messages.DIRECTION_NOT_CHOSEN);
                }
                else
                {
                    if (isIncrease)
                    {
                        if (motorSpeedLevel < MOTOR_SPEED_LEVEL_MAX)
                            motorSpeedLevel++;
                    }
                    else
                    {
                        if (motorSpeedLevel > 0)
                            motorSpeedLevel--;
                    }
                    SendMotorData();
                    PrintMotorSpeed();
                }
            }
        }

        private void SendMotorData()
        {
            try
            {
                string motorMsg = string.Format("M:{0}{1:D3}", motorDirection, MotorSpeed());
                serialPortStm.WriteLine(motorMsg);
                Debug.WriteLine(motorMsg);
            }
            catch (Exception ex)
            {
                Debug.WriteLine("Motor data send error: " + ex.Message);
            }
        }

        private void PrintMotorSpeed()
        {
            textBoxMotorSpeed.Text = MotorSpeed().ToString();
        }

        private int MotorSpeed()
        {
            return (motorSpeedLevel * MOTOR_SPEED_MAX / MOTOR_SPEED_LEVEL_MAX);
        }

        private void TrackBarLedRed_Scroll(object sender, EventArgs e)
        {
            if (!serialPortStm.IsOpen)
            {
                MessageBox.Show(Messages.PORT_NOT_OPEN);
                trackBarLedRed.Value = trackBarLedRed.Minimum;
            }
            else
            {
                SendLedData("R");
            }
        }

        private void TrackBarLedGreen_Scroll(object sender, EventArgs e)
        {
            if (!serialPortStm.IsOpen)
            {
                MessageBox.Show(Messages.PORT_NOT_OPEN);
                trackBarLedGreen.Value = trackBarLedGreen.Minimum;
            }
            else
            {
                SendLedData("G");
            }
        }

        private void TrackBarLedBlue_Scroll(object sender, EventArgs e)
        {
            if (!serialPortStm.IsOpen)
            {
                MessageBox.Show(Messages.PORT_NOT_OPEN);
                trackBarLedBlue.Value = trackBarLedBlue.Minimum;
            }
            else
            {
                SendLedData("B");
            }
        }

        private void SendLedData(string color)
        {
            try
            {
                string ledMsg = string.Format("L:{0}{1:D3}", color, LedValue(color));
                serialPortStm.WriteLine(ledMsg);
                Debug.WriteLine(ledMsg);
            }
            catch (Exception ex)
            {
                Debug.WriteLine("LED data send error: " + ex.Message);
            }
        }

        private int LedValue(string colorOfTrackBar)
        {
            int trackBarValue = 0;
            int trackBarMax = 1;
            switch (colorOfTrackBar)
            {
                case "R":
                    trackBarValue = trackBarLedRed.Value;
                    trackBarMax = trackBarLedRed.Maximum;
                    break;
                case "G":
                    trackBarValue = trackBarLedGreen.Value;
                    trackBarMax = trackBarLedGreen.Maximum;
                    break;
                case "B":
                    trackBarValue = trackBarLedBlue.Value;
                    trackBarMax = trackBarLedBlue.Maximum;
                    break;
                default:
                    break;
            }
            return LED_VALUE_MAX - (trackBarValue * LED_VALUE_MAX / trackBarMax);
        }

        private void ButtonReadDb_Click(object sender, EventArgs e)
        {
            listBoxReadDb.Items.Clear();
            listBoxReadDb.Items.AddRange(dbControl.ReadTemperatureFromDb().ToArray());
        }

        private void ButtonClearReadDb_Click(object sender, EventArgs e)
        {
            listBoxReadDb.Items.Clear();
        }

        private void ButtonDeleteDb_Click(object sender, EventArgs e)
        {
            try
            {
                int deleteId = Convert.ToInt32(textBoxDeleteId.Text);
                int? rowsAffected = dbControl.DeleteTemperatureFromDb(deleteId);
                if (rowsAffected != null)
                {
                    string msg = Messages.DELETE_FROM_DB_OK;
                    if (logDirectory != null)
                    {
                        FileInfo fileInfo = fileControl.CreateDeleteLogFile(logDirectory, (int)rowsAffected);
                        labelLogFile.Text = fileInfo.FullName;
                        msg += "\r\n" + Messages.LOG_FILE_CREATED;
                    }
                    MessageBox.Show(msg);
                }
            }
            catch (FormatException fe)
            {
                MessageBox.Show(Messages.FORMAT_EXCEPTION);
                Debug.WriteLine("Delete db error: " + fe.Message);
            }
            catch (Exception ex)
            {
                Debug.WriteLine("Delete db error: " + ex.Message);
            }
        }

        private void ButtonCreateLogDirectory_Click(object sender, EventArgs e)
        {
            if (textBoxLogDirectoryName.Text != "")
            {
                string logDirectoryName = "\\" + textBoxLogDirectoryName.Text;
                logDirectory = fileControl.CreateDirectory(logDirectoryName);
                labelLogDirectory.Text = logDirectory.FullName;
            }
        }
    }
}
