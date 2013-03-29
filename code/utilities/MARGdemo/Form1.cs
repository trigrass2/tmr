#define TMR_BOARD
#define USE_SERIAL_3
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
//using xioInterface;
using System.IO.Ports;
//using Microsoft.Win32;
using System.Management;

// ********************************************
// Description:
//
// - Uses orientation filter algorithm to orientate on-screen red, green and blue axes.
// - Sensors calibration values must be specified within the source code and will vary between apparently identical sensors.
// - Click and hold the form to force high gain values for initial convergence.  Release to revert to normal gains and 'zero position' of the on-screen image.
// - IMU must be stationary on start up in order to correctly sample gyroscope biases
// - Connections to Sparkfun IMU 6DOF Razor:
//   - ACH0 = accelerometer x-axis
//   - ACH1 = accelerometer y-axis
//   - ACH2 = accelerometer z-axis
//   - ACH3 gyroscope y-axis
//   - ACH4 gyroscope x-axis
//   - ACH5 gyroscope z-axis
// - Sensors should be uniquely calibrated.  Gains and Biases may vary between apparently identical sensors and will vary with temperature.
// - Requires alternative x-io Board firmware for interface to the HMC5843 via th x-io Board COMMS header
//
// ********************************************

namespace MARGdemo
{
    public partial class Form1 : Form
    {
        // general global objects
        #if (!TMR_BOARD)
        private xioBoard MyBoard = new xioBoard("COM4");            // x-io Board object
        #else
        private SerialPort sercomm = new SerialPort("COM4", 115200, Parity.None, 8, StopBits.One);
        List<byte> bBuffer = new List<byte>();
        string sBuffer = String.Empty;
        #endif
        private Timer graphicsRefreshTimer = new Timer();           // graphics refresh timer
        private double AEq_1 = 1, AEq_2 = 0, AEq_3 = 0, AEq_4 = 0;  // quaternion orientation of earth frame relative to auxiliary frame

        // calibrated sensor measurements
        double a_x, a_y, a_z;                                       // accelerometer measurements
        double w_x, w_y, w_z;                                       // gyroscope measurements
        double m_x, m_y, m_z;                                       // magnetometer measurements

        // sensor calibration variables and constants
        private const double a_xBias = 32634.2779917682;            // accelerometer bias
        private const double a_yBias = 32300.1140276867;
        private const double a_zBias = 32893.0853282136;
        private const double a_xGain = -0.00150042985864975;        // accelerometer gains
        private const double a_yGain = -0.00147414192905898;
        private const double a_zGain = 0.00152294825926844;
        private double w_xBias = 25247;                             // gyroscope bias
        private double w_yBias = 25126;
        private double w_zBias = 24463;
        private const double w_xGain = 0.00102058528925813;         // gyroscope gains
        private const double w_yGain = -0.00110455853342484;
        private const double w_zGain = 0.00107794298635984;
        private const double m_xBias = -8.20750399495073;           // magnetometer baises
        private const double m_yBias = 15.6531909021474;
        private const double m_zBias = 7.32498941411782;
        private const double m_xGain = -0.00160372297752976;        // magnetometer gains
        private const double m_yGain = 0.0016037818986323;
        private const double m_zGain = 0.00182483736430979;
        private bool initSample = true;                             // flag used to indicate the initial sample

        // filter variables and constants
        private const double gyroMeasError = 30;                                            // gyroscope measurement error (in degrees per second)
        private const double gyroBiasDrift = 0.5;                                           // gyroscope bias drift (in degrees per second per second)
        private double beta = Math.Sqrt(3.0 / 4.0) * (Math.PI * (gyroMeasError / 180.0));   // filter gain beta
        private double zeta = Math.Sqrt(3.0 / 4.0) * (Math.PI * (gyroBiasDrift / 180.0));   // filter gain zeta
        private double SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;                          // estimated orientation quaternion elements with initial conditions
        private const double deltat = 0.001;                                                // sampling period
        private double b_x = 1, b_z = 0;                                                    // estimated direction of earth's magnetic field in the earth frame
        private double w_bx = 0, w_by = 0, w_bz = 0;                                        // estimate gyroscope biases error

        public Form1()
        {
            InitializeComponent();
            #if (!TMR_BOARD)
            MyBoard.DataPacketReceived += new xioBoard.onDataPacketReceived(MyBoard_DataPacketReceived);
            MyBoard.Open();
			#else
            //SearchCommPort();
            sercomm.DataReceived += new SerialDataReceivedEventHandler(SerialDataPacketReceived);
            sercomm.Open();
            #endif
            graphicsRefreshTimer.Interval = 20;
            graphicsRefreshTimer.Tick += new EventHandler(graphicsRefreshTimer_Tick);
            graphicsRefreshTimer.Start();
        }

        #if (!TMR_BOARD)
        private void MyBoard_DataPacketReceived(object sender, IncomingBoardMessage e)
        #else
        private void SerialDataPacketReceived(object sender, EventArgs e)
        #endif
        {
            // Use either the binary OR the string technique (but not both)
            
            // Buffer and process binary data
            while (sercomm.BytesToRead > 0)
                bBuffer.Add((byte)sercomm.ReadByte());
            ProcessBuffer(bBuffer);

            // Buffer string data
            sBuffer += sercomm.ReadExisting();
            ProcessBuffer(sBuffer);
                
            // sample the gyroscope bias as the initial sample
            if (initSample)
            {
                #if (!TMR_BOARD)
                w_xBias = (double)e.ADC[3];
                w_yBias = (double)e.ADC[4];
                w_zBias = (double)e.ADC[5];
                #else
                w_xBias = 0.0;
                w_yBias = 0.0;
                w_zBias = 0.0;
                #endif

                initSample = false;
            }

            // calibrate sensor measurments
            #if (!TMR_BOARD)
            a_x = ((double)e.ADC[0] - a_xBias) * a_xGain;                                       // 16-bit uint ADC acceleroemter values to m/s/s
            a_y = ((double)e.ADC[1] - a_yBias) * a_yGain;
            a_z = ((double)e.ADC[2] - a_zBias) * a_zGain;
            w_x = ((double)e.ADC[3] - w_xBias) * w_xGain;                                       // 16-bit uint ADC gyrocope values to rad/s (HP filtered)
            w_y = ((double)e.ADC[4] - w_yBias) * w_yGain;
            w_z = ((double)e.ADC[5] - w_zBias) * w_zGain;
            m_x = ((short)(((e.Comms.Data[2]) << 8) | e.Comms.Data[3]) - m_xBias) * m_xGain;    // 2*char to signed int16 magnetometer values (normalised units)
            m_y = ((short)(((e.Comms.Data[0]) << 8) | e.Comms.Data[1]) - m_yBias) * m_yGain;
            m_z = ((short)(((e.Comms.Data[4]) << 8) | e.Comms.Data[5]) - m_zBias) * m_zGain;
            #else
            a_x = 0;  // 16-bit uint ADC acceleroemter values to m/s/s
            a_y = 0;
            a_z = 100;
            w_x = 0 - w_xBias;  // 16-bit uint ADC gyrocope values to rad/s (HP filtered)
            w_y = 0 - w_yBias;
            w_z = 0 - w_zBias;
            m_x = 100.0; // 2*char to signed int16 magnetometer values (normalised units)
            m_y = 0;
            m_z = 100.0;
            #endif

            // update filter with sensor data
            filterUpdate(w_x, w_y, w_z, a_x, a_y, a_z, m_x, m_y, m_z);
        }

        private void ProcessBuffer(string sBuffer)
        {
            // Look in the string for useful information
            // then remove the useful data from the buffer
        }

        private void ProcessBuffer(List<byte> bBuffer)
        {
            // Look in the byte array for useful information
            // then remove the useful data from the buffer
        }

        private void filterUpdate(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z, double m_x, double m_y, double m_z)
        {
            // local system variables
            double norm;                                                            // vector norm
            double SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;  // quaternion rate from gyroscopes elements
            double f_1, f_2, f_3, f_4, f_5, f_6;                                    // objective function elements
            double J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,              // objective function Jacobian elements
            J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
            double SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;              // estimated direction of the gyroscope error (quaternion derrivative)
            double w_err_x, w_err_y, w_err_z;                                       // estimated direction of the gyroscope error (angular)
            double h_x, h_y, h_z;                                                   // computed flux in the earth frame

            // axulirary variables to avoid reapeated calcualtions
            double halfSEq_1 = 0.5 * SEq_1;
            double halfSEq_2 = 0.5 * SEq_2;
            double halfSEq_3 = 0.5 * SEq_3;
            double halfSEq_4 = 0.5 * SEq_4;
            double twoSEq_1 = 2.0 * SEq_1;
            double twoSEq_2 = 2.0 * SEq_2;
            double twoSEq_3 = 2.0 * SEq_3;
            double twoSEq_4 = 2.0 * SEq_4;
            double twob_x = 2 * b_x;
            double twob_z = 2 * b_z;
            double twob_xSEq_1 = 2 * b_x * SEq_1;
            double twob_xSEq_2 = 2 * b_x * SEq_2;
            double twob_xSEq_3 = 2 * b_x * SEq_3;
            double twob_xSEq_4 = 2 * b_x * SEq_4;
            double twob_zSEq_1 = 2 * b_z * SEq_1;
            double twob_zSEq_2 = 2 * b_z * SEq_2;
            double twob_zSEq_3 = 2 * b_z * SEq_3;
            double twob_zSEq_4 = 2 * b_z * SEq_4;
            double SEq_1SEq_2;
            double SEq_1SEq_3 = SEq_1 * SEq_3;
            double SEq_1SEq_4;
            double SEq_2SEq_3;
            double SEq_2SEq_4 = SEq_2 * SEq_4;
            double SEq_3SEq_4;
            double twom_x = 2 * m_x;
            double twom_y = 2 * m_y;
            double twom_z = 2 * m_z;

            // normalise the accelerometer measurement
            norm = Math.Sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
            a_x /= norm;
            a_y /= norm;
            a_z /= norm;

            // normalise the magnetometer measurement
            norm = Math.Sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
            m_x /= norm;
            m_y /= norm;
            m_z /= norm;

            // compute the objective function and jacobian
            f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
            f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
            f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
            f_4 = twob_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
            f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
            f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;

            J_11or24 = twoSEq_3;                                                    // J_11 negated in matrix multiplication
            J_12or23 = 2 * SEq_4;
            J_13or22 = twoSEq_1;                                                    // J_12 negated in matrix multiplication
            J_14or21 = twoSEq_2;
            J_32 = 2 * J_14or21;                                                    // negated in matrix multiplication
            J_33 = 2 * J_11or24;                                                    // negated in matrix multiplication
            J_41 = twob_zSEq_3;                                                     // negated in matrix multiplication
            J_42 = twob_zSEq_4;
            J_43 = 2 * twob_xSEq_3 + twob_zSEq_1;                                   // negated in matrix multiplication
            J_44 = 2 * twob_xSEq_4 - twob_zSEq_2;                                   // negated in matrix multiplication
            J_51 = twob_xSEq_4 - twob_zSEq_2;                                       // negated in matrix multiplication
            J_52 = twob_xSEq_3 + twob_zSEq_1;
            J_53 = twob_xSEq_2 + twob_zSEq_4;
            J_54 = twob_xSEq_1 - twob_zSEq_3;                                       // negated in matrix multiplication
            J_61 = twob_xSEq_3;
            J_62 = twob_xSEq_4 - 2 * twob_zSEq_2;
            J_63 = twob_xSEq_1 - 2 * twob_zSEq_3;
            J_64 = twob_xSEq_2;

            // compute the gradient (matrix multiplication)
            SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
            SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
            SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
            SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

            // normalise the gradient to estimate direction of the gyroscope error
            norm = Math.Sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
            //#if (!TMR_BOARD)
            SEqHatDot_1 = SEqHatDot_1 / norm;
            SEqHatDot_2 = SEqHatDot_2 / norm;
            SEqHatDot_3 = SEqHatDot_3 / norm;
            SEqHatDot_4 = SEqHatDot_4 / norm;
            //#endif

            // compute angular estimated direction of the gyroscope error
            w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
            w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
            w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

            // compute and remove the gyroscope baises
            w_bx += w_err_x * deltat * zeta;
            w_by += w_err_y * deltat * zeta;
            w_bz += w_err_z * deltat * zeta;

            w_x -= w_bx;
            w_y -= w_by;
            w_z -= w_bz;

            // compute the quaternion rate measured by gyroscopes
            SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
            SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
            SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
            SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

            // compute then integrate the estimated quaternion rate
            SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
            SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
            SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
            SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

            // normalise quaternion
            norm = Math.Sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
            //#if (!TMR_BOARD) will caused accumulated error
            SEq_1 /= norm;
            SEq_2 /= norm;
            SEq_3 /= norm;
            SEq_4 /= norm;
            //#endif

            // compute flux in the earth frame
            SEq_1SEq_2 = SEq_1 * SEq_2;                                             // recompute axulirary variables
            SEq_1SEq_3 = SEq_1 * SEq_3;
            SEq_1SEq_4 = SEq_1 * SEq_4;
            SEq_3SEq_4 = SEq_3 * SEq_4;
            SEq_2SEq_3 = SEq_2 * SEq_3;
            SEq_2SEq_4 = SEq_2 * SEq_4;

            h_x = twom_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
            h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
            h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3);

            // normalise the flux vector to have only components in the x and z
            b_x = Math.Sqrt((h_x * h_x) + (h_y * h_y));
            b_z = h_z;
        }

        private void graphicsRefreshTimer_Tick(object sender, EventArgs e)
        {
            double ESq_1, ESq_2, ESq_3, ESq_4;                              // quaternion describing orientation of sensor relative to earth
            double ASq_1, ASq_2, ASq_3, ASq_4;                              // quaternion describing orientation of sensor relative to auxiliary frame
            double r_11, r_12, r_13, r_21, r_22, r_23, r_31, r_32, r_33;    // rotation matrix elements
            double phi, theta, psi;                                         // Euler angles
            Graphics graphics = this.CreateGraphics();                      // graphics object for visualisation
            Pen penB; ;                                                     // blue pen for x-axis
            Pen penR;                                                       // blue pen for x-axis
            Pen penG;                                                       // blue pen for x-axis
            Font font = new Font("Arial", 12);                              // font object for graphics text
            #if (!TMR_BOARD)
            int origin_x = this.ClientRectangle.Width / 2;                  // x origin of axis in form
            #else
            int origin_x = this.ClientRectangle.Width / 2 + 100;                  // x origin of axis in form
            #endif
            int origin_y = this.ClientRectangle.Height / 2;                 // y origin of axis in form
            int axisLength = (int)(0.8 * origin_y);                         // length of axis displayed in graphics
            int axisWidth = (int)(0.075 * axisLength);                      // width of the axis displayed in graphics

            //#if (TMR_BOARD)
            //SerialDataPacketReceived(sender, e); // to get sensor informations
            //#endif

            // compute the quaternion conjugate
            ESq_1 = SEq_1;
            ESq_2 = -SEq_2;
            ESq_3 = -SEq_3;
            ESq_4 = -SEq_4;

            // compute the quaternion product
            ASq_1 = ESq_1 * AEq_1 - ESq_2 * AEq_2 - ESq_3 * AEq_3 - ESq_4 * AEq_4;
            ASq_2 = ESq_1 * AEq_2 + ESq_2 * AEq_1 + ESq_3 * AEq_4 - ESq_4 * AEq_3;
            ASq_3 = ESq_1 * AEq_3 - ESq_2 * AEq_4 + ESq_3 * AEq_1 + ESq_4 * AEq_2;
            ASq_4 = ESq_1 * AEq_4 + ESq_2 * AEq_3 - ESq_3 * AEq_2 + ESq_4 * AEq_1;

            // compute the Euler anles from the quaternion
            phi = Math.Atan2(2 * ASq_3 * ASq_4 - 2 * ASq_1 * ASq_2, 2 * ASq_1 * ASq_1 + 2 * ASq_4 * ASq_4 - 1);
            theta = -Math.Asin(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_3);
            psi = Math.Atan2(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_4, 2 * ASq_1 * ASq_1 + 2 * ASq_2 * ASq_2 - 1);

            // compute rotation matrix from quaternion
            r_11 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_2 * ASq_2;
            r_12 = 2 * (ASq_2 * ASq_3 + ASq_1 * ASq_4);
            r_13 = 2 * (ASq_2 * ASq_4 - ASq_1 * ASq_3);
            r_21 = 2 * (ASq_2 * ASq_3 - ASq_1 * ASq_4);
            r_22 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_3 * ASq_3;
            r_23 = 2 * (ASq_3 * ASq_4 + ASq_1 * ASq_2);
            r_31 = 2 * (ASq_2 * ASq_4 + ASq_1 * ASq_3);
            r_32 = 2 * (ASq_3 * ASq_4 - ASq_1 * ASq_2);
            r_33 = 2 * ASq_1 * ASq_1 - 1 + 2 * ASq_4 * ASq_4;

            // draw sensor frame axes to graphics object in correct order
            penB = new Pen(Color.Blue, axisWidth);
            penR = new Pen(Color.Red, axisWidth);
            penG = new Pen(Color.Green, axisWidth);

            graphics.Clear(Color.Black);

            if ((r_21 > r_22) && (r_21 > r_23))
                graphics.DrawLine(penB, origin_x, origin_y, (int)(origin_x + (axisLength * r_11)), (int)(origin_y + (axisLength * -r_31)));
            else if ((r_22 > r_21) && (r_22 > r_23))
                graphics.DrawLine(penR, origin_x, origin_y, (int)(origin_x + (axisLength * r_12)), (int)(origin_y + (axisLength * -r_32)));
            else if ((r_23 > r_21) && (r_23 > r_22))
                graphics.DrawLine(penG, origin_x, origin_y, (int)(origin_x + (axisLength * r_13)), (int)(origin_y + (axisLength * -r_33)));
            if (((r_21 > r_22) && (r_21 < r_23)) || ((r_21 < r_22) && (r_21 > r_23)))
                graphics.DrawLine(penB, origin_x, origin_y, (int)(origin_x + (axisLength * r_11)), (int)(origin_y + (axisLength * -r_31)));
            else if (((r_22 > r_21) && (r_22 < r_23)) || ((r_22 < r_21) && (r_22 > r_23)))
                graphics.DrawLine(penR, origin_x, origin_y, (int)(origin_x + (axisLength * r_12)), (int)(origin_y + (axisLength * -r_32)));
            else if (((r_23 > r_21) && (r_23 < r_22)) || ((r_23 < r_21) && (r_23 > r_22)))
                graphics.DrawLine(penG, origin_x, origin_y, (int)(origin_x + (axisLength * r_13)), (int)(origin_y + (axisLength * -r_33)));
            if ((r_21 < r_22) && (r_21 < r_23))
                graphics.DrawLine(penB, origin_x, origin_y, (int)(origin_x + (axisLength * r_11)), (int)(origin_y + (axisLength * -r_31)));
            else if ((r_22 < r_21) && (r_22 < r_23))
                graphics.DrawLine(penR, origin_x, origin_y, (int)(origin_x + (axisLength * r_12)), (int)(origin_y + (axisLength * -r_32)));
            else if ((r_23 < r_21) && (r_23 < r_22))
                graphics.DrawLine(penG, origin_x, origin_y, (int)(origin_x + (axisLength * r_13)), (int)(origin_y + (axisLength * -r_33)));

            graphics.DrawString("x", font, Brushes.White, (int)(origin_x + (axisLength * r_11)), (int)(origin_y + (axisLength * -r_31)));
            graphics.DrawString("y", font, Brushes.White, (int)(origin_x + (axisLength * r_12)), (int)(origin_y + (axisLength * -r_32)));
            graphics.DrawString("z", font, Brushes.White, (int)(origin_x + (axisLength * r_13)), (int)(origin_y + (axisLength * -r_33)));

            // add sensor data text to graphics object
            graphics.DrawString("Accelerometer (m/s/s):", font, Brushes.White, 0, 10);
            graphics.DrawString("x = " + String.Format("{0:0.00}", a_x), font, Brushes.White, 5, 25);
            graphics.DrawString("y = " + String.Format("{0:0.00}", a_y), font, Brushes.White, 5, 40);
            graphics.DrawString("z = " + String.Format("{0:0.00}", a_z), font, Brushes.White, 5, 55);

            graphics.DrawString("Gyroscope (rad/s):", font, Brushes.White, 0, 75);
            graphics.DrawString("x = " + String.Format("{0:0.00}", w_x - w_bx), font, Brushes.White, 5, 90);
            graphics.DrawString("y = " + String.Format("{0:0.00}", w_y - w_by), font, Brushes.White, 5, 105);
            graphics.DrawString("z = " + String.Format("{0:0.00}", w_z - w_bz), font, Brushes.White, 5, 120);

            graphics.DrawString("Magnetometer (unit):", font, Brushes.White, 0, 140);
            graphics.DrawString("x = " + String.Format("{0:0.00}", m_x), font, Brushes.White, 5, 155);
            graphics.DrawString("y = " + String.Format("{0:0.00}", m_y), font, Brushes.White, 5, 170);
            graphics.DrawString("z = " + String.Format("{0:0.00}", m_z), font, Brushes.White, 5, 185);

            // add internal filter states to graphics object
            graphics.DrawString("Earth's flux (unit):", font, Brushes.White, 0, 205);
            graphics.DrawString("x = " + String.Format("{0:0.00}", b_x), font, Brushes.White, 5, 220);
            graphics.DrawString("z = " + String.Format("{0:0.00}", b_z), font, Brushes.White, 5, 235);
            graphics.DrawString("Gyroscope biases (rad/s):", font, Brushes.White, 0, 255);
            graphics.DrawString("x = " + String.Format("{0:0.00}", w_bx), font, Brushes.White, 5, 270);
            graphics.DrawString("y = " + String.Format("{0:0.00}", w_by), font, Brushes.White, 5, 285);
            graphics.DrawString("z = " + String.Format("{0:0.00}", w_bz), font, Brushes.White, 5, 300);

            // add Euler angles to graphics object 
            graphics.DrawString("Eular angles (degrees):", font, Brushes.White, 0, 320);
            graphics.DrawString("x = " + String.Format("{0:0.0}", phi * (180.0 / Math.PI)), font, Brushes.White, 5, 335);
            graphics.DrawString("y = " + String.Format("{0:0.0}", theta * (180.0 / Math.PI)), font, Brushes.White, 5, 350);
            graphics.DrawString("z = " + String.Format("{0:0.0}", psi * (180.0 / Math.PI)), font, Brushes.White, 5, 365);
        }

        // Find the friendly comport name for the TMR board
        private void SearchCommPort()
        {
            bool find = false;
            try
            {
                ManagementObjectSearcher searcher = new ManagementObjectSearcher("SELECT * FROM Win32_PnPEntity");

                foreach (ManagementObject obj in searcher.Get())
                {
                    #if (USE_SERIAL_1)
                    if (obj["Name"].ToString().Contains("GoldenBridge") && obj["DeviceID"].ToString().Contains("VID_1680&PID_0460"))
                    {
                        {
                            find = true;
                            MessageBox.Show("COM : " + obj["Name"].ToString());
                            break;

                            //Dim startIndex As Int16 = queryObj("Name").ToString.IndexOf("(") + 1
                            //Dim COM_Length As Int16 = queryObj("Name").ToString.IndexOf(")") - startIndex

                            //Dim portName As String = queryObj("Name").ToString.Substring(startIndex, COM_Length)
                            //tbcPort.Text = portName
                            //btnConnect.PerformClick()
                        }
                    }
                    #endif

                    #if (USE_SERIAL_2)
					if (obj["Name"].ToString().Contains("Silicon Labs CP210x") && obj["DeviceID"].ToString().Contains("VID_10C4&PID_EA60"))
                    {
                        {
                            find = true;
                            MessageBox.Show("COM : " + obj["Name"].ToString());
                            break;
                        }
                    }
                    #endif

                    #if (USE_SERIAL_3)
                    if (obj["Name"].ToString().Contains("USB-SERIAL CH340") && obj["DeviceID"].ToString().Contains("VID_1A86&PID_7523"))
                    {
                        {
                            find = true;
                            MessageBox.Show("COM : " + obj["Name"].ToString());
                            break;
                        }
                    }
                    #endif
                }
                if (find != true)
                {
                    MessageBox.Show("COM : NO COM find!!");
                    //Application.Exit();
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("An error occurred while querying for WMI data: " + ex.Message);
            }
        }

        private void Form1_MouseDown(object sender, MouseEventArgs e)                   // hold mouse button to speed up convergence with high gains
        {
            beta = 10;
            zeta = 5;
        }

        private void Form1_MouseUp(object sender, MouseEventArgs e)                     // release mouse button to revert to normal gains and reset graphic's orientation
        {
            beta = Math.Sqrt(3.0 / 4.0) * (Math.PI * (gyroMeasError / 180.0));
            zeta = Math.Sqrt(3.0 / 4.0) * (Math.PI * (gyroBiasDrift / 180.0));
            AEq_1 = SEq_1;                                                              // store orientation of auxiliary frame
            AEq_2 = SEq_2;
            AEq_3 = SEq_3;
            AEq_4 = SEq_4;
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            //sercomm.Close();
            //this.Close();
            Environment.Exit(Environment.ExitCode);
        }

    }
}
