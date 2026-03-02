using XemaCsharp;
using System;
using System.Runtime.InteropServices;
using System.Text;
using System.IO;

namespace example
{
    class Program
    {
        static void Main(string[] args)
        {
            string exeDir = AppDomain.CurrentDomain.BaseDirectory;
            Console.WriteLine("Files will be saved to: " + exeDir + "\n");

            Console.Write("Enter camera IP address: ");
            string cameraId = Console.ReadLine();
            if (string.IsNullOrWhiteSpace(cameraId)) return;

            XemaCls xema = new XemaCls();
            if (xema.DfConnect_Csharp(cameraId) != 0)
            {
                Console.WriteLine("Connection failed!");
                Console.ReadKey();
                return;
            }

            int width, height, channels;
            xema.GetCameraResolution_Csharp(out width, out height);
            xema.DfGetCameraChannels_Csharp(out channels);
            Console.WriteLine($"Resolution: {width}x{height}, Channels: {channels}");

            // Get and save calibration
            CalibrationParam calib;
            if (xema.DfGetCalibrationParam_Csharp(out calib) == 0)
            {
                SaveCalibration(Path.Combine(exeDir, "calibration.txt"), calib);
            }

            // Allocate memory
            byte[] brightness = new byte[width * height];
            float[] depth = new float[width * height];
            float[] pointcloud = new float[width * height * 3];

            IntPtr pBright = Marshal.AllocHGlobal(brightness.Length);
            IntPtr pDepth = Marshal.AllocHGlobal(depth.Length * 4);
            IntPtr pCloud = Marshal.AllocHGlobal(pointcloud.Length * 4);

            // Set camera parameters
            xema.DfSetParamSmoothing_Csharp(1);
            xema.DfSetParamCameraConfidence_Csharp(2);
            xema.DfSetParamCameraGain_Csharp(0);
            xema.DfSetParamDepthFilter_Csharp(1, 70);
            xema.DfSetParamRadiusFilter_Csharp(1, 2.5f, 20);
            xema.DfSetParamReflectFilter_Csharp(1, 20);

            // Capture HDR
            int[] exposure = { 5000, 8000 };
            int[] led = { 1023, 1023 };
            // Single exposure mode
            xema.DfSetParamLedCurrent_Csharp(1023);          // LED brightness
            xema.DfSetParamCameraExposure_Csharp(20000);     // Single exposure time
            xema.DfSetCaptureEngine_Csharp(XemaEngine.Reflect);

            StringBuilder timestamp = new StringBuilder(30);
            Console.WriteLine("Capturing...");
            if (xema.DfCaptureData_Csharp(1, timestamp) == 0)  // 1 = single exposure
            {
                Console.WriteLine("Capture success! Timestamp: " + timestamp);

                // Get brightness
                if (xema.DfGetBrightnessData_Csharp(pBright) == 0)
                {
                    Marshal.Copy(pBright, brightness, 0, brightness.Length);
                    SavePGM(Path.Combine(exeDir, "brightness.pgm"), brightness, width, height);
                    Console.WriteLine("Saved: brightness.pgm");
                }

                // Get depth
                if (xema.DfGetDepthDataFloat_Csharp(pDepth) == 0)
                {
                    Marshal.Copy(pDepth, depth, 0, depth.Length);
                    SaveRaw(Path.Combine(exeDir, "depth.raw"), depth, width, height);
                    Console.WriteLine("Saved: depth.raw + depth.txt");
                }

                // Get point cloud
                if (xema.DfGetPointcloudData_Csharp(pCloud) == 0)
                {
                    Marshal.Copy(pCloud, pointcloud, 0, pointcloud.Length);

                    // Save height map
                    float[] heightmap = new float[width * height];
                    for (int i = 0; i < width * height; i++)
                        heightmap[i] = pointcloud[i * 3 + 2];
                    SaveRaw(Path.Combine(exeDir, "heightmap.raw"), heightmap, width, height);
                    Console.WriteLine("Saved: heightmap.raw + heightmap.txt");

                    // Save PLY
                    SavePLY(Path.Combine(exeDir, "pointcloud.ply"), pointcloud, brightness, width, height);
                    Console.WriteLine("Saved: pointcloud.ply");
                }
            }

            Marshal.FreeHGlobal(pBright);
            Marshal.FreeHGlobal(pDepth);
            Marshal.FreeHGlobal(pCloud);

            xema.DfDisconnect_Csharp(cameraId);
            Console.WriteLine("\nDone! Press any key to exit...");
            Console.ReadKey();
        }

        static void SaveCalibration(string path, CalibrationParam c)
        {
            using (var w = new StreamWriter(path))
            {
                w.WriteLine("=== Intrinsic ===");
                foreach (var v in c.intrinsic) w.WriteLine(v);
                w.WriteLine("\n=== Extrinsic ===");
                foreach (var v in c.extrinsic) w.WriteLine(v);
                w.WriteLine("\n=== Distortion ===");
                foreach (var v in c.distortion) w.WriteLine(v);
            }
        }

        // Simple PGM format (Portable Graymap) - very simple header
        static void SavePGM(string path, byte[] data, int w, int h)
        {
            using (var wr = new StreamWriter(path))
            {
                wr.WriteLine("P2");           // PGM magic number
                wr.WriteLine($"{w} {h}");     // Width Height
                wr.WriteLine("255");          // Max gray value

                for (int i = 0; i < data.Length; i++)
                {
                    wr.Write(data[i] + " ");
                    if ((i + 1) % w == 0) wr.WriteLine();
                }
            }
        }

        // Raw binary format with metadata text file
        static void SaveRaw(string path, float[] data, int w, int h)
        {
            // Save binary data
            byte[] bytes = new byte[data.Length * 4];
            Buffer.BlockCopy(data, 0, bytes, 0, bytes.Length);
            File.WriteAllBytes(path, bytes);

            // Save metadata
            string txtPath = Path.ChangeExtension(path, ".txt");
            File.WriteAllText(txtPath, $"Width: {w}\nHeight: {h}\nType: float32\nEndian: little");
        }

        // PLY
        static void SavePLY(string path, float[] cloud, byte[] bright, int w, int h)
        {
            using (var wr = new StreamWriter(path))
            {
                wr.WriteLine("ply");
                wr.WriteLine("format ascii 1.0");
                wr.WriteLine($"element vertex {w * h}");
                wr.WriteLine("property float x");
                wr.WriteLine("property float y");
                wr.WriteLine("property float z");
                wr.WriteLine("property uchar red");
                wr.WriteLine("property uchar green");
                wr.WriteLine("property uchar blue");
                wr.WriteLine("end_header");

                for (int i = 0; i < w * h; i++)
                {
                    wr.WriteLine($"{cloud[i * 3]} {cloud[i * 3 + 1]} {cloud[i * 3 + 2]} {bright[i]} {bright[i]} {bright[i]}");
                }
            }
        }
    }
}