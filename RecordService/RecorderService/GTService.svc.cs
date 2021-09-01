using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Diagnostics;


namespace RecorderService
{
    
    // NOTE: You can use the "Rename" command on the "Refactor" menu to change the class name "Service1" in code, svc and config file together.
    // NOTE: In order to launch WCF Test Client for testing this service, please select Service1.svc or Service1.svc.cs at the Solution Explorer and start debugging.
    public class ServiceKpi : IServiceContract
    { 

        public static int gtSource;
        public static string asmToUse;
        public static string classToUse;
        public static string trackingSystem;
        public static GTData gtData;
        public static string outputPath = @"..\..\..\..\output\OptitrackResults\PosesFiles";
        public static string outputFolder = string.Empty;
        //output folder in relative to shared path on server (local machine of service)
        public static string outputFolderRelative = string.Empty;
        public static int frameNumber;
        public static Assembly asm;
        public static Type type;
        private static string RBName = string.Empty;
        private const string ghcFolder = @"..\..\..\..\..\HandEyeCalibrationService\GHCFiles";
        private const string libFolder = @"..\..\..\..\..\Common\GTUtils";
        private const string helpScriptsFolder = @"..\..\..\..\..\help_scripts";
        private const string PostProcessingFolder = @"..\..\..\..\..\PostProcessing\";
        private const string rubbishValue = "na";

        public static void initReflection()
        {
            string absoluteLibFolderPath = Path.GetFullPath(libFolder);
            trackingSystem = Enum.GetName(typeof(GTSource), gtSource);
#if DEBUG
            absoluteLibFolderPath += "\\Debug";
#else
            absoluteLibFolderPath += "\\Release";
#endif
            asm = Assembly.LoadFile(Directory.GetFiles(absoluteLibFolderPath, "*.dll").FirstOrDefault(x => x.Contains(trackingSystem)));
            type = asm.ExportedTypes.FirstOrDefault(x => x.Namespace.Contains("Wrapper"));//(classToUse);
            
        }

        object reflectionMethod(string funcToInvoke, object[] parametersArray)
        {
            object result = null;
            if (type != null)
            {
                MethodInfo methodInfo = type.GetMethods().FirstOrDefault(x => x.Name.Contains(funcToInvoke));

                if (methodInfo != null)
                {
                    ParameterInfo[] parameters = methodInfo.GetParameters();
                    object classInstance = Activator.CreateInstance(type, null);

                    if (parameters.Length == 0)
                    {
                        result = methodInfo.Invoke(classInstance, null);
                    }
                    else
                    {
                        result = methodInfo.Invoke(classInstance, parametersArray);
                    }

                }
            }

            return result;
        }

        private bool init()
        {
            CreateDirectory();
            return true;
        }

        private void shutdown()
        {
            //TODO: loop over all gt sources as a list and verify all are null
            if (gtData.vr == null && gtData.ot == null)
                outputFolder = string.Empty;
        }

        private void CreateDirectory(string suffix = "")
        {
            if (outputFolder == string.Empty)
            {
                bool exist = Directory.Exists(outputPath);
                DirectoryInfo directoryInfo = null;
                if (!exist)
                    directoryInfo = Directory.CreateDirectory(outputPath);
                if (exist || (directoryInfo != null && directoryInfo.Exists))//if poses files folder has been created before, or has created now
                {
                    string dirName = DateTime.Now.ToString("dd_MM_yyyy") + "_" + DateTime.Now.ToString("HH_mm_ss") + suffix;
                    DirectoryInfo dinf = Directory.CreateDirectory(outputPath + @"\" + dirName);
                    outputFolder = dinf.FullName;
                    outputFolderRelative = dinf.Name;
                    coloredText("Saves files into " + outputFolder, ConsoleColor.Green); 
                }
                else
                    coloredText("Can not find path: " + outputPath + "\nfiles will not be saved!", ConsoleColor.Red);
                
            }            
        }

        public string OTInit(string rb_name, string motive_serverIP, string motive_clientIP)
        {
            init();
            gtData.ot = new OT();
            gtData.ot.outputFolder = outputFolder + "\\" + gtData.ot.name;
            Directory.CreateDirectory(gtData.ot.outputFolder);
            RBName = String.Empty;
            frameNumber = 0;
            try
            {

                Object[] paramArr = null;
                if (gtSource == (int)GTSource.Optotrak)
                    paramArr = new Object[] { float.Parse(rb_name), float.Parse(motive_serverIP), float.Parse(motive_clientIP) };
                else if (gtSource == (int)GTSource.Optitrack)
                {
                    RBName = rb_name;
                    paramArr = new Object[] { rb_name, motive_serverIP, motive_clientIP, gtData.ot.outputFolder };
                }
                reflectionMethod("Init", paramArr).ToString();
                return outputFolderRelative;
                }
            catch (Exception ex)
            {
                return String.Empty + ex.GetBaseException();
            }
        }

        public void VRInit(string host, string domain, string user, string password, string shared)
        {
            init();
            gtData.vr = new VR();
            gtData.vr.outputFolder = outputFolder + "\\" + gtData.vr.name;
            Directory.CreateDirectory(gtData.vr.outputFolder);
            gtData.vr.targetMachine = host;
            gtData.vr.domain = domain;
            gtData.vr.user = user;
            gtData.vr.password = password;
            gtData.vr.shared = shared;
            string command = "\\\\" + gtData.vr.targetMachine + " -u " + gtData.vr.domain + "\\" + gtData.vr.user + " -p " + gtData.vr.password + " taskkill /im openvr_recorder.exe /f ";
            //runProcess("\\\\" + gtData.vr.targetMachine + " -u " + gtData.vr.user + " -p " + gtData.vr.password + " taskkill /im openvr_recorder.exe /f ", "psexec");
            runProcess(command, "psexec");
        }

        public string GetGHCString()
        {
            string res = string.Empty;
            try
            {
                string[] lines = File.ReadAllLines(ghcFolder + "\\" + RBName + ".txt");
                res += lines.FirstOrDefault(s => s.Contains("GHC_mat")).Split('=')[1];
            }
            catch (Exception)
            {
                Console.Write("can not find ghc file for " + RBName + "\n");
            }
            return res;
        }

        public void OTStart()
        {
            reflectionMethod("Start", null);
        }

        public bool VRStart()
        {
            //runProcess("", @"C:\Users\ntuser\Documents\TFS\CVL\Tools\GTool\RecordService\output\OptitrackResults\PosesFiles\18_10_2017_13_45_18\New Text Document.bat");
            string command = "\\\\" + gtData.vr.targetMachine + " -d -u " + gtData.vr.domain + "\\" + gtData.vr.user + " -p " + gtData.vr.password + " -i VR_recorder_start";//-i 2
            int res = runProcess(command, "psexec");
            //int res = runProcess("\\\\edebby-perc -i 1 -d -u ger\\lab_lablabomek -p trio_012 VR_recorder_start", /*"psexec"*/"C:\\Users\\ntuser\\Documents\\installers\\PsTools\\PsExec.exe");
            //return res == 0 ? true : false;
            return !Convert.ToBoolean(res);
        }

        public void OTStop()
        {
            //OptitrackLibWrapper.OptitrackWrapper.optitrackStopGetPoses();
            reflectionMethod("Stop", null);
            reflectionMethod("Shutdown", null);
        }

        public bool VRStop()
        {
            //int res = runProcess("\\\\edebby-perc -i 1 -d -u ger\\lab_lablabomek -p trio_012 VR_recorder_stop", /*"psexec"*/"C:\\Users\\ntuser\\Documents\\installers\\PsTools\\PsExec.exe");
            string command = "\\\\" + gtData.vr.targetMachine + " -d -u " + gtData.vr.domain + "\\" + gtData.vr.user + " -p " + gtData.vr.password + " -i VR_recorder_stop";//-i 2
            int res = runProcess(command, "psexec");
            //return !Convert.ToBoolean(res);

            string location = "\\\\" + gtData.vr.targetMachine + "\\" + gtData.vr.shared;
            try
            {
                var mostRecentlyModified = Directory.GetFiles(location, "VR*.txt")
                                            .Select(f => new FileInfo(f))
                                            .OrderByDescending(fi => fi.LastWriteTime)
                                            .First()
                                            .FullName;
                coloredText("vive file is: " + mostRecentlyModified + "\n", ConsoleColor.DarkBlue, ConsoleColor.Yellow);
                const string VR_converter = helpScriptsFolder + "\\python\\convertBrekel2Tum.py";
                string VR_converter_absolutePath = Path.GetFullPath(VR_converter);
                runProcess(VR_converter_absolutePath + " " + mostRecentlyModified + " " + gtData.vr.outputFolder + "\\VR_"/* + RBName*/, "python");
                runProcess("/C move " + mostRecentlyModified + " " + gtData.vr.outputFolder, "cmd.exe");
            }
            catch (Exception)
            {
                coloredText("Can not find VR files in: " + location, ConsoleColor.Red);
            }
            return true;
        }

        public bool OTShutdown()
        {
            //reflectionMethod("Shutdown", null);
            gtData.ot = null;
            Console.WriteLine("\n****************************Shutdown OT provider*****************************\n");
            shutdown();
            return true;
        }

        public bool VRShutdown()
        {
            gtData.vr = null;
            Console.WriteLine("\n****************************Shutdown VR provider*****************************\n");
            shutdown();
            return true;
        }

        public bool OTIsInRange()
        {
            return (bool)reflectionMethod("InRange", null); ;
        }

        public bool runPrediction(string file_name, string input_foramt, string time)
        {
            string prediction_tool_rel_path = PostProcessingFolder + @"Prediction\predictPoses.py";
            string prediction_tool_abs_path = Path.GetFullPath(prediction_tool_rel_path);
            //TODO: distinguish between different GT sources
            string folder = "";
            if (gtData.ot != null)
                folder = gtData.ot.outputFolder;

            string input_file_path =  folder + "\\" + file_name;
            
            string in_config_partial_name = input_foramt;
            string formats_folder = PostProcessingFolder + @"..\Common\python\PosesReader\KnownFormats\";
            DirectoryInfo hdDirectoryInWhichToSearch = new DirectoryInfo(formats_folder);
            FileInfo[] filesInDir = hdDirectoryInWhichToSearch.GetFiles("*" + in_config_partial_name + "*.ini");
            if (filesInDir != null)
            {
                string in_config_full_name = filesInDir[0].FullName;
                string out_config_rel_path = formats_folder + "TUM.ini";
                string out_config_abs_path = Path.GetFullPath(out_config_rel_path);
                string command = prediction_tool_abs_path + " --in_file_path " + input_file_path + " --in_config_file " + in_config_full_name
                    + " --out_file_path " + folder + "\\" + file_name.Replace("pose.csv","p.txt") + " --prediction_time_in_ms "
                    + Convert.ToString(time) + " --out_config_file " + out_config_abs_path;
                Console.WriteLine("-------------running prediction:-----------\n" + command);
                runProcess(command, "python");
                return true; 
            }
            return false;
        }
        //public bool GTPostProcessing(string TA_params, string proj_params)
        //{
        //    bool res = true;
        //    //TODO: loop over all gt sources as a list and run post processing for all of them
        //    if (gtData.ot != null)
        //        res &= OTTemporalAlignment(TA_params) && OTProjection(proj_params);
        //    if (gtData.vr != null)
        //        res &= VRTemporalAlignment(TA_params) && VRProjection(proj_params);
        //    return res;

        //}

        public bool OTTemporalAlignment(string TA_params)
        {
            string TA_str = string.Empty;
            return !Convert.ToBoolean(runTemporalAlignment(TA_str + " " + TA_params));
        }

        public bool VRTemporalAlignment(string TA_params)
        {
            string TA_str = string.Empty;
            if (gtData.vr != null)
                TA_str = "-folder " + gtData.vr.outputFolder + " -rb " + RBName;//default arguments, determined by service
            return !Convert.ToBoolean(runTemporalAlignment(TA_str + " " + TA_params));

        }

        public bool OTProjection(string proj_params)
        {
            string proj_str = string.Empty;
            if (gtData.ot != null)
                proj_str = "-folder " + gtData.ot.outputFolder + " -head " + RBName;//default arguments, determined by service
            return !Convert.ToBoolean(runProjection(proj_str + " " + proj_params));
        }

        public bool VRProjection(string proj_params)
        {
            string proj_str = string.Empty;
            if (gtData.vr != null)
                proj_str += "-folder " + gtData.vr.outputFolder + " -head " + RBName;//default arguments, determined by service
            return !Convert.ToBoolean(runProjection(proj_str + " " + proj_params));
        }

        private int runTemporalAlignment(string TA_parameters)
        {
            Console.Write("parameters for temporal alignment: " + TA_parameters + "\n");
            return runProcess(TA_parameters, Path.GetFullPath(PostProcessingFolder + @"OT_TemporalAlignment\x64\Release\OT_TemporalAlignment.exe"));
        }
        private int runProjection(string projection_parameters)
        {
            Console.Write("parameters for projection: " + projection_parameters + "\n");
            return runProcess(projection_parameters, Path.GetFullPath(PostProcessingFolder + @"Projector\x64\Release\OT_Projector.exe"));
        }

        public string VRGetWorkingDirectory()
        {
            return GetWorkingDirectory(gtData.vr);
        }

        public string OTGetWorkingDirectory()
        {
            return GetWorkingDirectory(gtData.ot);
        }

        private string GetWorkingDirectory(GT gt)
        {
            return gt.outputFolder;
        }
        private int runProcess(string arguments, string exePath)
        {
            // Prepare the process to run
            ProcessStartInfo start = new ProcessStartInfo();
            // Enter in the command line arguments, everything you would enter after the executable name itself
            start.Arguments = arguments;
            // Enter the executable to run, including the complete path
            start.FileName = exePath;
            // Do you want to show a console window?
            start.WindowStyle = ProcessWindowStyle.Hidden;
            start.UseShellExecute = false;
            string dir = Path.GetDirectoryName(exePath);
            //Console.WriteLine("working directory for " + exePath + " is: " + dir);
            start.WorkingDirectory = dir;
            //start.CreateNoWindow = true;

            // Run the external process & wait for it to finish
            using (Process proc = Process.Start(start))
            {
                proc.WaitForExit();

                // Retrieve the app's exit code
                return proc.ExitCode;
            }
        }
        private void coloredText(string str, ConsoleColor fore = ConsoleColor.White, ConsoleColor back = ConsoleColor.Black)
        {
            Console.ForegroundColor = fore;
            Console.BackgroundColor = back;
            Console.WriteLine(str);
            Console.ResetColor();
        }

    }
}

