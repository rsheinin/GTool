
import argparse
import signal
import os
import sys
import shutil
import platform
from GTProvider import OT_Provider, VR_Provider
from Recorder import TM2_Recorder,Host_Recorder


signal.signal(signal.SIGINT, signal.SIG_IGN)
def parseArguments():
    parser = argparse.ArgumentParser(
        description="Wrapper for sensor recording: pre process, open process, post process")

    parser.add_argument("--outdir", type=str, default="/home/ntuser/Desktop/recorder_wraper_output",
                        help="output directory")
    parser.add_argument("--outputName", type=str, default="testProcess", help="name for the output rc/6dof/bag ")

    parser.add_argument("--server", type=str, default="192.168.0.102",help="server")
    parser.add_argument("--ot_server", type=str, default="127.0.0.1",help="ot_server")
    parser.add_argument("--ot_client", type=str, default="127.0.0.1",help="ot_client")

    parser.add_argument("--mounted_dir", type=str, default="/home/ntuser/Desktop/GTService/",help="mounted dir")
    parser.add_argument("--capturecheck-path", type=str, default="/home/ntuser/Desktop/tester-script/", help="capturecheck script path")
    parser.add_argument("--exporting-script-path", type=str, default="/home/ntuser/Desktop/exporting-script/", help="rc exporting script path")

    parser.add_argument("--ot", action='store_true', default=True, help="set if 6Dof will be record or not")
    parser.add_argument("--vr", action='store_true', default=False, help="set if VR will be record or not")

    parser.add_argument("--TM2-recorder", type=str, default="",
                        help="set recorder to be tm2, c for rawData 6 for 6dof rc for both")
    parser.add_argument("--Host-recorder", type=str, default="60",
                        help="set recorder to be host, and give time")
    parser.add_argument("--recorder-path", type=str,
                        help="insert recorder folder path")
    parser.add_argument("--TA-params", type=str, default="",
                        help="insert parameters for temporal alighnment")
    parser.add_argument("--Projection-params", type=str, default="",
                        help="insert parameters for projection")
    parser.add_argument("--prediction", type=str ,default="40",
                        help="insert time for prediction")
    parser.add_argument("--hmd", type=str, nargs="+", default=["ES3_20_12"],
                        help="insert hmd rigid body")
    parser.add_argument("--ctrl1", type=str, nargs="+", default=["ctrl_A_18_1_18", "7e4b05738fc0"],
                        help="insert ctrl1 rigid body and mac address")
    parser.add_argument("--ctrl2", type=str, nargs="+",
                        help="insert ctrl2 rigid body and mac address")


    args = parser.parse_args()
    return args

def post_processing(GT,outputName,obj_name):
    try:
        GT.temporalAlignment(outputName,obj_name)
        GT.projection( outputName,obj_name)
        test_status = GT.testOutput(GT.mounted_dir + "/*" + getattr(GT, obj_name).projection_dir_name+"*/"+ getattr(GT, obj_name).rigid_body + "_global.csv")
        if test_status == False:
            print("projection GT is not good")
            test_status = GT.testOutput(GT.mounted_dir + "/" + getattr(GT,obj_name).rigid_body + ".csv")
            if test_status == False:
                print("source GT is not good")

    except:
        print("Unexpected error:", sys.exc_info()[0])

def copy_to_loacl(GTList,Recorder):
    print("------------Copying pose data from {} to {}".format(GTList[0].mounted_dir, Recorder.out_dir))
    shutil.copy(GTList[0].mounted_dir + "/.", Recorder.out_dir + "/")

    if platform.system() == 'Linux':
        command = "sudo chmod -R 777 " + Recorder.out_dir
        os.system(command)

def main():

    args = parseArguments()

    GTList = []

    if args.ot:
        GTList.append(OT_Provider(args.server,args.ot_server,args.ot_client,args.mounted_dir,args.hmd,args.ctrl1,args.ctrl2))
    if args.vr:
        GTList.append(VR_Provider(args.server,args.ot_server,args.ot_client,args.mounted_dir,args.hmd,args.ctrl1,args.ctrl2))
    if args.TM2_recorder:
        Recorder = TM2_Recorder(args.TM2_recorder,args.outputName,args.outdir,args.recorder_path)
    if args.Host_recorder:
        Recorder = Host_Recorder(args.Host_recorder,args.outputName,args.outdir,args.recorder_path)

    try:

        try:
            for GT in GTList:
                GT.init()
                GT.start()

            Recorder.run(GT.ctrl1,GT.ctrl2)

            for GT in GTList:
                GT.stop()
        except:
            print("Unexpected error:", sys.exc_info()[0])
        finally:
            Recorder.arrangeData(GTList[0].mounted_dir,args.exporting_script_path)

        for GT in GTList:
            post_processing(GT,args.outputName,'hmd')
            if GT.ctrl1:
                post_processing(GT,args.outputName,'ctrl1')
            if GT.ctrl2:
                post_processing(GT,args.outputName,'ctrl2')

        if args.prediction:
            try:
                command = "sudo cp "+Recorder.out_dir +"/*_pose.csv "+ GTList[0].mounted_dir+"/ "
                os.system(command)
                GT.prediction(args.outputName,args.prediction)
            except:
                print("prediction Unexpected error:", sys.exc_info()[0])

        copy_to_loacl(GTList, Recorder)
        Recorder.testOutput(args.capturecheck_path)
        
    except:
        print("Unexpected error:", sys.exc_info()[0])
    finally:
        for GT in GTList:
            GT.shutdown()
    

if __name__ == '__main__':
    main()
