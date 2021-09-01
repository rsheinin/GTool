import os

import urllib.request
import urllib.parse

from GTcheck import GTCheck


def requestController(urlString, action):
    url = urlString
    print(action + ": {}".format(url))
    proxy_support = urllib.request.ProxyHandler({})
    opener = urllib.request.build_opener(proxy_support)
    urllib.request.install_opener(opener)
    response = urllib.request.urlopen(url)
    return response



class Obj:
    def type(self): pass
    def rigid_body(self): pass
    def projection_dir_name(self): pass
    def TA_dir_name(self): pass
    def mac_address(self): pass

    def __init__(self,type,post_process_dtype,rigid_body,mac_address=None):
        self.type = type
        self.rigid_body = rigid_body
        self.post_process_dtype = post_process_dtype
        name = 'val'
        if type == 'hmd':
            name = 'dev'

        self.TA_dir_name = "*"+self.post_process_dtype+"*temporalAlignment*"+self.rigid_body
        self.projection_dir_name = self.post_process_dtype+"_projection_"+self.rigid_body
        self.mac_address = mac_address





class GT_Provider:

    def result_dir (self):
        pass
    def server (self):
        pass
    def port (self):
        pass
    def service_name (self):
        pass
    def ot_server (self):
        pass
    def ot_client (self):
        pass

    def rigid_body (self):
        pass
    def controller_RB (self):
        pass

    def post_process_dtype (self):
        pass

    def mounted_dir (self):
        pass

    def __init__(self,server,ot_server,ot_client,mounted_dir,hmd,ctrl1=None,ctrl2=None):
        self.result_dir = ""
        self.server = server
        self.port = "8080"
        self.service_name = "GTService"
        self.ot_server = ot_server
        self.ot_client = ot_client
        self.url = 'http://' + self.server + ':' + self.port + '/' + self.service_name
        self.post_process_dtype = "6dof"  # options: gyro, 6dof

        self.mounted_dir = mounted_dir

        self.hmd = Obj('hmd',self.post_process_dtype, hmd[0])
        self.ctrl1 = None
        self.ctrl2 = None
        if ctrl1 is not None:
            self.ctrl1 = Obj('ctrl',self.post_process_dtype,ctrl1[0],ctrl1[1])
        if ctrl2 is not None:
            self.ctrl2 = Obj('ctrl',self.post_process_dtype,ctrl2[0],ctrl2[1])
    def prediction(self,outputName, predictionTime):
        requestController(self.url+ '/runPrediction/'+outputName+'_pose.csv/host_pose/'+predictionTime,
                          'run prediction')

    def init(self):
        raise NotImplementedError("GT controller must implement init func")

    def start(self):
        raise NotImplementedError("GT controller must implement start func")

    def stop(self):
        raise NotImplementedError("GT controller must implement stop func")

    def shutdown(self):
        raise NotImplementedError("GT controller must implement shutdown func")

    def temporalAlignment(self,outputName,obj_name):
        raise NotImplementedError("GT controller must implement temporalAlignment func")

    def projection(self,outputName,obj_name):
        raise NotImplementedError("GT controller must implement projection func")

    def testOutput(self, GTPath):
        gtcheck = GTCheck(GTPath)
        return gtcheck.runTests()


class OT_Provider(GT_Provider):
    def __init__(self,server,ot_server,ot_client,mounted_dir,hmd,ctrl1=None,ctrl2=None):
        super(OT_Provider, self).__init__(server,ot_server,ot_client,mounted_dir,hmd,ctrl1,ctrl2)

    def init(self):
        response = requestController(
            self.url + '/OTInit/' + self.hmd.rigid_body + '/' + self.ot_server + '/' + self.ot_client,
            'Init server')
        print("response: {}".format(response))
        result_dir = response.read().strip(b'"')
        print("result_dir: {}".format(result_dir))
        self.result_dir = result_dir.decode("utf-8")
        self.mounted_dir = self.mounted_dir + self.result_dir +"/Optitrack"
        response = requestController(self.url + '/OTGetWorkingDirectory',
                          'get server working directory')
        self.server_dir = response.read().strip(b'"').decode("utf-8")
        print("server_dir:",self.server_dir)

    def start(self):
        requestController(self.url+ '/OTStart',
                          'Start optitrack recording')

    def stop(self):
        requestController(self.url + '/OTStop',
                          'Stop optitrack recording')

    def shutdown(self):
        requestController(self.url + '/OTShutdown',
                          'Shutdown server')

    def temporalAlignment(self,outputName,obj_name):
        TA_params = "-io " + self.server_dir + " -gt " + getattr(self, obj_name).rigid_body + " -ta "+outputName +",6dof,"+ obj_name + " -interp " +outputName + ",6dof,"+ obj_name +" -cdc 0 -o true"
        if self.post_process_dtype == "gyro":
            TA_params = TA_params.replace('6dof','gyro')
        if getattr(self, obj_name).type == 'hmd':
            TA_params = TA_params.replace(" -o true"," -o false")
        requestController(
            self.url + "/OTTemporalAlignment/"+urllib.parse.quote(TA_params),'Temporal Alignment')


    def projection(self, outputName,obj_name):
        projection_params = "-folder " + self.server_dir +" -head " + self.hmd.rigid_body +" -o "+getattr(self, obj_name).projection_dir_name+" -src "+outputName+" -ta "+getattr(self, obj_name).TA_dir_name+" -"+getattr(self, obj_name).type
        requestController(
            self.url + "/OTProjection/"+urllib.parse.quote(projection_params),'Projection')


class VR_Provider(GT_Provider):
    def __init__(self,server,ot_server,ot_client,mounted_dir,hmd,ctrl1=None,ctrl2=None):
        super(VR_Provider, self).__init__(server,ot_server,ot_client,mounted_dir,hmd,ctrl1,ctrl2)
    def init(self):
        response = requestController(
            self.url + '/VRInit/edebby-perc/ger/lab_lablabomek/trio_013/files',
            'Init server')
        result_dir = response.read().strip(b'"')
        print("result_dir: {}".format(result_dir))
        self.result_dir = result_dir.decode("utf-8")
        self.mounted_dir = self.mounted_dir + self.result_dir+'/VR'
        response = requestController(self.url + '/VRGetWorkingDirectory',
                          'get server working directory')
        self.server_dir = response.read().strip(b'"').decode("utf-8")

    def start(self):
        requestController(self.url + '/VRStart',
                          'Start optitrack recording')

    def stop(self):
        requestController(self.url + '/VRStop',
                          'Stop optitrack recording')

    def shutdown(self):
        requestController(self.url + '/VRShutdown',
                          'Shutdown server')

    def temporalAlignment(self,outputName,obj_name):
        TA_params = "-io " + self.server_dir + " -gt " + getattr(self, obj_name).rigid_body + " -ta "+outputName +",6dof," + obj_name +" -interp " +outputName + ",6dof,"+ obj_name +" -cdc 0 -o true"
        if getattr(self, obj_name).type == 'hmd':
            TA_params = TA_params.replace(" -o true"," -o false")
        if self.post_process_dtype == "gyro":
            TA_params = TA_params.replace('6dof','gyro')
        requestController(
            self.url + "/VRTemporalAlignment/"+ urllib.parse.quote(TA_params),'Temporal Alignment')

    def projection(self,outputName,obj_name):
        projection_params = "-folder " + self.server_dir+" -head " + self.hmd.rigid_body +" -o "+getattr(self, obj_name).projection_dir_name+" -src "+outputName+" -ta " +getattr(self, obj_name).TA_dir_name+" -"+obj_name

        requestController(
            self.url + "/VRProjection/"+urllib.parse.quote(projection_params),'Projection')

