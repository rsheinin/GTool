using System;
using System.Runtime.Serialization;
using System.ServiceModel;
using System.ServiceModel.Web;

namespace RecorderService
{
    
    // NOTE: You can use the "Rename" command on the "Refactor" menu to change the interface name "IService1" in both code and config file together.
    [ServiceContract]
    public interface IServiceContract
    {
        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/OTInit/{rb_name}/{motive_serverIP}/{motive_clientIP}")]
        [OperationContract]
        string OTInit(string rb_name, string motive_serverIP, string motive_clientIP/*, string param4, string param5*/);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/VRInit/{host}/{domain}/{user}/{password}/{shared}")]
        [OperationContract]
        void VRInit(string host, string domain, string user, string password, string shared);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        string GetGHCString();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        void OTStart();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        bool VRStart();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        void OTStop();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        bool VRStop();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        bool OTShutdown();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        bool VRShutdown();

        //[WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/GTPostProcessing/{*TA_params}/{*proj_params}")]
        //[OperationContract]
        //bool GTPostProcessing(string TA_params, string proj_params);
        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/runPrediction/{file_name}/{input_foramt}/{time}")]
        [OperationContract]
        bool runPrediction(string file_name, string input_foramt, string time);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/OTTemporalAlignment/{*TA_params}")]
        [OperationContract]
        bool OTTemporalAlignment(string TA_params);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/VRTemporalAlignment/{*TA_params}")]
        [OperationContract]
        bool VRTemporalAlignment(string TA_params);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/OTProjection/{*proj_params}")]
        [OperationContract]
        bool OTProjection(string proj_params);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET", UriTemplate = "/VRProjection/{*proj_params}")]
        [OperationContract]
        bool VRProjection(string proj_params);

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        bool OTIsInRange();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        string OTGetWorkingDirectory();

        [WebInvoke(ResponseFormat = WebMessageFormat.Json, Method = "GET")]
        [OperationContract]
        string VRGetWorkingDirectory();

    }


    // Use a data contract as illustrated in the sample below to add composite types to service operations.
    [DataContract]
    public class CompositeType
    {

        bool boolValue = true;
        string stringValue = "Hello ";

        [DataMember]
        public bool BoolValue
        {
            get { return boolValue; }
            set { boolValue = value; }
        }

        [DataMember]
        public string StringValue
        {
            get { return stringValue; }
            set { stringValue = value; }
        }
    }

    //[DataContract]
    //public struct TranslationStruct { public float x; public float y; public float z;}

    public enum GTSource
    {
        Optitrack = 0,
        Optotrak = 1,
        Kinect = 2,
        ThermalCam = 3,
        VR = 4
    };

    public class GTSourceList
    {
        bool Optitrack = true;
        bool Optotrak = false;
        bool Kinect = false;
        bool ThermalCam = false;
        bool VR = true;        
    };

    public class GT
    {
        [DataMember]
        public string name;
        [DataMember]
        public string outputFolder;
    }

    public class VR : GT
    {
        public VR()
        {
            name = Enum.GetName(typeof(GTSource), GTSource.VR);
        }
        [DataMember]
        public string targetMachine;
        [DataMember]
        public string domain;
        [DataMember]
        public string user;
        [DataMember]
        public string password;
        [DataMember]
        public string shared;
    }
    public class OT : GT
    {
        public OT()
        {
            name = Enum.GetName(typeof(GTSource), GTSource.Optitrack);
        }
    }
    public struct GTData
    {
        [DataMember]
        public VR vr;
        [DataMember]
        public OT ot;
    }
}
