using System;
using System.ServiceModel;
using System.ServiceModel.Web;
using System.ServiceModel.Description;
using System.Xml;


namespace RecorderHost
{
    class Program
    {
        static void Main(string[] args)
        {
            int gtSrc = 0;
            //bool _useOptitrack = true;
            if (args.Length > 0 /*&& args[0].Equals("OT")*/)
            {
                gtSrc = int.Parse(args[0]);
                //_useOptitrack = false;
            }
            Uri baseAddress = new Uri("http://127.0.0.1:8080/GTService/");
            WebServiceHost webHost = new WebServiceHost(typeof(RecorderService.ServiceKpi), baseAddress);

            var binding = new WebHttpBinding();
            binding.MaxReceivedMessageSize = int.MaxValue;
            binding.MaxBufferSize = int.MaxValue;
            XmlDictionaryReaderQuotas rq = new XmlDictionaryReaderQuotas();
            rq.MaxArrayLength = 10000;
            rq.MaxDepth = 1000;
            rq.MaxStringContentLength = int.MaxValue;
            rq.MaxBytesPerRead = 2147483647;
            rq.MaxNameTableCharCount = 16384;
            binding.ReaderQuotas = rq;
            ServiceEndpoint ep = webHost.AddServiceEndpoint(typeof(RecorderService.IServiceContract), binding/*new WebHttpBinding()*/, "");
            ServiceDebugBehavior stp = webHost.Description.Behaviors.Find<ServiceDebugBehavior>();
            stp.HttpHelpPageEnabled = false;
            try
            {
                webHost.Open();
                Console.WriteLine(DateTime.Now.ToString());
                Console.WriteLine("GT Service is up and running");
                Console.WriteLine("Press enter to quit ");
                //SPLiveKpisService.ServiceKpi.useOptitrack = (gtSrc == 0) ? true : false;
                RecorderService.ServiceKpi.initReflection();
                Console.WriteLine("\n\r\n\rTracking system is: " + Enum.GetName(typeof(RecorderService.GTSource), gtSrc));//(SPLiveKpisService.ServiceKpi.useOptitrack ? "OPTITRACK" : "OPTOTRACK"));
                Console.WriteLine("\tChanging ground-truth source by specifing its # in cmd line args:\n\r");
                foreach (var item in Enum.GetValues(typeof(RecorderService.GTSource)))
                {
                    Console.WriteLine("\t#{0} : {1}", (int)item, Enum.GetName(typeof(RecorderService.GTSource), item));
                }

                string gt = Console.ReadLine();
                webHost.Close();
            }
            catch (Exception)
            {

                throw;
            }

        }
    }
}
