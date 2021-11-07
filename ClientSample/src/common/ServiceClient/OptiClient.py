from ServiceClient.GTServiceClient import GTServiceClient


class OptiClient(GTServiceClient):
    def __init__(self, ip_address, opti_server, opti_client, log=True):
        super(OptiClient, self).__init__(ip_address)

        self.opti_server = opti_server
        self.opti_client = opti_client

        self.log = log

    def request_init(self, rigid_body):
        response = self.request('OTinit', rigid_body, self.opti_server, self.opti_client)

        result_dir = response.read().strip(b'"').decode("utf-8")
        print(f'result_dir: {result_dir}')

        return result_dir

        # self.result_dir = result_dir.decode("utf-8")
        # self.mounted_dir = f'{self.mounted_dir}{self.result_dir}/Optitrack'

    def request_start(self):
        self.request('OTStart', log=self.log)

    def request_stop(self):
        self.request('OTStop', log=self.log)

    def request_shutdown(self):
        self.request('OTShutdown', log=self.log)

    def request_get_working_directory(self):
        response = self.request('OTGetWorkingDirectory', log=self.log)
        # self.server_dir = response.read().strip(b'"').decode("utf-8")
        # print(f'server_dir:{self.server_dir}')

    def request_is_in_range(self):
        return self.request('OTIsInRange', log=False)



