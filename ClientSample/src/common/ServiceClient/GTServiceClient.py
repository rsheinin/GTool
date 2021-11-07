import urllib.request
import urllib.parse


class GTServiceClient:
    def __init__(self, ip_address):
        self.ip_address = ip_address
        self.port = "8080"
        self.service_name = "GTService"
        self.url = f'http://{self.ip_address}:{self.port}/{self.service_name}'

    def request(self, api, *args, log=True):
        url = '/'.join([self.url, api] + list(args))
        if log:
            print(f'{api}: {url}')
        # todo: check if can do only once
        proxy_support = urllib.request.ProxyHandler({})
        opener = urllib.request.build_opener(proxy_support)
        urllib.request.install_opener(opener)
        # end todo
        response = urllib.request.urlopen(url)

        if log:
            print(f'{api} response msg:', response.msg)
        return response
