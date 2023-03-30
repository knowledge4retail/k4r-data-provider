# communication with dtapi, use request_pkcs12 for development and
# normal requests inside the kubernetes cloud

import os
import requests
import json
from pathlib import Path
try:
    from requests_pkcs12 import get, post, put, delete
    HAS_PKCS12 = True
except:
    HAS_PKCS12 = False
try:
    import rospy
    ROS_AVAILABLE = True
except:
    # local testing without ROS!
    ROS_AVAILABLE = False

BASE_PATH = Path(os.path.dirname(__file__)).parent.parent
# DT-API configuration
DTAPI_URI = 'https://dt-api.sandbox.knowledge4retail.org/k4r/'
# DTAPI_URI = 'http://k4r-dt-api:8090/k4r/'
DTAPI_USE_CERTIFICATE = True
CERTIFICATE_PATH = BASE_PATH / 'certificate'
CERTIFICATE_FILENAME = 'sandbox-2022.p12'

PKS12_PASSWORD = 'SECRET'

DTAPI_BASE = 'api/v0/'
# list all stores (will create maps from it)
DTAPI_STORES = f'{DTAPI_BASE}stores'
DTAPI_MAP2D = f'{DTAPI_STORES}STORE_ID/map2d'
DTAPI_DEVICES = f'{DTAPI_BASE}devices'
DTAPI_ITEMS = f'{DTAPI_BASE}items'
DTAPI_PRODUCTS = f'{DTAPI_BASE}products'
DTAPI_PRODUCT_UNITS = f'{DTAPI_BASE}productunits'
DTAPI_MATERIALGROUPS = f'{DTAPI_BASE}materialgroups'
DTAPI_SHELVES = f'{DTAPI_BASE}stores/STORE_ID/shelves'
DTAPI_SHELF_LAYER = f'{DTAPI_BASE}shelves/SHELF_ID/shelflayers'
DTAPI_ALL_GTIN = f'{DTAPI_BASE}productgtins'
DTAPI_UNITS = f'{DTAPI_BASE}units'
DTAPI_SL_BARCODES = f'{DTAPI_BASE}shelflayers/SHELF_LAYER_ID/barcodes'
DTAPI_BARCODES = f'{DTAPI_BASE}barcodes/BARCODE_ID'
DTAPI_TROLLEYS = f'{DTAPI_BASE}stores/STORE_ID/trolleys'
DTAPI_TROLLEY_ROUTES = f'{DTAPI_BASE}trolleys/TROLLEY_ID/trolleyroutes'
DTAPI_DELIVERED_UNITS = f'{DTAPI_BASE}deliveredunits'
DTAPI_STORES_AGGREGATES = f'{DTAPI_STORES}/aggregates'


class DTApiCon:
    def __init__(self, pks12_pw=None, certificate_path=None, dtapi_uri=None):
        self.dtapi_uri = dtapi_uri if dtapi_uri else DTAPI_URI 
        self.certificate_path = certificate_path if certificate_path else \
            CERTIFICATE_PATH / CERTIFICATE_FILENAME

        if ROS_AVAILABLE:
            self.from_ros()
        else:
            self.use_cert = True
            if os.path.exists(pks12_pw):
                with open(pks12_pw, 'r') as f:
                    self.pks12_pw = f.read()
            else:
                self.pks12_pw = pks12_pw
        
        assert os.path.exists(self.certificate_path), \
            self.certificate_path


    def from_ros(self):
        # configuration from ros_params
        use_cert = rospy.get_param('~use_certificate', DTAPI_USE_CERTIFICATE)
        if isinstance(use_cert, bool):
            self.use_cert = use_cert
        elif use_cert == '':
            self.use_cert = False
        elif distutils.util.strtobool(use_cert):
            self.use_cert = True
        else:
            self.use_cert = False

        self.pks12_pw = rospy.get_param(
            '~pks12_pw', PKS12_PASSWORD
        )
        self.dtapi_uri = rospy.get_param(
            '~dtapi_uri', DTAPI_URI
        )
        rospy.loginfo('DTAPI-URI:')
        rospy.loginfo(self.dtapi_uri)

        certificate_path = rospy.get_param(
            '~certificate_path', CERTIFICATE_PATH)
        certificate_filename = rospy.get_param(
            '~certificate_filename', CERTIFICATE_FILENAME)
        if not certificate_filename:
            # no certificate given, assuring direct connection inside the
            # kubernetes platform
            return
        self.certificate_path = certificate_path / certificate_filename


    def get_units(self) -> dict:
        return self.get(DTAPI_UNITS)

    def get_stores(self) -> list:
        """
        Get list of all stores
        """
        return self.get(DTAPI_STORES)

    def get_shelves(self, store_id: int, store_name: str = None) -> list:
        return self.get(DTAPI_SHELVES.replace('STORE_ID', str(store_id)))

    def get_shelf_layer(self, shelf_id: int, store_name: str = None) -> list:
        return self.get(DTAPI_SHELF_LAYER.replace('SHELF_ID', str(shelf_id)))

    def get_trolleys(self, store_id: int, store_name: str = None) -> list:
        return self.get(DTAPI_TROLLEYS.replace('STORE_ID', str(store_id)))

    def get_trolley_routes(self, trolley_id: int, store_name: str = None) -> list:
        return self.get(
            DTAPI_TROLLEY_ROUTES.replace('TROLLEY_ID', str(trolley_id)))

    def get_materialgroups(self):
        return self.get(DTAPI_MATERIALGROUPS)

    def get_gtins(self):
        return self.all_gtins()

    def get_items(self) -> list:
        return self.get(DTAPI_ITEMS)

    def get_products(self) -> list:
        return self.get(DTAPI_PRODUCTS)

    def get_product_units(self) -> list:
        return self.get(DTAPI_PRODUCT_UNITS)

    def get_delivered_units(self) -> list:
        # WARNING: This might be an empty list, so we provide a list
        # based on static files in that case.
        delivered_units = self.get(DTAPI_DELIVERED_UNITS)
        if not delivered_units:
            print('WARNING: No delivered units')
        return delivered_units

    def all_stores_aggregates(self) -> list:
        return self.get(DTAPI_STORES_AGGREGATES)

    def all_gtins(self):
        return self.get(DTAPI_ALL_GTIN)

    def get_shelf_layer_barcodes(self, shelf_layer_id: int) -> list:
        return self.get(DTAPI_SL_BARCODES.replace(
            'SHELF_LAYER_ID', str(shelf_layer_id)))

    def get_bacode(self, barcode_id: int) -> object:
        return self.get(DTAPI_BARCODES.replace('BARCODE_ID', str(barcode_id)))

    ### wrapper around requests
    def check_okay(self, response: str) -> bool:
        if response.startswith('<html>') or \
                response.startswith('<!doctype html'):
            error_msg = response.split('<title>')[1].split('</title>')[0]
            raise RuntimeError(error_msg)
        try:
            json.loads(response)
        except json.JSONDecodeError as e:
            print(f'error parsing JSON: "{response}"')
            raise e
        return True

    def post(self, url: str, data: dict) -> dict:
        # create
        url = self.dtapi_uri + url
        if not HAS_PKCS12 or not self.use_cert or not self.certificate_path:
            r = requests.post(url, json=data, headers=HEADERS)
        else:
            r = post(url, json=data,
                headers=HEADERS,
                pkcs12_filename=self.certificate_path,
                pkcs12_password=self.pks12_pw)
        return json.loads(r.text)

    def get(self, url: str, data: dict = None) -> dict:
        # read
        headers = {'accept': '*/*'}
        if not data:
            data = {}
        url = self.dtapi_uri + url
        if ROS_AVAILABLE:
            rospy.loginfo(f'DTAPI-request: {url}')
        else:
            print(url)
        if not HAS_PKCS12 or not self.use_cert or not self.certificate_path:
            r = requests.get(url, data=data, headers=headers)
        else:
            r = get(url, data=data, headers=headers,
                pkcs12_filename=self.certificate_path,
                pkcs12_password=self.pks12_pw)
        self.check_okay(r.text)
        return json.loads(r.text)


    def put(self, url: str, data: dict) -> dict:
        # update
        url = self.dtapi_uri + url
        if not HAS_PKCS12 or not self.use_cert or not self.certificate_path:
            r = requests.put(url, data=data, headers=HEADERS)
        else:
            r = put(url, data=data, headers=HEADERS,
                pkcs12_filename=self.certificate_path,
                pkcs12_password=self.pks12_pw)
        # if ROS_AVAILABLE:
        #     rospy.loginfo(r.text)
        # else:
        #     print(r.text)
        return json.loads(r.text)

    def delete(self, url: str, data: dict) -> dict:
        # delete
        url = self.dtapi_uri + url
        if not HAS_PKCS12 or not self.use_cert or not self.certificate_path:
            r = requests.delete(url, data=data, headers=HEADERS)
        else:
            r = delete(url, data=data, headers=HEADERS,
                pkcs12_filename=self.certificate_path,
                pkcs12_password=self.pks12_pw)
        return json.loads(r.text)


def dict_by_key(lst: list, key: str) -> dict:
    """Generates a dictionary from a list by a key inside the list.

    Example: [
        {'key': 'a', 'value': 1},
        {'key': 'a', 'value': 2},
        {'key': 'b', 'value': 5}
    ]
    will return
    {
        'a': [{'key': 'a', 'value': 1}, {'key': 'a', 'value': 2}],
        'b': [{'key': 'b', 'value': 5}]
    }
    """
    dic = {}
    for item in lst:
        dic.setdefault(item[key], [])
        dic[item[key]].append(item)
    return dic
