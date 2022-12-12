import os
# communication with dtapi, use request_pkcs12 for development and
# normal requests inside the kubernetes cloud
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
# DTAPI_URI = 'https://dt-api.sandbox.knowledge4retail.org/k4r/'
DTAPI_URI = 'http://k4r-dt-api:8090/k4r/'
DTAPI_USE_CERTIFICATE = True
CERTIFICATE_PATH = BASE_PATH / 'certificate'
CERTIFICATE_FILENAME = 'sandbox-2022.p12'

PKS12_PASSWORD = 'SECRET'

DTAPI_BASE = 'api/v0/'
# list all stores (will create maps from it)
DTAPI_STORES = f'{DTAPI_BASE}stores'
DTAPI_MAP2D = f'{DTAPI_STORES}STORE_ID/map2d'
DTAPI_DEVICES = f'{DTAPI_BASE}devices'
DTAPI_SHELVES = f'{DTAPI_BASE}stores/STORE_ID/shelves'
DTAPI_SHELF_LAYER = f'{DTAPI_BASE}shelves/SHELF_ID/shelflayers'
DTAPI_ALL_GTIN = f'{DTAPI_BASE}productgtins'
DTAPI_UNITS = f'{DTAPI_BASE}units'
DTAPI_SL_BARCODES = f'{DTAPI_BASE}shelflayers/SHELF_LAYER_ID/barcodes'
DTAPI_BARCODES = f'{DTAPI_BASE}barcodes/BARCODE_ID'
DTAPI_TROLLEYS = f'{DTAPI_BASE}stores/STORE_ID/trolleys'
DTAPI_TROLLEY_ROUTES = f'{DTAPI_BASE}trolleys/TROLLEY_ID/trolleyroutes'
DTAPI_DELIVERED_UNITS = f'{DTAPI_BASE}deliveredunits'



class DTApiCon:
    def __init__(self, pks12_pw=None, certificate_path=None, dtapi_uri=None):
        self.dtapi_uri = dtapi_uri if dtapi_uri else DTAPI_URI 
        self.certificate_path = certificate_path if certificate_path else \
            CERTIFICATE_PATH / CERTIFICATE_FILENAME

        if ROS_AVAILABLE:
            self.from_ros()
        else:
            self.use_cert = True
            self.pks12_pw = "HP$q!$P8tphiL5UYsA*8tn4US!*Mjz" # pks12_pw
        
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

    def get_shelves(self, store_id: int) -> list:
        return self.get(DTAPI_SHELVES.replace('STORE_ID', str(store_id)))

    def get_shelf_layer(self, shelf_id: int) -> list:
        return self.get(DTAPI_SHELF_LAYER.replace('SHELF_ID', str(shelf_id)))

    def get_trolleys(self, store_id: int) -> list:
        return self.get(DTAPI_TROLLEYS.replace('STORE_ID', str(store_id)))

    def get_trolley_routes(self, trolley_id: int) -> list:
        return self.get(
            DTAPI_TROLLEY_ROUTES.replace('TROLLEY_ID', str(trolley_id)))

    def get_delivered_units(self) -> list:
        # FIXME might be an empty list?
        return self.get(DTAPI_DELIVERED_UNITS)

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
        rospy.loginfo(r.text)
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

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Access DT-API.')
    parser.add_argument(
        '--pks12_pw', type=str, help='PKS12-Password token.', default=None,
        required=False)
    parser.add_argument(
        '--certificate_path', type=str, required=False,
        help='absolute path to Certificate', default=None)
    parser.add_argument(
        '--dtapi_uri', type=str, help='URI to API-endpoint.', default=None,
        required=False)
    # Test to see if the API is working
    con = DTApiCon(**(parser.parse_args().__dict__))
    store_id = None
    for store in con.get_stores():
        print(store['storeName'], store['id'])
        if 'PPELE' in store['storeName']:
            store_id = store['id']
    #if not store_id:
    #    raise Exception('No WATER(FRONT) store found!')
    print(store_id)

    print(con.get_delivered_units())
    # trolleys = con.get_trolleys(store_id)
    # for trolley in trolleys:
    #     print(trolley)
    #     trolley_id = trolley['id']
    #     trolley_routes = con.get_trolley_routes(trolley_id)
    #     trolley_missions = dict_by_key(trolley_routes, 'sortingDate')
    #     for name, trolley_route in trolley_missions.items():
    #         trolley_missions[name] = sorted(trolley_route, 
    #             key=lambda tr: tr['routeOrder'])
    #     print(json.dumps(
    #         trolley_missions, indent=2))
    #     for route in trolley_routes:
    #         print(route)
    


    # for gtin in con.all_gtins():
    #     if gtin['gtin']:
    #         print(gtin)

    # for shelf in con.get_shelves(store_id):
    #     shelf_id = shelf['id']
    #     shelf_layers = con.get_shelf_layer(shelf_id)
    #     for shelf_layer in shelf_layers:
    #         barcodes = con.get_shelf_layer_barcodes(shelf_layer['id'])
    #         for barcode in barcodes:
    #             print(barcode['productGtinId'])

    #print(len(gtins))
    # existing_gtins = []
    # for gtin_entry in gtins:
    #     if 'gtin' in gtin_entry and gtin_entry['gtin']:
    #         existing_gtins.append(gtin_entry)

    # print(existing_gtins)
    #saved_gtins = [g.split('.json')[0] for g in 
    #    os.listdir(BASE_PATH / 'data' / 'products')]
    #for gtin in gtins:
    #    if gtin['gtin'] in saved_gtins:
    #        existing_gtins += [gtin]
    #print(len(existing_gtins))

def noop():
    stores = con.get_stores()
    print(f'Received {len(stores)} stores')
    selected_store = None
    for store in stores:
        shelves_txt = '?'
        if store["storeName"].startswith('HB-WATER'):
            shelves = con.get_shelves(store["id"])
            shelves_txt = str(len(shelves))
            if len(shelves) > 0:
                selected_store = store['id'], store["storeName"], shelves
        print(f' - {store["id"]}: {store["storeName"]} {shelves_txt}')

    if selected_store:
        store_id, store_name, shelves = selected_store
        print(
            f'Select store {store_name} ({store_id}) with '
            f'{len(shelves)} shelves.')
