from pathlib import Path
import os
import json
import time
import requests
from requests.exceptions import ConnectionError, ReadTimeout
from ..model import DataProvider, Store, Shelf, Product
from .files import FilesDataProvider, get_product_list

DT_API_DEFAULT = '848_Uni-Bremen'
DEFAULT_TIMEOUT = 20
USE_CACHE = True
ROS_AVAILABLE = True
try:
    import rospy
except ModuleNotFoundError:
    ROS_AVAILABLE = False



class KnowrobDataProvider(DataProvider):
    def __init__(
            self, con=None,
            store_name=DT_API_DEFAULT,
            use_cache=USE_CACHE, timeout=DEFAULT_TIMEOUT,
            retry=True, max_queries=1000):
        self.con = con
        self.store_name = store_name
        self.name = 'Knowrob'
        if use_cache:
            self.cache = FilesDataProvider()
        else:
            self.cache = None
        self.stores = [Store(id='848', storeName=self.store_name)]
        self.timeout = timeout
        self.retry = True
        self.max_queries = max_queries

    def get_stores(self) -> list:
        return self.stores

    def get_units(self) -> dict:
        return [{
            'id': 1,
            'name': 'meter',
            'type': 'length',
            'symbol': 'm'
        }]

    def get_shelves(self, store_name: str, max_shelves=100):
        if self.cache:
            data = self.cache.get_shelves(gtin)
            if data:
                return data
            # else: not found in cache
        knowrob_data = query_knowrob(
                "findall([Shelf, FilePath, Pose, Depth, Width, Height], "
                "(has_type(Shelf, dmshop:'DMShelfFrame'),"
                " triple(Shelf, soma:hasShape, Shape),"
                " triple(Shape, dul:hasRegion, ShapeRegion),"
                " triple(ShapeRegion, soma:hasFilePath, FilePath),"
                " is_at(Shelf, Pose),"
                " object_dimensions(Shelf, Depth, Width, Height)"
                "), Shelves).",
                max_shelves,
                timeout=self.timeout,
                retry=self.retry)
        if not knowrob_data:
            return []
        position, orientation = shelf[2][1:3]
        if '#' in shelf[0]:
            shelf_id = shelf[0].split('#')[1].lower()
        else:
            shelf_id = shelf[0]
        return [
            Shelf(
                shelf_id=shelf_id,
                model=shelf[1],
                position=position,
                orientation=orientation,
                depth=shelf[3],
                width=shelf[4],
                height=shelf[5],
                quaternion=True,
                radians=True)
            for shelf in knowrob_data['response'][0]['Shelves']]

    def get_products(self, store_name: str) -> list:
        product_list = get_product_list()
        self.product_list = {}
        for prod in product_list:
            self.product_list[prod['gtin']] = prod['name']
        return [
            self.get_product(prod['gtin'])
            for prod in product_list]

    def get_product(self, store_name: str, gtin: str) -> Product:
        if self.cache:
            prod_data = self.cache.get_product_data(gtin, store_name)
            if prod_data:
                return prod_data
        prod_data = self.get_product_data(gtin, store_name)
        name = 'unknown'
        if gtin in self.product_list:
            name = self.product_list[gtin]
        position = product_data["FacingPose"][1]
        orientation = product_data["FacingPose"][2]
        return Product(
            gtin=gtin,
            product_id=product_data['Product'],
            # TODO: what about the frame pose?
            position=position,
            orientation=orientation,
            shelf_ids=[product_data['TFFrame'].lower()],
            quaternion=True,
            radians=True)

    def get_product_data(self, store_name: str, gtin: str) -> dict:
        if self.cache:
            data = self.cache.get_product_data(gtin, store_name)
            if data:
                return data
            # else: not found in cache
        resp = query_knowrob(
            f'product_to_gtin(Product,\'{gtin}\'), '\
            'shelf_facing_product_type(Facing,Product), '\
            'shelf_facing(Layer,Facing), '\
            'shelf_layer_frame(Layer,Frame), '\
            'holds(Frame, knowrob:frameName, TFFrame), '\
            'is_at(Frame, FramePose), '\
            'is_at(Facing, FacingPose), !.', 
            max_solution_count=1,
            timeout=self.timeout, 
            retry=self.retry)
        return resp['response'][0]

    def get_product_shelves(self, store_name: str, max_shelves: int=100) -> list:
        # TODO?
        raise NotImplementedError()
        # return []


def query_knowrob(
        query, max_solution_count=100, base_url=None, ros=True, 
        timeout=DEFAULT_TIMEOUT, retry=True):
    if ros and ROS_AVAILABLE:
        sleep = rospy.sleep
        loginfo = rospy.loginfo
        logerr = rospy.logerr
    else:
        sleep = time.sleep
        loginfo = print
        logerr = print
    
    loginfo(query)
    if base_url:
        url = base_url
    else:
        if 'KNOWROB_URI' in os.environ:
            url = os.environ['KNOWROB_URI']
        else:
            url = 'http://localhost:62226'
    url += '/knowrob/api/v1.0/query'
    data = {
        "query": query,
        "maxSolutionCount": max_solution_count
    }
    # print(json.dumps(data))
    success = False
    while not success:
        try:
            # loginfo(f'[knowrob] send query: {url}: {json.dumps(data)}')
            r = requests.post(url, json=data, timeout=timeout)
            # loginfo(f'[knowrob] response: {r.text}')
            success = True
            response = json.loads(r.text)
            if 'message' in data and data['message'] == 'Bad query':
                logerr(
                    '[knowrob] ERROR: Bad request to knowrob:'
                    f'{json.dumps(data)}')
            return response
        except ConnectionError:
            if retry:
                loginfo(
                    '[knowrob] Can not connect to knowrob, try again in 10 s.')
                sleep(10)
            else:
                logerr('[knowrob] Can not connect to knowrob. Aboring!')
                return
        except ReadTimeout:
            if retry:
                loginfo(
                    '[knowrob] Timeout during connection, will try again.')
            else:
                logerr('[knowrob] Timeout during connection. Aboring!')
                return