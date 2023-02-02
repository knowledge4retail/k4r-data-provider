from pathlib import Path
import os
import json
from ..model import DataProvider, Store, Shelf, Product
from .dt_api import parse_shelves, GTINS, GTINS_SHELVES


BASE = Path(os.path.dirname(__file__))
DATA_PATH = BASE / 'data'
PRODUCT_PATH = 'products'  # folder
PRODUCT_LIST = 'product_list.json'
# We have 2 different sources from shelves, either knowrob or dtapi.
# The data is structured differently!
SHELVES_KNOWROB_PATH = 'shelves_knowrob.json'
SHELVES_DTAPI_PATH = 'shelves_dtapi.json'


def store_path(name):
    if '_' in name:
        name = name.split('_')[1]
    return DATA_PATH / name


class FilesDataProvider(DataProvider):
    def __init__(self, con=None):
        super().__init__(con)
        self.name = 'files'

    def get_stores(self) -> list:
        self.stores = [
            Store(id=_id, storeName=name) 
            for _id, name in enumerate(os.listdir(DATA_PATH))]
        return self.stores

    def get_units(self) -> dict:
        return [{
            'id': 1,
            'name': 'meter',
            'type': 'length',
            'symbol': 'm'
        }]

    def get_shelves(self, store_name: str) -> list:
        # get shelves from static knowrob entry
        shelves = []
        path = store_path(store_name)
        shelves_path_kr = path / SHELVES_KNOWROB_PATH
        shelves_path_dt = path / SHELVES_DTAPI_PATH
        if os.path.exists(shelves_path_kr):
            shelves_file = open(shelves_path_kr, 'r')
            shelves = json.load(shelves_file)
            if '#' in shelf[0]:
                shelf_id = shelf[0].split('#')[1].lower()
            else:
                shelf_id = shelf[0]
            return [
                Shelf(
                    uri=shelf[0],
                    shelf_id=shelf_id,
                    model=shelf[1],
                    position=shelf[2][1],
                    orientation=shelf[2][2],
                    depth=shelf[3],
                    width=shelf[4],
                    height=shelf[5],
                    quaternion=True,
                    radians=True)
                for shelf in shelves]
        elif os.path.exists(shelves_path_dt):
            shelves_file = open(shelves_path_dt, 'r')
            shelves = json.load(shelves_file)
            return parse_shelves(shelves)
        else:
            print(f'{shelves_path_dt} nor {shelves_path_kr} exist')
            return []

    def get_products(self, store_name: str) -> list:
        product_list = get_product_list(store_name)
        if not product_list:
            return []
        self.product_list = {}
        for prod in product_list:
            self.product_list[prod['gtin']] = prod['name']
        return [
            self.get_product(store_name, prod['gtin'])
            for prod in product_list]

    def get_product(self, store_name: str, gtin: str) -> Product:
        product_data = self.get_product_data(store_name, gtin)
        if not product_data:
            print('Warning: product data not set')
            return None
        name = 'unknown'
        if gtin in self.product_list:
            name = self.product_list[gtin]
        # TODO: what about the frame pose?
        position, orientation = product_data["FacingPose"][1:3]
        return Product(
            gtin=gtin,
            product_id=product_data['Product'],
            name=name,
            shelf_ids=[GTINS_SHELVES[gtin]['shelf']],
            position=position,
            orientation=orientation,
            quaternion = True,
            radians=True)

    def get_product_data(self, store_name: str, gtin: str) -> dict:
        path = store_path(store_name)
        product_path = path / PRODUCT_PATH / f'{gtin}.json'
        data = {}
        if os.path.exists(product_path):
            product_file = open(product_path, 'r')
            data = json.load(product_file)[0]
        else:
            data = {
                "Facing": f"#{gtin}",
                "FacingPose": [
                    "unknown",
                    [
                        0,
                        0,
                        0
                    ],
                    [
                        0.0,
                        0.0,
                        0.0,
                        1.0
                    ]
                ],
                "Frame": f"http://knowrob.org/kb/#unknown_{gtin}",
                "FramePose": [
                    "map",
                    [
                        0,
                        0,
                        0.0
                    ],
                    [
                        0.0,
                        0.0,
                        0.0,
                        0.0
                    ]
                ],
                "Layer": f"http://knowrob.org/kb/#unknown_{gtin}",
                "Product": f"#http://knowrob.org/kb/shop.owl#unknown_{gtin}",
                # shelf_id
                "TFFrame": f"unknown_{gtin}"
            }
            print(f'file {product_path} does not exist.')
        data['gtin'] = gtin
        return data

    def get_product_shelves(self, store_name: str, max_shelves: int=100) -> list:
        # TODO!
        # raise NotImplementedError()
        return []

def get_product_list(store_name):
    path = store_path(store_name)
    if (path / PRODUCT_LIST).exists():
        print(f'Try to load {store_name}')
        fp = open(path / PRODUCT_LIST, 'r')
        return json.load(fp)
    else:
        print(
            f'Could not load {store_name}, file does not exist')
        return None

def store_cache(filename, store_name, data):
    path = store_path(store_name)
    fp = open(path / filename, 'w')
    json.dump(data, fp, indent=2)
    fp.close()