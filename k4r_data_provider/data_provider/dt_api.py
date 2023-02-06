from ..model import DataProvider, Store, Shelf, Product
from ..dtapi_con import DTApiCon
try:
    import rospy
    loginfo = rospy.loginfo
except:
    def loginfo(msg):
        print(msg)
import math
from ..utils import quaternion_to_euler


GTINS_SHELVES = {
    # HiPP Gesichttücher
    # 4. sloped row from right, 2nd shelf from the top
    '0000040623474': {
        'shelf': 'w205255822293145',
        'name': 'HiPP Gesichttücher'
    }, 

    # Domestos Kraft Universal-Reiniger
    # 3. row from top, 2nd shelf from right
    '4000125077310': {
        'shelf': 'w20525582154278',
        'name': 'Domestos Kraft Universal-Reiniger'
    },

    # hydro nature alverde naturkosmetik Waschgel men
    # 4. sloped row from the left, 4th shelf from the bottom
    '4010355348388': {
        'shelf': 'w205255822838190',
        'name': 'hydro nature alverde naturkosmetik Waschgel men'
    },

    # Somat10 Tabs Extra All in 1, 25 St
    # 1st row from the bottom, 1st from the right
    #'4015000962940': ['w205255822968201'],
    #'4015000962940':  ['w20525582141867'],
    '4015000962940': {
        'shelf': 'w20525582098729',
        'name': 'Somat10 Tabs Extra All in 1, 25 St'
    },
    # Blend-a-med Kräuter Zahnpasta 75ml
    # 4th row from the top, 4th from the right
    '8001090272355': {
        'shelf': 'w20525582144869',
        'name': 'Blend-a-med Kräuter Zahnpasta 75ml'
    }
}
GTINS = GTINS_SHELVES.keys()


def convert_unit(unit, depth, width, height):
    mul = 1
    if not unit:
        loginfo(f'WARNING: Unit type undefined!')
        return depth * mul, width * mul, height * mul
    if unit['name'].lower() == 'meter':
        mul = 1.0
    elif unit['name'].lower() in ['milimeter', 'millimeter']:
        mul = 0.001
    elif unit['name'].lower() in [
            'centimetre', 'centimeter', 'zentimeter']:
        mul = 0.1
    # try symbols after all names fail
    elif unit['symbol'].lower() == 'm':
        loginfo(
            f'WARNING: Assuming meter for symbol {unit["symbol"]}')
        mul = 1.0
    elif unit['symbol'].lower() == 'mmt':
        loginfo(
            f'WARNING: Assuming millimeter for symbol {unit["symbol"]}')
        mul = 0.001
    elif unit['symbol'].lower() == 'cmt':
        loginfo(
            f'WARNING: Assuming centimeter for symbol {unit["symbol"]}')
        mul = 0.1
    else:
        # we stick to 1.0 if the type is unknown
        loginfo(f'WARNING: Unhandled unit type {unit["name"]}')
    return depth * mul, width * mul, height * mul


def parse_shelves(_shelf_data, unit_cache=None):
    shelves = []
    if not unit_cache:
        unit_cache = []
    
    for shelf_data in _shelf_data:
        depth = shelf_data['depth']
        width = shelf_data['width']
        height = shelf_data['height']
        if 'lengthUnitId' in shelf_data and shelf_data['lengthUnitId']:
            for unit in unit_cache:
                if unit['id'] == shelf_data['lengthUnitId']:
                    depth, width, height = convert_unit(
                        unit, depth, width, height)
        # we assume a shelf is not higher than 5 meters, so if the unit 
        # is labeld incorrectly a value of 500 for height should be a
        # mistake and we assume dimensions in millimeter!
        if height > 500:
            width *= 0.001
            depth *= 0.001
            height *= 0.001

        orientation = [
            shelf_data['orientationX'],
            shelf_data['orientationY'],
            shelf_data['orientationZ'],
            shelf_data['orientationW']
        ]

        size = [width, depth, height]

        shelf_center_z = shelf_data['positionZ']
        if -0.02 < shelf_center_z < 0.02:
            # see BottomLeftPositionStrategy.ts
            # euler = roll, pitch, yaw in radians
            euler = quaternion_to_euler(*orientation)

            # angle = Euler yaw in radians (NOT shifted by Pi)
            angle = euler[2] # + (math.pi / 2)

            dist_x, dist_y = rotate_offset(size, angle)

            dist_z = size[2] / 2
            # we only add the additional dimensions if the shelf
            # is centered around 0 so it does not go below the ground
            position = [
                shelf_data['positionX'] - dist_x,
                shelf_data['positionY'] - dist_y,
                shelf_data['positionZ'] + dist_z
            ]
        else:
            position = [
                shelf_data['positionX'],
                shelf_data['positionY'],
                shelf_data['positionZ']
            ]


        shelf_id = shelf_data['externalReferenceId']
        # rosplan does not allow numbers, make sure the id is a string
        # by adding a "w" infront
        shelf_id = f'w{shelf_id}'
        shelf = Shelf(
            shelf_id=shelf_id, dtapi_id=shelf_data['id'], 
            model='', uri=f'dt-api#{shelf_id}',
            position=position, orientation=orientation, 
            width=width, height=height, depth=depth,
            quaternion=True, radians=False)
        shelf.guess_model()
        shelves.append(shelf)
    return shelves


# math util functions
# see k4r-storeviz/src/client/util/MathUtil.ts
def rotate_offset(size, angle):
    dist_x = - size[0] / 2 * math.cos(angle) + size[1] / 2 * math.sin(angle)
    dist_y = - size[0] / 2 * math.sin(angle) - size[1] / 2 * math.cos(angle)
    return dist_x, dist_y


class DTAPIDataProvider(DataProvider):
    def __init__(self, con=None):
        super().__init__()
        self.name = 'DT-API'
        self.con = con if con else DTApiCon()
        self.unit_cache = self.con.get_units()

    def get_units(self) -> list:
        return self.unit_cache

    def get_gtins(self) -> list:
        return self.con.all_gtins()

    def get_items(self) -> list:
        return self.con.get_items()

    def get_stores(self) -> list:
        self.stores = [Store(**s) for s in self.con.get_stores()]
        return self.stores

    def knows_store(self, store_name):
        return super().knows_store(store_name) if '_' in store_name else False

    def get_shelves(self, store_name: str, store_id: int, max_shelves=100) -> list:
        _shelf_data = self.con.get_shelves(store_id)

        shelves = parse_shelves(_shelf_data, self.unit_cache)
        
        self.shelves[f'{store_id}_{store_name}'] = shelves
        return shelves

    def get_materialgroups(self) -> list:
        return self.con.get_materialgroups()

    def get_products(self, store_name, store_id) -> list:
        # self.con.get_products() would get all products from all 
        # stores but we just return dummy data for testing!!
        self.get_dummy_products(store_name, store_id)

    def get_dummy_products(self, store_name, store_id):
        for gtin in GTINS:
           products.append(self.get_product(store_name, store_id, gtin))
        return products

    def get_product_units(self) -> list:
        # get products from all stores
        return self.con.get_product_units()

    def get_product(self, store_name: str, store_id: int, gtin: str) -> Product:
        # We do not have the product position or orientation!
        return Product(**self.get_product_data(store_name, store_id, gtin))

    def get_product_data(self, store_name: str, store_id: int, gtin: str) -> dict:
        # TODO: request from server!
        raise NotImplementedError()
        # position = [0]*3
        # orientation = [0]*3+[1]
        # return {
        #     'gtin': gtin,
        #     'name': GTINS_SHELVES[gtin]['name'],
        #     'product_id': gtin,
        #     'position': position,
        #     'orientation': orientation,
        #     'shelf_ids': [GTINS_SHELVES[gtin]['shelf']],
        #     'quaternion': True,
        #     'radians': True
        # }

    def get_product_shelves(self, store_name: str, store_id: int, max_shelves: int=100) -> list:
        raise NotImplementedError()
        # return []
