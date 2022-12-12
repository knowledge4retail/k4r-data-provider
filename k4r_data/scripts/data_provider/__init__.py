from pathlib import Path
import os
import json
from .static import StaticDataProvider
from .knowrob import KnowrobDataProvider
from .dt_api import DTAPIDataProvider

def get_data_provider_class(name):
    if name == 'static':
        return StaticDataProvider
    elif name == 'knowrob':
        return KnowrobDataProvider
    elif name in ['dt_api', 'dtapi', 'dt', 'digital_twin']:
        return DTAPIDataProvider
    else:
        rospy.logerr(f'Unknown data provider: {name}')


if __name__ == "__main__":
    map_name = 'Uni-Bremen' # TEST DATA
    print(f'get all shelves for map {map_name}')
    static_prov = StaticDataProvider()
    knowrob_prov = KnowrobDataProvider(
        use_cache=False, timeout=30, retry=False)

    # fill cache with data from Knowrob
    shelves = knowrob_prov.get_shelves()['response'][0]['Shelves']
    static_prov.store_cache('shelves.json', shelves)
    print(f'done. Found {len(shelves)} shelves.')

    # print('get all product GTINs')
    # res = query_knowrob(
    #     'dan_gtin(DAN, GTIN)',
    #     max_solution_count=100000,
    #     timeout=30)
    # gtins = [r['GTIN'] for r in res['response']]
    # print(f'done. found {len(set(gtins))} facings, {len(gtins)} GTINS in total.')

    gtins = [
        '4000125077310',
        '4015000962940',
        '8001090272355',
        '0000040623474',
        '4010355348388',
    ]
    i = 0
    success = 0
    for filename in os.listdir(BASE / 'data' /  'products'):
        if filename.endswith('.json'):
            os.remove(BASE / 'data' / 'products' / filename)
    for gtin in gtins:
        i += 1
        print(f'Get GTIN {gtin} ({i}/{len(gtins)}).')
        products = []
        gtin_file = Path('products') / f'{gtin}.json'
        if os.path.exists(gtin_file):
            fp = open(gtin_file, 'r')
            products = json.load(fp)
            fp.close()
        product_position = knowrob.get_product_positions(gtin)
        if not product_position:
            continue
        products.append(product_position['response'][0])
        static_prov.store_cache(gtin_file, products)
        success += 1
        print(f'GTIN {gtin} done.')

    print(f'done! {success} of {len(gtins)} successful.')