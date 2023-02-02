from .product import Product

class DataProvider:
    def __init__(self, con=None):
        self.con = con
        # list of all known stores
        self.stores = []
        # products by store-id
        self.products = {}
        # shelves by store-id
        self.shelves = {}

    def get_stores(self) -> list:
        raise NotImplementedError()

    def knows_store(self, store_name: str) -> bool:
        return store_name in [s.name for s in self.stores]

    def get_shelves(self, map_name: str) -> list:
        raise NotImplementedError()

    def get_products(self, map_name: str) -> list:
        raise NotImplementedError()

    def get_product(self, map_name: str, gtin: str) -> Product:
        raise NotImplementedError()

    def get_product_data(self, map_name: str, gtin: str) -> dict:
        raise NotImplementedError()

    def get_product_shelves(self, map_name: str, max_shelves: int=100) -> list:
        raise NotImplementedError()
