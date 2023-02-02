from dataclasses import dataclass

@dataclass
class Store:
    id: str
    storeName: str = ''
    storeNumber: str = ''
    addressCountry: str = ''
    addressState: str = ''
    addressCity: str = ''
    addressPostcode: str = ''
    addressStreet: str = ''
    addressStreetNumber: str = ''
    addressAdditional: str = ''
    longitude: float = 0.0
    latitude: float = 0.0
    cadPlanId: str = ''
    externalReferenceId: str = None

    @property
    def name(self):
        return f'{self.id}_{self.storeName}'