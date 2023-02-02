from .files import FilesDataProvider
from .knowrob import KnowrobDataProvider
from .dt_api import DTAPIDataProvider
from ..model import DataProvider


def get_data_provider_class(name: str) -> DataProvider:
    if name in ['static', 'files', 'cache']:
        return FilesDataProvider
    elif name == 'knowrob':
        return KnowrobDataProvider
    elif name in ['dt_api', 'dtapi', 'dt', 'digital_twin']:
        return DTAPIDataProvider
    else:
        raise NotImplementedError(f'Unknown data provider: {name}')