from k4r_data_provider.data_provider import get_data_provider_class
from k4r_data_provider.data_provider.files import FilesDataProvider


def test_load_file_provider():
    pro = get_data_provider_class('files')
    assert pro == FilesDataProvider

def test_parse_data():
    # TODO: Load .json files for Uni-Bremen example into to model/-classes
    assert True