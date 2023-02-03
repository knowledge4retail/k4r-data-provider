#!/usr/bin/env python3

# PYTHON_ARGCOMPLETE_OK

"""K4R Data Provider command line tool."""

import os
import json
import dataclasses
import argparse
import argcomplete
import logging
from pathlib import Path
from .data_provider import get_data_provider_class, DTAPIDataProvider
from .dtapi_con import DTApiCon

CONFIG_FORMATTER = '%(asctime)s %(name)s[%(levelname)s] %(message)s'
logger = logging.getLogger(__name__)


CMDS = {
    'get_stores': {'param': [], 'depends': []},
    'knows_store': {'param': ['store_name'], 'depends': ['get_stores']},
    'get_shelves': {'param': ['store_name'], 'depends': ['get_stores']},
    'get_products': {'param': ['store_name'], 'depends': ['get_stores']},
    'get_product_data': {'param': ['store_name'], 'depends': ['get_stores']},
    'get_product_shelves': {'param': ['store_name'], 'depends': ['get_stores']}
}


def setup_logging():
    log_level = os.environ.get('LOG_LEVEL', 'INFO')
    log_level = getattr(logging, log_level)
    logging.basicConfig(level=log_level, format=CONFIG_FORMATTER)


def main():
    setup_logging()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('cmd', help='Command to execute', choices=CMDS.keys())
    parser.add_argument(
        '--data_source', help='Data Source.', choices=[
            'files', 'dtapi', 'knowrob'], default='dtapi')
    # configuration for DT-API
    parser.add_argument(
        '--certificate_path', help='Certificate path (dtapi only).', 
        default='certificate')
    parser.add_argument(
        '--pks12_pw', help='Certificate password or path to password file (dtapi only).', 
        default='')
    parser.add_argument(
        '--dtapi_uri', help='URI to dtapi-endpoint.', 
        default='https://dt-api.sandbox.knowledge4retail.org/k4r/')
    parser.add_argument('--store_name',
        help='name of the store, normally the store id, an underscore '
            'and the map name', default='')
    parser.add_argument('--gtin', help='GTIN of the product', default='')
    parser.add_argument('--data-path', help='path to input data', default=None)
    argcomplete.autocomplete(parser)

    args, unknown = parser.parse_known_args()
    if not args.data_source:
        parser.print_help()
        return
    clz = get_data_provider_class(args.data_source)
    if not clz:
        parser.print_help()
        return
    if not hasattr(clz, args.cmd):
        logger.error('unknown command')
        return
    con = None
    if clz == DTAPIDataProvider:
        if not Path(args.certificate_path).exists():
            logger.error('Certificate path does not exist')
            return
        con = DTApiCon(
            pks12_pw=args.pks12_pw,
            certificate_path=args.certificate_path,
            dtapi_uri=args.dtapi_uri)
    inst = clz(con)
    # execute command
    for depend in CMDS[args.cmd]['depends']:
        getattr(inst, depend)()
    params = [(name, getattr(args, name)) for name in CMDS[args.cmd]['param']]
    result = getattr(inst, args.cmd)(**dict(params))
    if dataclasses.is_dataclass(result):
        print(json.dumps(dataclasses.as_dict(dict_result), indent=2))
    else:
        print(json.dumps(result))



if __name__ == '__main__':
    main()