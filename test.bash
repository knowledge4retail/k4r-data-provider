#!/bin/bash
k4r_data_provider \
    get_stores \
    --data_source files
    
k4r_data_provider \
    knows_store \
    --store_name "Uni-Bremen" \
    --store_id 0 \
    --data_source files

k4r_data_provider \
    get_products \
    --store_name "Uni-Bremen" \
    --store_id 0 \
    --data_source files

k4r_data_provider \
    get_shelves \
    --store_name "Uni-Bremen" \
    --store_id 0 \
    --data_source files

k4r_data_provider \
    get_shelves \
    --store_name "Uni-Bremen" \
    --store_id 0 \
    --data_source files

echo "--- STORES ---"
k4r_data_provider \
    get_stores \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/stores.json

echo "--- UNITS ---"
k4r_data_provider \
    get_units \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/units.json

echo "--- SHELVES ---"
k4r_data_provider \
    get_shelves \
    --store_name 'HB-WATERFRON' \
    --store_id 5018 \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/shelves.json

echo "--- PRODUCTS ---"
k4r_data_provider \
    get_products \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/products.json

echo "--- PRODUCT_UNITS ---"
k4r_data_provider \
    get_product_units \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/product_units.json

echo "--- GTINS ---"
k4r_data_provider \
    get_gtins \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/gtins.json

# MaterialGroups contain a human-readyble name of the product group
echo "--- MATERIALGROUPS ---"
k4r_data_provider \
    get_materialgroups \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/materialgroups.json

echo "--- ITEMS ---"
k4r_data_provider \
    get_items \
    --data_source dtapi \
    --pks12_pw ./k4r_data/certificate/pw \
    --certificate_path ./k4r_data/certificate/sandbox-2022.p12 > dtapi_data/items.json

# TODO: facings?!