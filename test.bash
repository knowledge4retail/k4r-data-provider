#!/bin/bash
k4r_data_provider \
    get_stores \
    --data_source files
    
k4r_data_provider \
    knows_store \
    --store_name "0_Uni-Bremen" \
    --data_source files

k4r_data_provider \
    get_products \
    --store_name "0_Uni-Bremen" \
    --data_source files

k4r_data_provider \
    get_shelves \
    --store_id 0 \
    --data_source files

#k4r_data_provider \
#    get_stores \
#    --data_source dt_api \
#    --pks12_pw ./certificate/pw \
#    --certificate_path ./k4r_data/certificate/k4r-dev-keystore.p12