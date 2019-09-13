
include_directories(${PROJECT_SOURCE_DIR}/wifibroadcast)
add_executable(tx_rawsock tx_rawsock.c fec.c lib.c)
target_link_libraries(tx_rawsock rt)
install(TARGETS tx_rawsock DESTINATION bin COMPONENT applications)

add_executable(rx rx.c fec.c lib.c radiotap.c)
target_link_libraries(rx rt pcap)
install(TARGETS rx DESTINATION bin COMPONENT applications)

add_executable(sharedmem_init_rx sharedmem_init_rx.c fec.c lib.c radiotap.c)
target_link_libraries(sharedmem_init_rx rt pcap)
install(TARGETS sharedmem_init_rx DESTINATION bin COMPONENT applications)

add_executable(sharedmem_init_tx sharedmem_init_tx.c fec.c lib.c radiotap.c)
target_link_libraries(sharedmem_init_tx rt pcap)
install(TARGETS sharedmem_init_tx DESTINATION bin COMPONENT applications)
