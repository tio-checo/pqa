idf_component_register(SRCS
     "daq.c" "pqa.c" "mcp3913.c" "time_sync.c" "rest_server.c"
                    INCLUDE_DIRS "." "include"
                    EMBED_TXTFILES server_root_cert.pem)
