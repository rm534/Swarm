config_firmware = {
            "wifi" : {
                "VM0134973_2GEXT": {
                    "user":"NA",
                    "pwd":"yn9ktYmxgdpp"
                },
                "Robin":{
                    "user":"NA",
                    "pwd":"Hello123"
                }
#                "eduroam": {
#                    "user":"rm534",
#                    "pwd":"2Be734"
#                },
#                "UoE_Guest": {
#                    "user":"NA",
#                    "pwd:":"NA"
#                },
#                "UoE_Open": {
#                    "user":"rm534",
#                    "pwd":"2Be734"
#                }
            },
            "mqtt" : {
                "user" : "mqtt",
                "pwd" : "test",
                "vhost" : "swarm",
                "port" : 5672,
                "server" : "35.164.26.30"
            },
            "device" : {
                "devid" : 0x25

            }


}
print(config_firmware["wifi"])
