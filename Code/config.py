config_firmware = {
            "wifi" : {
                "VM0134973_2GEXT": {
                    "user":"NA",
                    "pwd":"yn9ktYmxgdpp"
                },
                "eduroam": {
                    "user":"rm534",
                    "pwd":"2Be734"
                }
            },
            "mqtt" : {
                "user" : "mqtt",
                "pwd" : "test",
                "vhost" : "swarm",
                "port" : 5672,
                "server" : "35.164.26.30"
                }
}
print(config_firmware["mqtt"])
