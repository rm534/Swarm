import pika
import csv
import struct

HEADER_UNPACK_FORMAT = ">QhLhh"
HEADER_BYTE_LENGTH = 8 + 2 + 4 + 2 + 2
STATE_UNPACK_FORMAT = ""


class Ingestor():
    def __init__(self):
        self.count = 0
        self.data_dump = []
        self.init_connection()
        self.init_csv()
        return

    def init_connection(self):
        self.credentials = pika.PlainCredentials('robin', 'focker12')
        self.connection = pika.BlockingConnection(
            pika.ConnectionParameters('34.213.163.151', 5672, 'swarm', self.credentials))
        self.channel = self.connection.channel()
        self.result = self.channel.queue_declare(exclusive=True)
        self.queue_name = self.result.method.queue
        self.binding_key = "demo.key"

        self.channel.queue_bind(exchange='amq.topic',
                                queue=self.queue_name, routing_key=self.binding_key)
        print(' [*] Waiting for logs. To exit press CTRL+C')

    def init_csv(self):
        file = open('temp_test.csv', 'a')
        header = ['MessageID', 'Device', "ts", "ms", 'Temperature', 'x', 'y', 'battery']
        with file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(header)
            file.close()
        return

    def callback(self, ch, method, properties, body):
        #print(" [x] %r:%r" % (method.routing_key, body))
        header = body[:HEADER_BYTE_LENGTH]
        #print(header)
        ts, ms, mid, dev_ID, message_type = struct.unpack(HEADER_UNPACK_FORMAT, body[:HEADER_BYTE_LENGTH])
        print("[+] message received from device {}".format(dev_ID))
        if message_type == 0x01 and self.count <= 20:
            temp, x, y, battery = struct.unpack(">ffff", body[HEADER_BYTE_LENGTH:])
            data = [mid, dev_ID, ts, ms, temp, x, y, battery]
            print("[+] temp: {} x: {} y: {} battery: {}".format(temp, x, y, battery))
            self.data_dump.append(data)
            self.count += 1

        elif message_type == 0x02:
            msg = struct.unpack("{}s".format(len(body[HEADER_BYTE_LENGTH:])), body[HEADER_BYTE_LENGTH:])
            print("[+] debug message: {}".format(msg))

        if self.count > 20:
            file = open('Device: {}.csv'.format(str(dev_ID)), 'w')
            file.write("")
            file.close()
            print("[+] dumping data")
            for element in self.data_dump:
                self.csv_write(element[0], element[1], element[2], element[3], element[4], element[5], element[6],
                               element[7])
            self.count = 0
            self.data_dump = []




        return

    def csv_write(self, mid, dev_ID, ts, ms, temp, x, y, battery):
        file = open('Device: {}.csv'.format(str(dev_ID)), 'a')

        header = ['MessageID', 'Device', "ts", "ms", 'Temperature', 'x', 'y', 'battery']
        with file:
            writer = csv.DictWriter(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL, fieldnames=header)
            file.write(",,,,,,,\n")
            writer.writerow(
                {"MessageID": mid, "Device": dev_ID, "ts": ts, "ms": ms, "Temperature": temp, "x": x, "y": y,
                 "battery": battery})

        file.close()
        return

    def consume(self):
        self.channel.basic_consume(self.callback,
                                   queue=self.queue_name,
                                   no_ack=True)

        self.channel.start_consuming()
        return


if __name__ == '__main__':
    ingestor = Ingestor()
    ingestor.consume()
