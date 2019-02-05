import pika
import csv
import struct

HEADER_UNPACK_FORMAT = ">QhLhh"
HEADER_BYTE_LENGTH = 8 + 2 + 4 + 2 + 2
STATE_UNPACK_FORMAT = ""


class Ingestor():
    def __init__(self):
        self.init_connection()
        self.init_csv()
        return

    def init_connection(self):
        self.credentials = pika.PlainCredentials('robin', 'focker12')
        self.connection = pika.BlockingConnection(
            pika.ConnectionParameters('34.221.207.211', 5672, 'swarm', self.credentials))
        self.channel = self.connection.channel()
        self.result = self.channel.queue_declare(exclusive=True)
        self.queue_name = self.result.method.queue
        self.binding_key = "demo.key"

        self.channel.queue_bind(exchange='amq.topic',
                                queue=self.queue_name, routing_key=self.binding_key)
        print(' [*] Waiting for logs. To exit press CTRL+C')

    def init_csv(self):
        file = open('temp_test.csv', 'w')
        header = ['Temperature/C']
        with file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(header)
        return

    def callback(self, ch, method, properties, body):
        print(" [x] %r:%r" % (method.routing_key, body))
        header = body[:HEADER_BYTE_LENGTH]
        print(header)
        ts, ms, mid, dev_ID, message_type = struct.unpack(HEADER_UNPACK_FORMAT, body[:HEADER_BYTE_LENGTH])
        if message_type == 0x01:
            temp, x, y, battery = struct.unpack(">ffff", body[HEADER_BYTE_LENGTH:])

            self.csv_write(mid, dev_ID, temp, x, y, battery)

        return

    def csv_write(self, mid, dev_ID, temp, x, y, battery):
        file = open('temp_test.csv', 'a')
        header = ['MessageID', 'Device', 'Temperature', 'x', 'y', 'battery']

        with file:
            writer = csv.DictWriter(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL, fieldnames=header)
            writer.writeheader()
            writer.writerow(
                {"MessageID": mid, "Device": dev_ID, "Temperature": temp, "x": x, "y": y, "battery": battery})
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
