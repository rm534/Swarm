import pika
import csv


class Ingestor():
    def __init__(self):
        self.init_connection()
        self.init_csv()
        return

    def init_connection(self):
        self.credentials = pika.PlainCredentials('robin', 'focker12')
        self.connection = pika.BlockingConnection(pika.ConnectionParameters('34.213.163.151', 5672, 'swarm', self.credentials))
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
        data = [body]
        self.csv_write(data)
        return

    def csv_write(self, data):
        file = open('temp_test.csv', 'a')
        header = ['Temperature/C']
        with file:
            writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(data)
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
