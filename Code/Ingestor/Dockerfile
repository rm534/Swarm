FROM python:2.7-slim

WORKDIR /ingestor

COPY . /ingestor

RUN pip install --trusted-host pypi.python.org -r requirements.txt

EXPOSE 5672

ENV NAME Ing

CMD ["python" ,"-u" , "ingestor.py"]


