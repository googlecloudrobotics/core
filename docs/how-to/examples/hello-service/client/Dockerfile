FROM python:alpine

RUN pip install --no-cache-dir google-auth requests

WORKDIR /data

COPY client.py ./

CMD [ "python", "-u", "./client.py" ]
