import asyncio
import sys
import time
import uuid

from src.python.networking.messages import HelloMessage
from src.python.networking.serialization import LaserTagJSONEncoder


class Session:
    def __init__(self, reader, writer, remote_endpoint, message_handler):
        self.remote_endpoint = remote_endpoint
        self._reader = reader
        self._writer = writer
        self._message_handler = message_handler

    def send(self, obj):
        """
        Sends an object as a JSON-message using the LaserTag-compliant wire format.
        """
        try:
            json_string = LaserTagJSONEncoder().encode(obj)
            json_bytes = json_string.encode("utf-8")
            total_size = 4 + 1 + len(json_bytes)  # size prefix + 'J' + JSON payload
            message_bytes = (
                int(total_size).to_bytes(length = 4, byteorder = "little")
                + b"J"
                + json_bytes
            )
            self._writer.write(message_bytes)
            #TODO: drain writer here
            #print("Sent %d bytes" % len(message_bytes))
        except:
            # TODO: why is exception never caught here?
            print("Exception caught")
            exit()

    async def _run(self):
        while True:
            # Read header and payload
            try:
                size_prefix = await self._reader.readexactly(4)
                timestamp = time.time()
                total_size = int.from_bytes(bytes = size_prefix, byteorder = "little")
                payload_size = total_size - 4
                payload = await self._reader.readexactly(payload_size)
                #print("Received %d bytes" % total_size)
                if total_size > 5:
                  # A JSON payload exists
                  json_string = payload[1:].decode("utf-8")
                  self._message_handler.handle_message(session = self, json_string = json_string, timestamp = timestamp)
            except:
                print(
                    "Disconnected from %s: %s"
                    % (self.remote_endpoint, sys.exc_info())
                )
                break


class Server:
    def __init__(self, message_handler):
        self.id = str(uuid.uuid1())
        self.sessions = []
        self._message_handler = message_handler

    async def on_client_connect(self, reader, writer):
        # Construct a client object
        remote_endpoint = writer.get_extra_info("socket")
        remote_endpoint = (remote_endpoint if remote_endpoint is not None else "unknown endpoint")
        print("New connection from %s" % remote_endpoint)
        session = Session(reader = reader, writer = writer, remote_endpoint = remote_endpoint, message_handler = self._message_handler)
        session.send(HelloMessage(id = self.id, app_type = "Server", message = "Hello from Python server!"))
        self._message_handler.on_connect(session = session)
        self.sessions.append(session)
        await session._run()
        self.sessions.remove(session)
        print("Ended session with %s" % remote_endpoint)
        self._message_handler.on_disconnect(session = session)

    async def run_server(self):
        server = await asyncio.start_server(client_connected_cb = self.on_client_connect, host = None, port = 6810)
        async with server:
            print("Starting server...")
            await server.serve_forever()
