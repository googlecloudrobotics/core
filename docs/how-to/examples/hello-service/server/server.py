from http import server

class MyRequestHandler(server.BaseHTTPRequestHandler):
  def do_GET(self):
    print('Received a request')
    self.send_response(200)
    self.send_header('Content-Type', 'text/plain')
    self.end_headers()
    self.wfile.write(b'Server says hello!\n')

def main():
  server_address = ('', 8000)
  httpd = server.HTTPServer(server_address, MyRequestHandler)
  httpd.serve_forever()

if __name__ == '__main__':
  main()
