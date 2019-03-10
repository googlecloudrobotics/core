from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import signal
import sys
import time
import uuid


class Controller(BaseHTTPRequestHandler):
  def sync(self, parent, children):
    """Actuate a ChargeAction custom resource.

    The resource is actuated by using the Charge Service to send the robot to a
    charger. While charging is in progress, the status of the resource is
    updated to reflect the charge level.

    Because the ChargeAction is handled by an external service, this controller
    doesn't create any child resources, ie the `children` list is empty.

    Args:
      parent: The current ChargeAction resource.
      children: Unused.

    Returns:
      A dict containing the latest status of the action, and an empty list of
      children, eg:
      {
        "status": {
          "state": "OK",
        },
        "children": [],
      }
    """

    # Get current status and copy to start building next status.
    current_status = parent.get("status", None) or {}
    desired_status = dict(current_status)
    state = current_status.get("state", "CREATED")

    if state == "CREATED":
      # The ChargeAction has just been created. Use the external Charge Service
      # to start charging. Store the request ID in the status so we can use it
      # to check the state of the charge request.
      request_id = self.charge_service.start_charging()
      desired_status["state"] = "IN_PROGRESS"
      desired_status["request_id"] = request_id

    elif state == "IN_PROGRESS":
      try:
        # Get the progress of the charge request from the external service.
        progress = self.charge_service.get_progress(
            current_status["request_id"])
        desired_status["charge_level_percent"] = progress

        if progress == 100:
          # Charging has completed.
          desired_status["state"] = "OK"

      except ValueError as e:
        # The charge request was not found. This could be because the robot was
        # restarted during a charge, and the request was forgotten.
        desired_status["state"] = "ERROR"
        desired_status["message"] = str(e)

    elif state in ["OK", "CANCELLED", "ERROR"]:
      # Terminal state, do nothing.
      pass

    else:
      desired_status["state"] = "ERROR"
      desired_status["message"] = "Unrecognized state: %r" % state

    return {"status": desired_status, "children": []}

  def do_POST(self):
    """Serve the sync() function as a JSON webhook."""
    observed = json.loads(self.rfile.read(int(self.headers["content-length"])))
    desired = self.sync(observed["parent"], observed["children"])

    self.send_response(200)
    self.send_header("Content-type", "application/json")
    self.end_headers()
    self.wfile.write(json.dumps(desired).encode('utf-8'))


class ChargeService(object):
  """ChargeService wraps an external API that send the robot to a charger.

  For this example, it just fakes the charging process.
  """

  SECONDS_FOR_FULL_CHARGE = 10

  def __init__(self):
    self._requests = {}

  def start_charging(self):
    request_id = str(uuid.uuid4())
    self._requests[request_id] = time.time()
    return request_id

  def get_progress(self, request_id):
    if request_id not in self._requests:
      raise ValueError("invalid request ID")

    charge_time = time.time() - self._requests[request_id]
    if charge_time > self.SECONDS_FOR_FULL_CHARGE:
      return 100
    else:
      return int(100 * (charge_time / self.SECONDS_FOR_FULL_CHARGE))


# Terminate process when Kubernetes sends SIGTERM.
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

Controller.charge_service = ChargeService()
HTTPServer(("", 8000), Controller).serve_forever()
