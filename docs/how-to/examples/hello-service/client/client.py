import google.auth
import google.auth.transport.requests as requests

credentials, project_id = google.auth.default()

authed_session = requests.AuthorizedSession(credentials)

response = authed_session.request(
  "GET", "https://www.endpoints.[PROJECT_ID].cloud.goog/apis/hello-server")

print(response.status_code, response.reason, response.text)
