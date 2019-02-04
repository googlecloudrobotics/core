# Setting up OAuth for web UIs

When a user loads a web UI hosted in the cloud Kubernetes cluster, the server has to authenticate them before allowing them to use the service.
To enable this, you'll need to set up OAuth with the Cloud Console.
Once you've completed these steps, you'll be able to access services with web UIs, such as [Grafana](https://grafana.com/).

Estimated time: 5 min

1. If you haven't already, complete the [Setting up the GCP project](../quickstart.md) steps.

1. Open the [cloud console](https://console.cloud.google.com/) and ensure that
     your cloud project is selected in the project selector dropdown at the top.

1. Configure the OAuth consent screen: [APIs & Services → Credentials → OAuth consent screen](https://console.cloud.google.com/apis/credentials/consent).
   * Application name: My Cloud Robotics Application
   * Support email: *your email address*
   * Add `www.endpoints.[PROJECT_ID].cloud.goog` to Authorized domains (where `[PROJECT_ID]` is your GCP project ID).
   * Leave the other fields blank.

1. Create an OAuth client ID: [APIs & Services → Credentials → Create credentials → OAuth client ID](https://console.cloud.google.com/apis/credentials/oauthclient).
   * Application type: Web application
   * Restrictions → Authorized JavaScript origins:<br/>
   `https://www.endpoints.[PROJECT_ID].cloud.goog`
   * Restrictions → Authorized redirect URIs: <br/>
   `https://www.endpoints.[PROJECT_ID].cloud.goog/oauth2/callback`
   * Click "Create".

    You'll see a dialog containing the client ID and secret. Use these to adjust the following lines in `config.sh`:

    ```
    CLOUD_ROBOTICS_OAUTH2_CLIENT_ID=[CLIENT_ID]
    CLOUD_ROBOTICS_OAUTH2_CLIENT_SECRET=[CLIENT_SECRET]
    ```

1. Run the following command to create a secret for encrypting cookies:

    ```
    python -c 'import os,base64; print base64.urlsafe_b64encode(os.urandom(16))'
    ```

    Use this to adjust the following line in `config.sh`:

    ```
    CLOUD_ROBOTICS_COOKIE_SECRET=[COOKIE_SECRET]
    ```

1. Update the deployment.

    ```
    ./deploy.sh update
    ```

After `deploy.sh` completes, OAuth is enabled in your cloud project.

## Try it out

Open a web browser and visit `https://www.endpoints.[PROJECT_ID].cloud.goog/grafana/dashboards`, replacing `[PROJECT_ID]` with your GCP project ID.
You'll be prompted to log in with your Google account, after which you'll see a list of dashboards.
Try selecting "Kubernetes Capacity Planning" to see the resource usage of the Kubernetes cluster.
