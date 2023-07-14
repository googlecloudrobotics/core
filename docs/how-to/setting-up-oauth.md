# Setting up OAuth for web UIs

Estimated time: 5 min

When a user loads a web UI hosted in the cloud Kubernetes cluster, the server has to authenticate them before allowing them to use the service.
To enable this, you'll need to set up OAuth with the Cloud Console.
Once you've completed these steps, you'll be able to access services with web UIs, such as [Grafana](https://grafana.com/).


If you haven't already, complete the [Quickstart Guide](../quickstart.md) or [Deploy Cloud Robotics Core from sources](deploy-from-sources.md) to set up your GCP project.

## Create OAuth credentials

1. Open the [cloud console](https://console.cloud.google.com/) and ensure that
     your cloud project is selected in the project selector dropdown at the top.

1. Configure the OAuth consent screen: [APIs & Services → Credentials → OAuth consent screen](https://console.cloud.google.com/apis/credentials/consent).
   * User Type: Internal
   * Application name: My Cloud Robotics Application
   * Support email: *your email address*
   * Add `[PROJECT_ID].cloud.goog` to Authorized domains (where `[PROJECT_ID]` is your GCP project ID).
   * Leave the other fields blank.

1. Create an OAuth client ID: [APIs & Services → Credentials → Create credentials → OAuth client ID](https://console.cloud.google.com/apis/credentials/oauthclient).
   * Application type: Web application
   * Restrictions → Authorized JavaScript origins:<br/>
   `https://www.endpoints.[PROJECT_ID].cloud.goog`
   * Restrictions → Authorized redirect URIs: <br/>
   `https://www.endpoints.[PROJECT_ID].cloud.goog/oauth2/callback`
   * Click "Create".

You'll see a dialog containing the client ID and secret which we will add to your `config.sh` next.

## Update your config and redeploy

1. update your `config.sh` in the Google Cloud Storage bucket:
    ```shell
    curl -fS "https://storage.googleapis.com/cloud-robotics-releases/run-install.sh" >run-install.sh
    bash ./run-install.sh $PROJECT_ID --set-oauth
    ```
    Enter the OAuth client ID and secret from the previous step when asked.
1. Update your cloud project:
    ```shell
    bash ./run-install.sh $PROJECT_ID
    ```

After the update has been deployed, OAuth is enabled in your cloud project.
Verify that `oauth2-proxy` is running now:
```console
$ kubectl get pods

NAME               READY   STATUS    RESTARTS   AGE
...
oauth2-proxy-xxx   1/1     Running   0          1m
```

## Try it out

Open a web browser and visit `https://www.endpoints.[PROJECT_ID].cloud.goog/grafana/dashboards`, replacing `[PROJECT_ID]` with your GCP project ID.
You'll be prompted to log in with your Google account, after which you'll see a list of dashboards.
Try selecting "Kubernetes Capacity Planning" to see the resource usage of the Kubernetes cluster.
