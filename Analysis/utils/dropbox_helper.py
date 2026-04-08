"""Dropbox authentication and file-access helpers for BeeSpy localApp.

All dropbox-SDK imports are deferred inside functions so the rest of the app
works even when the 'dropbox' package is not installed.
"""
import json
import os
from io import BytesIO

CONFIG_PATH = os.path.expanduser("~/.beespy_dropbox.json")


def load_config() -> dict:
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as f:
            return json.load(f)
    return {}


def save_config(data: dict):
    with open(CONFIG_PATH, "w") as f:
        json.dump(data, f, indent=2)


def get_saved_client():
    """Return an authenticated Dropbox client from saved tokens, or None."""
    try:
        import dropbox
        cfg = load_config()
        if not cfg.get("app_key") or not cfg.get("refresh_token"):
            return None
        dbx = dropbox.Dropbox(
            app_key=cfg["app_key"],
            oauth2_refresh_token=cfg["refresh_token"],
        )
        dbx.users_get_current_account()   # verify the token is still valid
        return dbx
    except Exception:
        return None


def start_oauth(app_key: str):
    """Begin PKCE OAuth2 flow.  Returns (flow, authorize_url)."""
    from dropbox.oauth import DropboxOAuth2FlowNoRedirect
    flow = DropboxOAuth2FlowNoRedirect(
        app_key, use_pkce=True, token_access_type="offline"
    )
    return flow, flow.start()


def finish_oauth(flow, code: str, app_key: str):
    """Exchange auth code for tokens, persist them, return authenticated client."""
    import dropbox
    result = flow.finish(code.strip())
    cfg = load_config()
    cfg["app_key"] = app_key
    cfg["refresh_token"] = result.refresh_token
    save_config(cfg)
    return dropbox.Dropbox(
        app_key=app_key,
        oauth2_refresh_token=result.refresh_token,
    )


def scope_to_root_namespace(dbx):
    """Return a Dropbox client scoped to the root (team) namespace if applicable.

    For Dropbox Business/Teams accounts the shared folders live in the team
    namespace, not the user's personal namespace.  Calling
    files_list_folder("") on an unscoped client won't show them.
    Scoping to root_namespace_id makes all team folders visible.

    For personal accounts root_namespace_id == home_namespace_id, so this
    is a no-op and the original client is returned.
    """
    try:
        import dropbox
        account = dbx.users_get_current_account()
        root_ns = account.root_info.root_namespace_id
        home_ns = account.root_info.home_namespace_id
        if str(root_ns) != str(home_ns):
            path_root = dropbox.common.PathRoot.namespace_id(str(root_ns))
            return dbx.with_path_root(path_root)
    except Exception:
        pass
    return dbx


def list_shared_folders(dbx) -> list:
    """Return SharedFolderMetadata for every folder shared with/by this account.

    These are returned by the Sharing API regardless of namespace scoping, so
    this works even when files_list_folder("") can't see the folder.
    Only entries that have a path_lower (i.e. mounted in the user's Dropbox)
    are returned.
    """
    try:
        result = dbx.sharing_list_folders(actions=[])
        entries = list(result.entries)
        while result.cursor:
            result = dbx.sharing_list_folders_continue(result.cursor)
            entries.extend(result.entries)
        return [e for e in entries if getattr(e, "path_lower", None)]
    except Exception:
        return []


def list_folder(dbx, path: str) -> list:
    """Return all entries (FolderMetadata / FileMetadata) in a Dropbox folder."""
    norm = "" if path in ("/", "") else path
    result = dbx.files_list_folder(norm)
    entries = list(result.entries)
    while result.has_more:
        result = dbx.files_list_folder_continue(result.cursor)
        entries.extend(result.entries)
    return entries


def download_to_bytes(dbx, path: str) -> BytesIO:
    """Download a Dropbox file and return it as an in-memory BytesIO object."""
    _, response = dbx.files_download(path)
    return BytesIO(response.content)
