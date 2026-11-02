# Private Debug Intake

This service is temporary debug infrastructure, not part of px4xplane production.
It requires Python 3.10+ and aiohttp (already installed on the target server).

Deploy `upload_server.py` to `/opt/px4xplane-debug-upload/` and the service unit to
`/etc/systemd/system/`. Supply a root-owned mode-0600 EnvironmentFile at
`/etc/px4xplane-debug-upload.env` with generated values for:

```text
PX4_DEBUG_UPLOAD_TOKEN=<random upload-only secret, at least 32 characters>
PX4_DEBUG_UPLOAD_EXPIRES=<UTC Unix timestamp, default seven days>
PX4_DEBUG_UPLOAD_ROOT=/var/lib/px4xplane-debug-upload
```

The existing HTTPS Caddy site proxies only `/px4xplane-debug/upload` to
`127.0.0.1:18764`. No port is exposed directly. POST accepts a Bearer token,
Content-Length and X-Content-SHA256; GET/download/listing are unavailable.
An authenticated HEAD with X-Upload-Bytes and X-Content-SHA256 checks size,
quota and free disk before the large upload. It returns 204 when admissible,
not a reservation: POST repeats all checks. A 507 is reported clearly by the
client instead of streaming into an already-rejected request/TLS EOF.
Limit: 256 MiB/object, 512 MiB total spool, 384 MiB free-disk reserve, one writer,
180-second request deadline. Wrong checksums and partial uploads are rejected.
Archives are stored by checksum and NEVER extracted. No automatic deletion of
unreviewed evidence. Upload repeats are idempotent.

Client credentials live outside the bundle in
`~/.config/px4xplane-debug/upload.json` (0600). Configure with the hidden token
prompt; do not place secrets in Git, public downloads, CLI arguments or flight
archives. The token is sent to the owner separately through the private Telegram
bridge. Do not include its value in reports.

After a test, the next reviewing agent uses authorized SSH to list JSON receipts
under `/var/lib/px4xplane-debug-upload`, retrieves the matching SHA-named ZIP and
analyzes it. The user no longer needs to copy results manually. Only remove an
archive after analysis and retention approval. If full, the endpoint returns 507
and the client retains a local pending upload.

To disable during production preparation: `bash run.sh disable-upload`, stop and
disable `px4xplane-debug-upload.service`, and remove only this scoped proxy route.
Do not remove unrelated Caddy routes or Telegram services.
