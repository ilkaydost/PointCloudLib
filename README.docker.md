Dockerizing PointCloudLib

This repository includes Dockerfiles and a docker-compose manifest to build and run the C++ server and Next.js UI inside containers.

Prerequisites
- Docker (Engine) and docker-compose installed on your machine.
- Sufficient disk space and time: building PCL from source via vcpkg can take a long time (30+ minutes) and requires several GBs.

Quick start (build & run both services):

```powershell
# From project root
docker compose build --progress=plain
docker compose up
```

- The C++ server will be available at http://localhost:5050
- The Next.js UI will be available at http://localhost:3000

Notes and troubleshooting
- The server Dockerfile boots vcpkg and installs dependencies listed in `vcpkg.json`. The first build downloads and compiles many libraries and will take significant time and CPU.
- If you run into permission or missing package issues, inspect the build output and install additional apt packages into `Dockerfile.server` as needed.
- To rebuild only the server or UI:

```powershell
Docker / Rancher Desktop

This repository includes Dockerfiles and a compose manifest to build and run the C++ server and Next.js UI inside containers. If you use Rancher Desktop (containerd + nerdctl), the same compose file works with `nerdctl compose`.

Prerequisites
- Rancher Desktop installed and running (or Docker Engine). If you prefer Docker CLI compatibility, enable the "dockerd" (dockerd shim) option in Rancher Desktop settings; otherwise use `nerdctl`.
- Sufficient disk space and time: building PCL from source via vcpkg can take a long time (30+ minutes) and requires several GBs.

Quick start (Rancher Desktop / nerdctl)

PowerShell (Windows) — recommended when using Rancher Desktop:

```powershell
# From project root
nerdctl compose build --progress=plain
nerdctl compose up
```

Bash (macOS / Linux or WSL):

```bash
# From project root
nerdctl compose build --progress=plain
nerdctl compose up
```

Notes
- The C++ server Dockerfile uses vcpkg to install dependencies from `vcpkg.json`. The first build downloads and compiles many libraries (PCL etc.) and may take a long time.
- If you enabled Docker (dockerd) shim in Rancher Desktop you can also use `docker compose build` / `docker compose up`.
- Bind mounts on Windows may require enabling file sharing or running Rancher Desktop with access to your user files. If you run into path or permission errors, try enabling WSL integration and running the commands from a WSL shell.

Rebuild only a single service

```powershell
# Rebuild server
nerdctl compose build server
# Rebuild UI
nerdctl compose build ui
```

Run a single service

```powershell
# Run only the server
nerdctl compose up server
```

Cleanup

```powershell
nerdctl compose down --rmi all --volumes
```

Windows helper script

There is a small helper script at `scripts/rd-up.ps1` that runs the above commands using `nerdctl` from PowerShell. Usage:

```powershell
# Build and run (foreground)
.\scripts\rd-up.ps1
# Build and run detached
.\scripts\rd-up.ps1 -Detach
```

If you'd like, I can also add a `Makefile` or GitHub Actions configuration that caches `vcpkg` artifacts to speed up CI builds. Want me to add that?