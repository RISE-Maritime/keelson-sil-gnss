# keelson-sil-gnss: a simulated GNSS receiver.
#
# Plain, and that is the point. Its predecessor lived inside a simulator's
# repository and had to be built with the parent directory as its context, so
# it could copy that simulator's kernel package in for one thing — the
# ellipsoid. The ellipsoid comes from pymap3d now, so this image is a package
# and its dependencies and nothing else.
#
# keelson installs from PyPI at the version pinned in pyproject.toml — never a
# local checkout or a hand-built tarball.
FROM python:3.12-slim

WORKDIR /app

COPY pyproject.toml README.md LICENSE ./
COPY keelson_sil_gnss ./keelson_sil_gnss

RUN pip install --no-cache-dir .

ENTRYPOINT ["keelson-sil-gnss"]
