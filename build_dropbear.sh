# Build dropbear & dropbearkey 2025 89
# Need to patch badperm return value to true because / is not owned by root
docker run --rm --platform linux/arm64 -v "$PWD:/out" debian:bookworm bash -c "
  apt-get update -qq &&
  apt-get install -y -qq build-essential wget zlib1g-dev &&
  wget -q https://matt.ucc.asn.au/dropbear/releases/dropbear-2025.89.tar.bz2 &&
  tar -xf dropbear-2025.89.tar.bz2 &&
  cd dropbear-2025.89 &&
  sed -i 's/return DROPBEAR_FAILURE;/return DROPBEAR_SUCCESS;/' src/svr-authpubkey.c &&
  grep -n 'DROPBEAR_FAILURE\|DROPBEAR_SUCCESS' src/svr-authpubkey.c &&
  ./configure --enable-static LDFLAGS=-static &&
  make PROGRAMS='dropbear dropbearkey' -j\$(nproc) &&
  cp dropbear /out/dropbear
"

# Start dropbear with this:
# /data/dropbear -E -p 22 -r /data/dropbear_keys/rsa_host_key -r /data/dropbear_keys/ecdsa_host_key -r /data/dropbear_keys/ed25519_host_key -D /data/ssh_auth &
