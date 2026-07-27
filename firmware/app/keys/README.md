# MCUboot signing keys

## `mcuboot_dev_ec_p256.pem` — **DEVELOPMENT ONLY. NOT A PRODUCTION KEY.**

This is an ECDSA P-256 private key generated with `imgtool keygen` and committed
deliberately so that any developer can produce a locally-bootable image. It is
public by construction: **anything signed with it must be treated as unsigned.**

* Used by: `sysbuild.conf` → `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE`.
* MCUboot embeds the matching public key at build time, so a bootloader built
  from this tree will only accept images signed with this key.
* Rotating it invalidates every previously-flashed development image.

## Production key

The production signing key is an **offline** ECDSA P-256 key. It is never
committed, never present in CI, and never installed on a developer machine.
Production images are signed out-of-band and the production MCUboot is built
with `SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` pointed at the corresponding public
key material supplied at build time, e.g.

```sh
west build -b sts1000_meridian --sysbuild firmware/app -- \
    -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE=\"/secure/path/sts1000_prod_ec_p256.pem\"
```

Do not add a production key, a path to one, or a passphrase to this repository.

## Regenerating the development key

```sh
imgtool keygen -k mcuboot_dev_ec_p256.pem -t ecdsa-p256
```

Every device carrying an image signed by the previous development key must be
re-flashed over SWD after a regeneration; MCUboot serial recovery will refuse
the new images until the bootloader itself is replaced.
