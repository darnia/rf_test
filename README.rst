# Install nrfutil

```console
# pip2 install --user nrfutil
```

If nrfutil already installed for python3 - remove it at once.

# Compile and flash with DNF
To compile and flash with nrfutil DNF:

```console
 # cd build/
 # cmake -GNinja -DBOARD=nrf52840_pca10059 ..
 # ninja
 # nrfutil pkg generate --hw-version 52 --sd-req 0x00 --application-version 1 --application zephyr/zephyr.hex update.zip
 # nrfutil dfu usb-serial -pkg update.zip -p /dev/ttyACM0
```
