This is the bootloader default configuration.

depends on:
1. mcuboot : provide image verification
2. mbedtls : provide RSA and hash algorithm
3. special application : now the app is app_nsh
4. sign_img.sh : provide image sign

The default application image size is 0x38000,
So the second application image location is
0x10000 + 0x38000

