This is the bootloader default configuration.

depends on:
1. mcuboot : provide image verification
2. mbedtls : provide RSA and hash algorithm
3. special application : now the app is app_nsh
4. sign_img.sh : provide image sign

