source .config

if [ x$IMG_VERSION == x ] ; then
  echo "NO define IMG_VERSION , will use default value 1.0.0+100"
  echo  -e "\tUsage:"
  echo  -e "\t 1. export IMG_VERSION=x.x.x+xxx"
  echo  -e "\t \t version is major.minus.rev+build"
  echo  -e "\t \t option: "
  echo  -e "\t \t   export IMG_START=xxxxx"
  echo  -e "\t \t \t  this is the location of image in flash"
  echo  -e "\t \t   export IMG_LEN=xxxx"
  echo  -e "\t \t \t  this is the total length of signed image"
  echo  -e "\t 2. $0 [ private_key.pem ]\n\n"

  IMG_VERSION="1.0.0+100"
else
  echo "Image_Version: $IMG_VERSION "
fi

if [ x$IMG_START == x ]; then
  if [ ! -z $CONFIG_FLASH_ORIGIN ] ; then
    echo "Flash_Img_Addr : $CONFIG_FLASH_ORIGIN "
    let SIGN_IMAGE_ADDR=(CONFIG_FLASH_ORIGIN-0x200)
   echo "Sign_Img_Addr : $SIGN_IMAGE_ADDR"
  else
   echo "There is NO define CONFIG_FLASH_ORIGIN"
   exit 1
  fi
else
  SIGN_IMAGE_ADDR=$IMG_START
  echo "Sign_Img_Addr : $SIGN_IMAGE_ADDR"
fi

# actually : flash_length include: header + image + magic
if [ x$IMG_LEN == x ]; then
  if [ ! -z $CONFIG_FLASH_ORIGIN ] ; then
    echo "Sign_Img_Len : $CONFIG_FLASH_LENGTH "
    SIG_IMG_LEN=$CONFIG_FLASH_LENGTH
  else
   echo "There is NO define CONFIG_FLASH_LENGTH"
   exit 1
  fi
else
  SIG_IMG_LEN=$IMG_LEN
  echo "Sign_Img_Len : $CONFIG_FLASH_LENGTH "
fi

EXTERNAL_DIR=external
echo "IMG_VERSION : $IMG_VERSION"

echo $#
if [ $# -eq 1 ]; then
  PRIVATE_KEY=$1
else
  PRIVATE_KEY=$EXTERNAL_DIR/key/root_rsa_2048.pem
fi

echo "The private key is $PRIVATE_KEY"
# sign image with rsa private key
$EXTERNAL_DIR/mcuboot/scripts/imgtool.py sign -k $PRIVATE_KEY --align 4 -v $IMG_VERSION -H 0x200 --pad $SIG_IMG_LEN  nuttx.bin nuttx.sign.bin

srec_cat nuttx.sign.bin -BINary -offset $SIGN_IMAGE_ADDR -o nuttx.sign.hex  -intel

exit 0
