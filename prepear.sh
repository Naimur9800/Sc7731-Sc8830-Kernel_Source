#!/bin/sh
# chmod -R a+w ./kernel/arch/arm/mach-mmp/*.*
# cp -f ./kernel/product/$1/*.* ./kernel/arch/arm/mach-mmp

SRCE=./kernel/product/$1/$2
DST=./kernel/arch/arm/configs/$2
if [ -f $SRCE ]; then
  if [ -f $DST ]; then
      RESULT=`diff   $SRCE   $DST`
      if [   "$RESULT "   !=   " "   ];then
        cp -f $SRCE $DST
      fi
  else
        cp -f $SRCE $DST
  fi
elif [ -f ./kernel/product/common/$2 ]; then
  cp -f ./kernel/product/common/$2 $DST
fi

if [   "$2"   =   "sp7731gea_2341a-dt_defconfig"   ];then
   MACH_BOARD=sprd-scx35_sp7731gea_2341A
elif [   "$2"   =   "sp7731gea-dt_defconfig"   ];then
   MACH_BOARD=sprd-scx35_sp7731gea
elif [   "$2"   =   "sp7731gea_hd-dt_defconfig"   ];then
   MACH_BOARD=sprd-scx35_sp7731gea_hd
else
   MACH_BOARD=?????
fi

SRCE=./kernel/product/$1/$MACH_BOARD.dts
DST=./kernel/arch/arm/boot/dts/$MACH_BOARD.dts
if [ -f $SRCE ]; then
  if [ -f $DST ]; then
      RESULT=`diff   $SRCE   $DST`
      if [   "$RESULT "   !=   " "   ];then
        cp -f $SRCE $DST
      fi
  else
        cp -f $SRCE $DST
  fi
elif [ -f ./kernel/product/common/$MACH_BOARD.dts ]; then
  cp -f ./kernel/product/common/$MACH_BOARD.dts $DST
fi


SRCE=./kernel/product/$1/ontim_device.c
DST=./kernel/arch/arm/mach-sc/ontim_device.c
if [ -f $SRCE ]; then
  if [ -f $DST ]; then
      RESULT=`diff   $SRCE   $DST`
      if [   "$RESULT "   !=   " "   ];then
        cp -f $SRCE $DST
      fi
  else
        cp -f $SRCE $DST
  fi
elif [ -f ./kernel/product/common/ontim_device.c ]; then
  cp -f ./kernel/product/common/ontim_device.c $DST
fi

#SRCE=./kernel/product/$1/cust_battery_para.h
#DST=./kernel/drivers/power/cust_battery_para.h
#if [ -f $SRCE ]; then
#  if [ -f $DST ]; then
#      RESULT=`diff   $SRCE   $DST`
#      if [   "$RESULT "   !=   " "   ];then
#        cp -f $SRCE $DST
#      fi
#  else
#        cp -f $SRCE $DST
#  fi
#elif [ -f ./kernel/product/common/cust_battery_para.h ]; then
#  cp -f ./kernel/product/common/cust_battery_para.h $DST
#fi

