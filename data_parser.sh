# Copyright 2015  Christoffer Heckman
#                 Steve McGuire
#                 Regents of the University of Colorado
#
# Released under the Apache 2.0 License
#
# this is a parser for data from a bunch of dat files in the format:
# # Vehicle state parameters
# 
# # Longitudinal vehicle acceleration in vcf m/s^2
# AccelX                          0.427791
# 
# # Lateral vehicle acceleration in vcf, m/s^2
# AccelY
# 
# [...]
# 
# TranRelX                       -1000.000
# 
# [...]
# 
# TranRelY                      -1000.000
# 
# [...]
# 
# RelYawRad                      4.00000
# filename convention: image_meta_NNNNNN.dat
#
# Output will be to a csv file with six columns, ordering:
# x,y,z,r,p,q
#

DATE=`date +%s`
FILENAME=data_$DATE.txt
touch $FILENAME

for i in `ls image_meta_*.dat` ; do
  TRANRELX=`grep TranRelX $i | sed 's/TranRelX//g' `;
  TRANRELY=`grep TranRelY $i | sed 's/TranRelY//g' `;
  RELYAWRAD=`grep RelYawRad $i | sed 's/RelYawRad//g' `;
  echo $TRANRELX,$TRANRELY,0,0,0,$RELYAWRAD >> $FILENAME;
done
