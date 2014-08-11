#! /bin/bash

#CHAINS='larm rarm lleg rleg'
#FOLDER_PREFIX="all"
#NUMVALDATA=2000

CHAINS='rarm'
FOLDER_PREFIX="rarm"
NUMVALDATA=500

NUMCHAINS=`echo $CHAINS | wc -w`

CSVIN='comparison_error_per_iteration.csv'
CSVOUT="${FOLDER_PREFIX}_results_100_iterations.csv"

# get the header
HEADER=`head -n 1 ${FOLDER_PREFIX}_100/$CSVIN`
#echo $HEADER

# append new columns to the header
# NUMOFMEASUREMENTS, OPTRMS, VALRMS, VALMINERROR, VALMINRMS, VALMINITERATION
#NEWHAEDER=`echo $HEADER | sed 's/\n/"NUMOFMEASUREMENTS\tOPTRMS\tVALRMS\tVALMINERROR\tVALMINRMS\tVALMINITERATION\n"/'`
NEWHEADER=${HEADER}$'\t'NUMOFMEASUREMENTS$'\t'OPTRMS$'\t'VALRMS$'\t'VALMINERROR$'\t'VALMINRMS$'\t'VALMINITERATION
echo $NEWHEADER > $CSVOUT

# write the header to the file

#for i in `seq 10 100`
for i in `seq 10 10 250`
do
      # get the last line
      LINE=`tail -n 1 ${FOLDER_PREFIX}_${i}/$CSVIN`
      #echo $LINE
  
      # calculate new values:
      NUMOFMEASUREMENTS=$i
      
      # OPTRMS
      OPTERROR=`echo $LINE | cut -d ' ' -f 2`      
      OPTERROR=`echo ${OPTERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
      OPTRMS=`echo "sqrt($OPTERROR / ($NUMOFMEASUREMENTS * $NUMCHAINS) )" | bc -l`

      # VALRMS
      VALERROR=`echo $LINE | cut -d ' ' -f 3` 
      VALERROR=`echo ${VALERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
      VALRMS=`echo "sqrt($VALERROR / $NUMVALDATA)" | bc -l`

      VALMMINLINE=`awk 'NR == 1 || $3 < min {line = $0; min = $3}END{print line}' ${FOLDER_PREFIX}_${i}/$CSVIN`
 
      # VALMINERROR
      VALMINERROR=`echo $VALMMINLINE | cut -d ' ' -f 3`
      VALMINERROR=`echo ${VALMINERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
 
      # VALMINRMS
      VALMINRMS=`echo "sqrt($VALMINERROR / $NUMVALDATA)" | bc -l`

      # VALMINITERATION
      VALMINITERATION=`echo $VALMMINLINE | cut -d ' ' -f 1`  
 
      # append the new values to the line
      NEWLINE=${LINE}$'\t'$NUMOFMEASUREMENTS$'\t'$OPTRMS$'\t'$VALRMS$'\t'$VALMINERROR$'\t'$VALMINRMS$'\t'$VALMINITERATION
      
      # write the line to the file
      echo $NEWLINE >> $CSVOUT
done