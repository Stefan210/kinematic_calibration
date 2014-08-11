#! /bin/bash

#CHAINS='larm rarm lleg rleg'
#FOLDER_PREFIX="all"
#NUMVALDATA=2000

CHAINS='larm'
FOLDER_PREFIX="larm"
NUMVALDATA=500

NUMCHAINS=`echo $CHAINS | wc -w`

CSVIN='comparison_error_per_iteration.csv'
CSVOUT="${FOLDER_PREFIX}_results_100_iterations_noise"

# get the header
HEADER=`head -n 1 ${FOLDER_PREFIX}_noise_1_pixel_rk_true/$CSVIN`
#echo $HEADER

# append new columns to the header
# NUMOFMEASUREMENTS, OPTRMS, VALRMS, VALMINERROR, VALMINRMS, VALMINITERATION
#NEWHAEDER=`echo $HEADER | sed 's/\n/"NUMOFMEASUREMENTS\tOPTRMS\tVALRMS\tVALMINERROR\tVALMINRMS\tVALMINITERATION\n"/'`
NEWHEADER=${HEADER}$'\t'NUMOFMEASUREMENTS$'\t'OPTRMS$'\t'VALRMS$'\t'VALMINERROR$'\t'VALMINRMS$'\t'VALMINITERATION$'\t'RANDPERCENTAGE$'\t'RK

for rk in "true" "false"
do
      echo $NEWHEADER > ${CSVOUT}_${rk}.csv
done

# write the header to the file

for i in `seq 0 10`
#for i in 90 100
do
      for rk in "true" "false"
      do
	    FOLDERNAME=${FOLDER_PREFIX}_noise_${i}_pixel_rk_${rk}
	    
	    # get the last line
	    LINE=`tail -n 1 ${FOLDERNAME}/$CSVIN`
	    #echo $LINE
	
	    # calculate new values:
	    NUMOFMEASUREMENTS=100
	    
	    # OPTRMS
	    OPTERROR=`echo $LINE | cut -d ' ' -f 2`      
	    OPTERROR=`echo ${OPTERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
	    OPTRMS=`echo "sqrt($OPTERROR / ($NUMOFMEASUREMENTS * $NUMCHAINS) )" | bc -l`

	    # VALRMS
	    VALERROR=`echo $LINE | cut -d ' ' -f 3` 
	    VALERROR=`echo ${VALERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
	    VALRMS=`echo "sqrt($VALERROR / $NUMVALDATA)" | bc -l`

	    VALMMINLINE=`awk 'NR == 1 || $3 < min {line = $0; min = $3}END{print line}'  ${FOLDERNAME}/$CSVIN`
      
	    # VALMINERROR
	    VALMINERROR=`echo $VALMMINLINE | cut -d ' ' -f 3`
	    VALMINERROR=`echo ${VALMINERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
      
	    # VALMINRMS
	    VALMINRMS=`echo "sqrt($VALMINERROR / $NUMVALDATA)" | bc -l`

	    # VALMINITERATION
	    VALMINITERATION=`echo $VALMMINLINE | cut -d ' ' -f 1`  
      
	    # RANDPERCENTAGE
	    RANDPERCENTAGE=$i
	    
	    
	    # append the new values to the line
	      NEWLINE=${LINE}$'\t'$NUMOFMEASUREMENTS$'\t'$OPTRMS$'\t'$VALRMS$'\t'$VALMINERROR$'\t'$VALMINRMS$'\t'$VALMINITERATION$'\t'$RANDPERCENTAGE$'\t'$rk
	    
	    # write the line to the file
	    echo $NEWLINE >> ${CSVOUT}_${rk}.csv
      done
done