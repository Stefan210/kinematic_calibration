#! /bin/bash

CHAINS='larm'
NUMVALDATA=150
FOLDER='merged'
NUMCHAINS=`echo $CHAINS | wc -w`

CSVIN='comparison_error_per_iteration.csv'

# get the header
HEADER=`head -n 1 all_except_1/larm_10/$CSVIN`
#echo $HEADER

# append new columns to the header
# NUMOFMEASUREMENTS, OPTRMS, VALRMS, VALMINERROR, VALMINRMS, VALMINITERATION
#NEWHAEDER=`echo $HEADER | sed 's/\n/"NUMOFMEASUREMENTS\tOPTRMS\tVALRMS\tVALMINERROR\tVALMINRMS\tVALMINITERATION\n"/'`
NEWHEADER=${HEADER}$'\t'NUMOFMEASUREMENTS$'\t'OPTRMS$'\t'VALRMS$'\t'VALMINERROR$'\t'VALMINRMS$'\t'VALMINITERATION$'\t'FOLD

CSVRESULT='result.csv'
rm $CSVRESULT
echo NUMOFMEASUREMENTS$'\t'AVGRMS$'\t'AVGVAR >> $CSVRESULT

mkdir $FOLDER

for poses in `seq 5 50`
do
      CSVOUT="results_${poses}_measurements.csv"
      
      # write the header to the file
      echo $NEWHEADER > $CSVOUT
      
      #for fold in `seq 1 5`
      for fold in 1 2 3 4 5
      do
	    # get the last line
	    LINE=`tail -n 1 all_except_${fold}/larm_${poses}/$CSVIN | sed -e 's/[eE]+*/\\*10\\^/g'`
	    echo $LINE
	
	    # calculate new values:
	    NUMOFMEASUREMENTS=$poses
	    
	    # OPTRMS
	    OPTERROR=`echo $LINE | cut -d ' ' -f 2`      
	    OPTERROR=`echo ${OPTERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
	    OPTRMS=`echo "sqrt($OPTERROR / ($NUMOFMEASUREMENTS * $NUMCHAINS) )" | bc -l`

	    # VALRMS
	    VALERROR=`echo $LINE | cut -d ' ' -f 3` 
	    VALERROR=`echo ${VALERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
	    VALRMS=`echo "sqrt($VALERROR / $NUMVALDATA)" | bc -l`

	    VALMMINLINE=`awk 'NR == 1 || $3 < min {line = $0; min = $3}END{print line}' all_except_${fold}/larm_${poses}/$CSVIN`
      
	    # VALMINERROR
	    VALMINERROR=`echo $VALMMINLINE | cut -d ' ' -f 3`
	    VALMINERROR=`echo ${VALMINERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
      
	    # VALMINRMS
	    VALMINRMS=`echo "sqrt($VALMINERROR / $NUMVALDATA)" | bc -l`

	    # VALMINITERATION
	    VALMINITERATION=`echo $VALMMINLINE | cut -d ' ' -f 1`  
      
	    # append the new values to the line
	    NEWLINE=${LINE}$'\t'$NUMOFMEASUREMENTS$'\t'$OPTRMS$'\t'$VALRMS$'\t'$VALMINERROR$'\t'$VALMINRMS$'\t'$VALMINITERATION$'\t'$fold
	    
	    # write the line to the file
	    echo $NEWLINE >> $CSVOUT
      done
      
      AVGRMS=`awk '{mean += $19} END {print mean/(NR-1);}' $CSVOUT`
      VAR=`awk -v CONVFMT=%.28g '{sum+=$19; sumsq+=($19*$19);} END {print (sumsq/(NR-1) - (sum/(NR-1)*sum/(NR-1)));}' $CSVOUT`
      echo $poses$'\t'$AVGRMS$'\t'$VAR >> $CSVRESULT
      mv $CSVOUT $FOLDER/
done