#! /bin/bash

CHAINS='larm'
NUMVALDATA=150
FOLDER='index'
NUMCHAINS=`echo $CHAINS | wc -w`

CSVIN='comparison_error_per_iteration.csv'
CSVOUT='index.csv'

# get the header
HEADER=`head -n 1 all_except_1/larm_10/$CSVIN`
#echo $HEADER

# append new columns to the header
# NUMOFMEASUREMENTS, OPTRMS, VALRMS, VALMINERROR, VALMINRMS, VALMINITERATION
#NEWHAEDER=`echo $HEADER | sed 's/\n/"NUMOFMEASUREMENTS\tOPTRMS\tVALRMS\tVALMINERROR\tVALMINRMS\tVALMINITERATION\n"/'`
NEWHEADER=NUMOFMEASUREMENTS$'\t'OPTRMS$'\t'VALRMS$'\t'VALMINERROR$'\t'VALMINRMS$'\t'VALMINITERATION$'\t'FOLD$'\t'OBSINDEXINIT$'\t'OBSINDEXEND

# write the header to the file
echo $NEWHEADER > $CSVOUT

for poses in `seq 5 50`
do      
      for fold in 1 2 3 4 5      
      do
	    # get the last line
	    LASTLINE=`tail -n 1 all_except_${fold}/larm_${poses}/$CSVIN`
	    OBSINDEXEND=`echo $LASTLINE | cut -d ' ' -f 16` 
	    
	    # calculate new values:
	    NUMOFMEASUREMENTS=$poses
	    
	    # OPTRMS
	    OPTERROR=`echo $LASTLINE | cut -d ' ' -f 2`      
	    OPTERROR=`echo ${OPTERROR} | sed -e 's/[eE]+*/\\*10\\^/'`
	    OPTRMS=`echo "sqrt($OPTERROR / ($NUMOFMEASUREMENTS * $NUMCHAINS) )" | bc -l`

	    # VALRMS
	    VALERROR=`echo $LASTLINE | cut -d ' ' -f 3` 
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
      
	    # get the first line
	    FIRSTLINE=`head -n 2 all_except_${fold}/larm_${poses}/$CSVIN | tail -n 1`
	    OBSINDEXINIT=`echo $FIRSTLINE | cut -d ' ' -f 16` 
      
	    # append the new values to the line
	    NEWLINE=$NUMOFMEASUREMENTS$'\t'$OPTRMS$'\t'$VALRMS$'\t'$VALMINERROR$'\t'$VALMINRMS$'\t'$VALMINITERATION$'\t'$fold$'\t'$OBSINDEXINIT$'\t'$OBSINDEXEND
	    
	    # write the line to the file
	    echo $NEWLINE >> $CSVOUT
	    #echo $NEWLINE
      done
done