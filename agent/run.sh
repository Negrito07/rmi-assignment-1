#!/bin/bash

challenge="1"
host="localhost"
robname="theAgent"
pos="0"
outfile="solution"

while getopts "c:h:r:p:f:" op
do
    case $op in
        "c")
            challenge=$OPTARG
            ;;
        "h")
            host=$OPTARG
            ;;
        "r")
            robname=$OPTARG
            ;;
        "p")
            pos=$OPTARG
            ;;
        "f")
            outfile=$OPTARG
            ;;
        default)
            echo "ERROR in parameters"
            ;;
    esac
done

shift $(($OPTIND-1))

case $challenge in
    1)
        # how to call agent for challenge 1
	    # activate virtual environment, if needed
        python3 c1Rob.py -h "$host" -p "$pos" -r "$robname"
        ;;
    2)
        # how to call agent for challenge 2
        python3 c2Rob.py -h "$host" -p "$pos" -r "$robname"
        mv mapping.out $outfile.map             # if needed
        ;;
    3)
        # how to call agent for challenge 3
        # python3 mainC1.py -h "$host" -p "$pos" -r "$robname"
        # mv your_pathfile $outfile.path           # if needed
        ;;
    4)
        # how to call agent for challenge 4
        python3 c4Rob.py -h "$host" -p "$pos" -r "$robname"
        mv mapping.out $outfile.map            # if needed
        # mv path.out $outfile.path              # if needed
        ;;
esac

