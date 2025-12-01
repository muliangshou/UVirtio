#!/bin/bash
# use this script to read the `.feature file` generated from `kconfig-mconf `, which  further parse Kfeature to build the menu 
# this script will genearte a more structured yaml file,
# the json file would further be sent to a decision tree for kernel selection.

VERSION=0.1.0

function generate_requirement_file()
{
    local SELECTOR_NAME=${1}

    # destination file using file descriptor 8
    exec 8>${2}

    echo -ne "version: ${VERSION}\n" >&8

    EMPTY_LINE='true'

    while read LN
    do
        LINE=`echo $LN | sed 's/[ \t\r\n]*$//g'`

        if [ -z "$LINE" ]; then
            continue
        fi

        if [ '#' = ${LINE:0:1} ]; then
            if [ ${#LINE} -eq 1 ]; then
                # empty line
                if $EMPTY_LINE; then
                    continue
                fi
                echo >&8
                EMPTY_LINE='true'
                continue
            fi
            
            if echo -ne "$LINE" | egrep '^# Required' >/dev/null 2>/dev/null; then 
                # the key in yaml
                echo -ne "${LINE:11}:\n" >&8
            else
                LINE=${LINE:1}
                echo -ne "# ${LINE}\n" >&8
            fi

            EMPTY_LINE='false'
        else
            EMPTY_LINE='false'

            OLD_IFS="$IFS"
            IFS='='
            REQUIREMENTS=($LINE)
            IFS="$OLD_IFS"

            if [ ${#REQUIREMENTS[@]} -ge 2 ]; then
                if echo -ne "$REQUIREMENTS[0]" | egrep '^CONFIG_' >/dev/null 2>/dev/null; then
                    REQUIREMENTS[0]="${REQUIREMENTS[0]:7}"
                fi

                if [ "${REQUIREMENTS[1]}" = 'y' ]; then
                    echo -ne " ${REQUIREMENTS[0]}: true\n" >&8
                else
                    echo -ne " ${REQUIREMENTS[0]}: ${LINE#*=}\n" >&8
                fi
            fi
        fi

    done < $SELECTOR_NAME

    exec 8<&-
}

generate_requirement_file $1 $BSP_ROOT/requirement.yaml
