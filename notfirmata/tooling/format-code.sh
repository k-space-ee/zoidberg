#!/bin/bash

# Simple script for code formatting in 1tbs
# See http://astyle.sourceforge.net/astyle.html for syntax and defaults

MINPARAMS=1
ORIG_SUFFIX=orig

if [ $# -lt "$MINPARAMS" ]
then
    echo "This script needs C source files passed as arguments"
    echo "USAGE: format-code.sh src/main.c src/somecode.c ..."
    exit 1
fi

#
for FILE in "$@"
do
    RESULT="$(astyle    --style=1tbs \
                        --indent-col1-comments \
                        --pad-oper \
                        --pad-header \
                        --add-brackets \
                        --convert-tabs \
                        --max-code-length=80 \
                        --break-after-logical \
                        --mode=c \
                        --suffix=.$ORIG_SUFFIX \
                        --lineend=linux \
                        $FILE)"

    # if file unchanged print unchanged result message
    if [[ "$RESULT" = Unchanged* ]]
    then
        echo "$RESULT"
    fi

    # if file formatted print result and renamed file name
    if [[ "$RESULT" = Formatted* ]]
    then
        echo "$RESULT"
        echo "Original code was preserved in file $FILE.$ORIG_SUFFIX"
    fi
done

exit 0
