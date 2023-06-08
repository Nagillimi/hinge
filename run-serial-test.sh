#!/bin/bash
pip3 install -e ./
python3 '.\tests\serial_test.py'

echo
echo "Finished. You can close this window now."
while [ true ] ; do
read -t 3 -n 1
if [ $? = 0 ] ; then
exit ;
fi
done