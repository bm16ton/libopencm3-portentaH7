#!/bin/bash
rm part3-properties
touch part3-properties
echo "struct property_entry generic_properties[] = {" > part3-properties
echo $1 >> part3-properties
echo $2 >> part3-properties
echo $3 >> part3-properties
echo $4 >> part3-properties
echo $5 >> part3-properties
echo $6 >> part3-properties
echo $7 >> part3-properties
echo $8 >> part3-properties
echo $9 >> part3-properties

echo "	{}" >> part3-properties
echo "};" >> part3-properties
sed -i '/^$/d' part3-properties
