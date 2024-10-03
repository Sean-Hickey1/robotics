#!/bin/bash

# find the project parent folder related to the current file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# define release folder
SRC=$DIR
# DES="${DIR//-private/-release}"
DES="${DIR//-private/}"
echo "Destination: $DES"
if [ "$SRC" == "$DES" ]; then
  echo "Source and destination are equal!!!"
  exit 1  # Exit with status 1 (or any non-zero value to indicate an error)
fi
read -p "Do you want to proceed? (yes/no): " user_input
# Convert input to lowercase to handle variations like Yes, YES, yEs, etc.
user_input=$(echo "$user_input" | tr '[:upper:]' '[:lower:]')

# Check if the input is "yes"
if [ "$user_input" == "yes" ]; then
  echo "Proceeding with the script..."
  # Place the rest of your script here
else
  echo "Exiting the script."
  exit 0  # Exit with a status code 0 (success)
fi

### The magic starts here 
if [ ! -d "$DES" ]; then
    echo "going to clone into $DES"
    git clone ssh://git@gitlab.tuwien.ac.at:822/lva-mr/2024/project.git $DES
fi
cd $DES
git pull
make clone

echo $DES/ws02
rsync -av --exclude={.git,__pycache__} $SRC/ws02/src/mr $DES/ws02/src/
rm -rf $DES/ws02/src/mr-students
cd $DES/ws02/src/mr/
#rm exercises/01
rm -rf exercises/02 mr_viz
rm -rf exercises/03 mr_pf
rm -rf exercises/04 mr_ekf
rm -rf exercises/05
rm -rf exercises/06
cd $DES/ws02
