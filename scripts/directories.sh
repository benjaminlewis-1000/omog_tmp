#! /bin/bash

# Assumes that the database is all set up, because I don't want to deal with that.

ESM_KF_DIR="ESMkeyframes"
FIND_KF_DIR="findOnlyImages"
FIND_MATCH_KF_DIR="findMatchImages"
# The location of the saved images from the respective databases.
IMG_SAVE_DIR="$HOME/catkin_ws/src/savedImages"

if [ -d "$IMG_SAVE_DIR" ]; then
	cd $IMG_SAVE_DIR
else
	mkdir -p $IMG_SAVE_DIR
	echo Creating image save directory.
	cd $IMG_SAVE_DIR
fi

if [ -d "$ESM_KF_DIR" ]; then
	echo "ESM dir exists"
else
	mkdir -p $ESM_KF_DIR
	echo Creating ESM KF directory.
fi

if [ -d "$FIND_KF_DIR" ]; then
	echo "FIND dir exists"
else
	mkdir -p $FIND_KF_DIR
	echo Creating FIND KF directory.
fi

if [ -d "$FIND_MATCH_KF_DIR" ]; then
	echo "Find Match dir exists"
else
	mkdir -p $FIND_MATCH_KF_DIR
	echo Creating Find-match KF directory.
fi
