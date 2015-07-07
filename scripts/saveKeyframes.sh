#!/bin/bash

# Database username and table name for postgresql database. 
# Presupposes that the database is set up this way.
DB_USER="benjamin"
TABLENAME="keyframe"

# Database names.
ESM_DATABASE="ESM_KEYFRAMES"
FIND_ONLY_DATABASE="FIND_ONLY_KEYFRAMES"
FIND_MATCH_DATABASE="FIND_MATCH_KEYFRAMES"
NOW=$(date +"%h_%d_%y_%H-%M-%Ss")

ESM_KF_DIR="ESMkeyframes"
FIND_KF_DIR="findOnlyImages"
FIND_MATCH_KF_DIR="findMatchImages"
# Location in which to save the backups. 
BACKUP_DIR="$HOME/catkin_ws/src/backup/$NOW"
# The location of the saved images from the respective databases.
IMG_SAVE_DIR="$HOME/catkin_ws/src/savedImages"

# Make a backup directory, linked to the time when the script was run. 
# This way I don't have to put the date in every file I save. 
pushd .
# Find and make a backup directory. 
if ! [ -d "$BACKUP_DIR" ]; then
	echo Creating time-specific backup directory.
	mkdir -p $BACKUP_DIR
#	cd $BACKUP_DIR
fi

if [ -d "$IMG_SAVE_DIR" ]; then
	cd $IMG_SAVE_DIR
else
	# No point in making the save directories. 
	echo Not found ":" image save directory. Exiting.
	exit
fi

#### Take care of ESM Database ####
if [ -d "$ESM_KF_DIR" ]; then
	cd $ESM_KF_DIR
	ESM_FOUND=1
else
	echo Not found ":" ESM directory
	ESM_FOUND=0
fi

# Check if there are actually any files in the ESM_KF_DIR. If not,
# then move on to the next directory.
if [ "$(ls -A $IMG_SAVE_DIR/$ESM_KF_DIR)" ] && [ $ESM_FOUND = 1 ]; then
	ESM_EMPTY=0
else
	echo ESM Directory is empty.
	ESM_EMPTY=1
fi

# If the directory is NOT empty, then save off the pictures in a zip directory
# and dump the database to a file. Then move the backups and delete the files. 
if [ $ESM_EMPTY = 0 ] && [ $ESM_FOUND = 1 ]; then # This directory isn't empty and it exists
	ZIP_NAME=esm_pictures.zip
	echo $ZIP_NAME
	zip -r $ZIP_NAME *
	DB_NAME="esm.db"
	pg_dump $ESM_DATABASE > $DB_NAME
	psql --username $DB_USER $ESM_DATABASE -c "DELETE FROM $TABLENAME"
	mv $ZIP_NAME $BACKUP_DIR
	mv $DB_NAME $BACKUP_DIR
	rm $IMG_SAVE_DIR/$ESM_KF_DIR/*  # Putting in the full path for safety.
fi

#### Take care of FIND_ONLY database, similar procedure #### 
cd $IMG_SAVE_DIR

if [ -d "$FIND_KF_DIR" ]; then
	cd $FIND_KF_DIR
	FIND_FOUND=1
else
	echo Not found ": find only directory"
	FIND_FOUND=0
fi

# Check if there are actually any files in the ESM_KF_DIR. If not,
# then move on to the next directory.
if [ "$(ls -A $IMG_SAVE_DIR/$FIND_KF_DIR)" ] && [ $FIND_FOUND = 1 ]; then
	FIND_EMPTY=0
else
	echo Find only directory is empty.
	FIND_EMPTY=1
fi

# If the directory is NOT empty, then save off the pictures in a zip directory
# and dump the database to a file. Then move the backups and delete the files. 
if [ $FIND_EMPTY = 0 ] && [ $FIND_FOUND = 1 ]; then # This directory isn't empty and it exists
#	echo Not empty
	ZIP_NAME=find_only_pictures.zip
	echo $ZIP_NAME
	zip -r $ZIP_NAME *
	DB_NAME="find_only.db"
	pg_dump $FIND_ONLY_DATABASE > $DB_NAME
	psql --username $DB_USER $FIND_ONLY_DATABASE -c "DELETE FROM $TABLENAME"
	mv $ZIP_NAME $BACKUP_DIR
	mv $DB_NAME $BACKUP_DIR
	rm $IMG_SAVE_DIR/$FIND_KF_DIR/*  # Putting in the full path for safety.
fi

#### Take care of FIND_MATCH database, similar procedure #### 
cd $IMG_SAVE_DIR

if [ -d "$FIND_MATCH_KF_DIR" ]; then
	cd $FIND_MATCH_KF_DIR
	FIND_MATCH_FOUND=1
else
	echo Not found ": find match directory"
	FIND_MATCH_FOUND=0
fi

# Check if there are actually any files in the ESM_KF_DIR. If not,
# then move on to the next directory.
if [ "$(ls -A $IMG_SAVE_DIR/$FIND_MATCH_KF_DIR)" ] && [ $FIND_MATCH_FOUND = 1 ]; then
	FIND_MATCH_EMPTY=0
else
	echo Find match directory is empty.
	FIND_MATCH_EMPTY=1
fi

# If the directory is NOT empty, then save off the pictures in a zip directory
# and dump the database to a file. Then move the backups and delete the files. 
if [ $FIND_MATCH_EMPTY = 0 ] && [ $FIND_MATCH_FOUND = 1 ]; then # This directory isn't empty and it exists
#	echo Not empty
	ZIP_NAME=find_match_pictures.zip
	echo $ZIP_NAME
	zip -r $ZIP_NAME *
	DB_NAME="find_match.db"
	pg_dump $FIND_MATCH_DATABASE > $DB_NAME
	psql --username $DB_USER $FIND_MATCH_DATABASE -c "DELETE FROM $TABLENAME"
	mv $ZIP_NAME $BACKUP_DIR
	mv $DB_NAME $BACKUP_DIR
	rm $IMG_SAVE_DIR/$FIND_MATCH_KF_DIR/*  # Putting in the full path for safety.
fi

popd
