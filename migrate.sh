#!/bin/sh
# auto migrator for Android
# run as sudo/tsu/su/root

FROM="/data/user/0/net.froemling.bombsquad/no_backup/ballistica_files/ba_data/meshes"
SAVE="/sdcard/Android/data/net.froemling.bombsquad/files/mods/Paths"
EXEC="python3"
ARGS="pathmaker.py"
TEMP="cob"
DONE="out"

rm -rf "$TEMP" > /dev/null 2>&1
mkdir "$TEMP"

for m in $(find "$FROM" | grep "LevelCollide")
  do cp "$m" "$TEMP"
done

"$EXEC" "$ARGS" "$TEMP"
rm -rf "$SAVE"
cp -r "$DONE" "$SAVE"
rm -rf "$TEMP" "$DONE"
