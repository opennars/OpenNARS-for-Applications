#!/bin/bash
for filename in *_20FPS.mp4; do
  echo "file $filename" >> concat-list.txt
done

ffmpeg -f concat -i concat-list.txt ONAvideo.mp4

echo "Concatenated videos list:"
cat concat-list.txt
rm concat-list.txt
