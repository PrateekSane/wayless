#!/bin/bash

# Usage: ./rename_bags.sh /path/to/bag_folder

if [ $# -ne 1 ]; then
  echo "Usage: $0 /path/to/bag_folder"
  exit 1
fi

cd "$1" || exit 1

echo "Renaming files in: $PWD"

for file in velo_*.bag; do
  # Skip if the file doesn't actually exist (handles no matches)
  [ -e "$file" ] || continue

  # Skip any active file that ends with .active
  if [[ "$file" == *.active ]]; then
    echo "Skipping active file: $file"
    continue
  fi

  # Extract the number
  num=$(echo "$file" | sed -E 's/velo_([0-9]+).bag/\1/')
  # Format with leading zeros to 2 digits
  newnum=$(printf "%02d" "$num")
  newname="velo_${newnum}.bag"

  if [ "$file" != "$newname" ]; then
    echo "Renaming $file -> $newname"
    mv "$file" "$newname"
  fi
done

echo "✅ Renaming done."
