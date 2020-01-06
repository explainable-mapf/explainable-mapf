#!/bin/bash
## path to test file
input="tests.txt"
timeout=100
echo "Starting tests..."

while IFS= read -r line
do
  ##printf 'Working on %s file...\n' "$line"
  ##echo "python ExplainablePlanning.py" "$line" "$timeout"
  runstring="ExplainablePlanning.py $line $timeout"
  echo $runstring
  python3 $runstring
done < "$input"

echo "Tests complete!"