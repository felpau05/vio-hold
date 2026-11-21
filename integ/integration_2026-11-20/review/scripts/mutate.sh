#!/bin/bash
# mutate.sh NAME FILE SED_EXPR : copy base, apply one change to FILE, show the changed line, run the new test.
R="${WORK:?set WORK to the work folder}"
name=$1; file=$2; expr=$3
rm -rf $R/mut/$name; cp -r $R/mut/base $R/mut/$name
sed -i "$expr" $R/mut/$name/$file
echo "== mutation $name: $file: sed '$expr'"
diff $R/mut/base/$file $R/mut/$name/$file | sed 's/^/   /' | head -8
if cmp -s $R/mut/base/$file $R/mut/$name/$file; then echo "   (sed changed nothing: mutation not applied)"; fi
cd $R/mut/$name && nice -n 5 ~/.venvs/vio/bin/python tests_apb/test_ground_tools.py 2>&1 | grep -E "FAIL |ALL PASSED|FAILED|Error" | grep -v "^  PASS" | sed 's/^/   /' | cut -c1-220
echo
