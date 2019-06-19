#for i in `ls regressions/configfiles/*dor*`;do ./booksim $i | grep "Overall average network latency" ;done | awk '{print $6}'
#for i in `ls regressions/configfiles/*adap*`;do ./booksim $i | grep "Overall average network latency" ;done | awk '{print $6}'
for i in `ls regressions/configfiles/*dbar*`;do ./booksim $i | grep "Overall average network latency" ;done | awk '{print $6}'
